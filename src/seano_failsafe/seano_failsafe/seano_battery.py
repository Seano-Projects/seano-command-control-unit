#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32, Bool, String
import serial
import json
import threading
import time
import math
import paho.mqtt.client as mqtt


class SeanoBatteryNode(Node):
    def __init__(self):
        super().__init__('seano_battery')
        
        # Publishers
        self.battery_voltage_pub = self.create_publisher(Float32, '/seano/battery/voltage', 10)
        self.battery_current_pub = self.create_publisher(Float32, '/seano/battery/current', 10)
        self.battery_percentage_pub = self.create_publisher(Float32, '/seano/battery/percentage', 10)
        self.battery_power_pub = self.create_publisher(Float32, '/seano/battery/power', 10)
        self.battery_status_pub = self.create_publisher(String, '/seano/battery/status', 10)
        self.battery_low_alert_pub = self.create_publisher(Bool, '/seano/battery/low_alert', 10)
        
        # Parameters
        self.declare_parameter('failsafe.battery.serial_port', '/dev/ttyTHS0')
        self.declare_parameter('failsafe.battery.baudrate', 115200)
        self.declare_parameter('failsafe.battery.check_interval', 1.0)
        self.declare_parameter('failsafe.battery.min_voltage', 10.5)  # Minimum voltage before critical
        self.declare_parameter('failsafe.battery.max_voltage', 12.6)  # Maximum voltage (full charge)
        self.declare_parameter('failsafe.battery.low_voltage_threshold', 11.1)  # Low voltage warning
        self.declare_parameter('failsafe.battery.critical_voltage_threshold', 10.8)  # Critical voltage
        self.declare_parameter('failsafe.battery.simulation_enabled', True)
        self.declare_parameter('failsafe.battery.simulation_timeout', 5.0)
        self.declare_parameter('failsafe.battery.use_internal_dummy', False)
        self.declare_parameter('failsafe.battery.battery_id', 1)
        self.declare_parameter('failsafe.battery.cell_count', 6)
        self.declare_parameter('failsafe.battery.base_temperature_c', 31.0)
        self.declare_parameter('failsafe.battery.temp_rise_per_amp', 0.75)
        self.declare_parameter('failsafe.battery.discharge_rate_pct_per_min', 0.35)
        self.declare_parameter('failsafe.battery.min_current', 2.0)
        self.declare_parameter('failsafe.battery.max_current', 5.8)

        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', False)
        
        self.serial_port = self.get_parameter('failsafe.battery.serial_port').value
        self.baudrate = self.get_parameter('failsafe.battery.baudrate').value
        self.check_interval = self.get_parameter('failsafe.battery.check_interval').value
        self.min_voltage = self.get_parameter('failsafe.battery.min_voltage').value
        self.max_voltage = self.get_parameter('failsafe.battery.max_voltage').value
        self.low_voltage = self.get_parameter('failsafe.battery.low_voltage_threshold').value
        self.critical_voltage = self.get_parameter('failsafe.battery.critical_voltage_threshold').value
        self.simulation_enabled = self.get_parameter('failsafe.battery.simulation_enabled').value
        self.simulation_timeout = self.get_parameter('failsafe.battery.simulation_timeout').value
        self.use_internal_dummy = self.get_parameter('failsafe.battery.use_internal_dummy').value
        self.battery_id = int(self.get_parameter('failsafe.battery.battery_id').value)
        self.cell_count = max(1, int(self.get_parameter('failsafe.battery.cell_count').value))
        self.base_temperature_c = float(self.get_parameter('failsafe.battery.base_temperature_c').value)
        self.temp_rise_per_amp = float(self.get_parameter('failsafe.battery.temp_rise_per_amp').value)
        self.discharge_rate_pct_per_min = float(
            self.get_parameter('failsafe.battery.discharge_rate_pct_per_min').value
        )
        self.min_current_draw = float(self.get_parameter('failsafe.battery.min_current').value)
        self.max_current_draw = float(self.get_parameter('failsafe.battery.max_current').value)

        self.vehicle_id = str(self.get_parameter('vehicle.id').value)
        self.mqtt_broker = str(self.get_parameter('mqtt.broker').value)
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = str(self.get_parameter('mqtt.username').value)
        self.mqtt_password = str(self.get_parameter('mqtt.password').value)
        self.mqtt_base_topic = str(self.get_parameter('mqtt.base_topic').value)
        self.mqtt_qos = int(self.get_parameter('mqtt.qos').value)
        self.mqtt_keepalive = int(self.get_parameter('mqtt.keepalive').value)
        self.mqtt_use_tls = bool(self.get_parameter('mqtt.use_tls').value)
        self.mqtt_tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)
        self.simulation_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/simulation/battery'
        self.battery_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/battery'
        
        # Battery state
        self.current_voltage = 0.0
        self.current_current = 0.0
        self.current_percentage = 0.0
        self.current_temperature = self.base_temperature_c
        self.serial_connected = False

        # Internal dummy generator state
        self.dummy_start_ts = time.time()
        self.dummy_percentage = 100.0

        # Simulation override state
        self.simulation_override_active = False
        self.last_simulation_ts = 0.0
        self.simulation_status = 'disabled'
        
        # Serial connection
        self.serial_conn = None
        self.serial_thread = None
        self.running = True

        # MQTT
        self.mqtt_client = None
        
        # Initialize serial connection unless internal dummy mode is enabled.
        if self.use_internal_dummy:
            self.get_logger().warn('Internal dummy battery mode enabled (no serial hardware required)')
        else:
            self.connect_serial()

        # Initialize MQTT simulation listener
        if self.simulation_enabled:
            self.setup_mqtt_simulation()
        
        # Timer for publishing status
        self.timer = self.create_timer(self.check_interval, self.publish_status)
        
        self.get_logger().info('SEANO Battery Monitor Node Started')
        self.get_logger().info(f'Serial Port: {self.serial_port} @ {self.baudrate}')
        self.get_logger().info(f'Voltage Range: {self.min_voltage}V - {self.max_voltage}V')
        self.get_logger().info(f'Low Voltage: {self.low_voltage}V | Critical: {self.critical_voltage}V')
        self.get_logger().info(f'Internal Dummy Mode: {self.use_internal_dummy}')
        self.get_logger().info(
            f'Battery Simulation: {self.simulation_enabled} | Topic: {self.simulation_topic}'
        )
        self.get_logger().info(f'Battery MQTT Topic: {self.battery_topic}')

    def setup_mqtt_simulation(self):
        """Setup MQTT client for battery simulation input."""
        try:
            self.mqtt_client = mqtt.Client()

            if self.mqtt_username:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)

            if self.mqtt_use_tls:
                self.mqtt_client.tls_set()
                self.mqtt_client.tls_insecure_set(self.mqtt_tls_insecure)

            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.mqtt_client.loop_start()

            self.simulation_status = 'mqtt_connected'
            self.get_logger().info(
                f'MQTT simulation listener connected to {self.mqtt_broker}:{self.mqtt_port}'
            )
        except Exception as e:
            self.simulation_status = 'mqtt_disconnected'
            self.get_logger().error(f'Failed to start MQTT simulation listener: {str(e)}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Subscribe to simulation topic when MQTT is connected."""
        if rc == 0:
            client.subscribe(self.simulation_topic, qos=self.mqtt_qos)
            self.simulation_status = 'mqtt_connected'
            self.get_logger().info(f'Subscribed simulation topic: {self.simulation_topic}')
        else:
            self.simulation_status = 'mqtt_disconnected'
            self.get_logger().error(f'MQTT connect failed with code {rc}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Track MQTT disconnect state."""
        self.simulation_status = 'mqtt_disconnected'
        if rc != 0:
            self.get_logger().warn('MQTT simulation listener disconnected unexpectedly')

    def on_mqtt_message(self, client, userdata, msg):
        """Handle simulation battery payload from MQTT."""
        try:
            payload_raw = msg.payload.decode('utf-8')
            payload = json.loads(payload_raw)
            self.apply_simulation_data(payload)
        except Exception as e:
            self.get_logger().warn(f'Invalid battery simulation payload: {str(e)}')

    def generate_internal_dummy_data(self):
        """Generate realistic battery discharge values without hardware."""
        elapsed_min = max(0.0, (time.time() - self.dummy_start_ts) / 60.0)
        self.dummy_percentage = max(0.0, 100.0 - (elapsed_min * self.discharge_rate_pct_per_min))
        soc = self.dummy_percentage / 100.0

        # Piecewise discharge profile: flat top, steady middle, sharp bottom.
        if soc >= 0.8:
            normalized_v = 0.92 + ((soc - 0.8) / 0.2) * 0.08
        elif soc >= 0.2:
            normalized_v = 0.30 + ((soc - 0.2) / 0.6) * 0.62
        else:
            normalized_v = (soc / 0.2) * 0.30

        self.current_voltage = self.min_voltage + normalized_v * (self.max_voltage - self.min_voltage)

        load_wave = 0.5 + 0.5 * math.sin(elapsed_min * 0.9)
        self.current_current = self.min_current_draw + (
            (self.max_current_draw - self.min_current_draw) * load_wave
        )

        temp_load = self.base_temperature_c + (self.current_current - self.min_current_draw) * self.temp_rise_per_amp
        temp_soc = (1.0 - soc) * 2.0
        self.current_temperature = temp_load + temp_soc

        self.current_percentage = self.dummy_percentage

    def build_battery_payload(self, status):
        """Build battery telemetry payload in MQTT JSON format."""
        cell_voltage = self.current_voltage / float(self.cell_count)
        cell_voltages = []
        for idx in range(self.cell_count):
            imbalance = (idx - (self.cell_count - 1) / 2.0) * 0.003
            cell_voltages.append(round(cell_voltage + imbalance, 3))

        return {
            'battery_id': self.battery_id,
            'percentage': round(self.current_percentage, 1),
            'voltage': round(self.current_voltage, 2),
            'current': round(self.current_current, 2),
            'temperature': round(self.current_temperature, 1),
            'status': status,
            'cell_voltages': cell_voltages,
            'cell_count': self.cell_count,
        }

    def publish_battery_mqtt(self, status):
        """Publish battery payload to seano/<vehicle>/battery topic."""
        if self.mqtt_client is None:
            return
        try:
            payload = self.build_battery_payload(status)
            self.mqtt_client.publish(
                self.battery_topic,
                json.dumps(payload),
                qos=self.mqtt_qos,
                retain=False,
            )
        except Exception as e:
            self.get_logger().warn(f'Failed publish battery MQTT: {str(e)}')

    def apply_simulation_data(self, payload):
        """Apply simulated battery values to override serial data temporarily."""
        if not isinstance(payload, dict):
            raise ValueError('payload must be JSON object')

        if 'voltage' not in payload:
            raise ValueError('payload must include voltage')

        voltage = float(payload['voltage'])
        current = float(payload.get('current', self.current_current))
        percentage = payload.get('percentage')

        self.current_voltage = voltage
        self.current_current = current

        if percentage is None:
            self.current_percentage = self.calculate_percentage(self.current_voltage)
        else:
            self.current_percentage = max(0.0, min(100.0, float(percentage)))

        self.simulation_override_active = True
        self.last_simulation_ts = time.time()
        self.simulation_status = 'simulation_active'

        self.get_logger().warn(
            f'SIM BATTERY UPDATE: {self.current_voltage:.2f}V | {self.current_current:.2f}A | '
            f'{self.current_percentage:.1f}%'
        )

    def is_simulation_active(self):
        """Check whether simulation override is still active within timeout."""
        if not self.simulation_override_active:
            return False

        if (time.time() - self.last_simulation_ts) <= self.simulation_timeout:
            return True

        self.simulation_override_active = False
        self.simulation_status = 'mqtt_connected'
        self.get_logger().info('Battery simulation timeout reached, back to serial sensor data')
        return False
    
    def connect_serial(self):
        """Connect to ESP32 via serial"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self.serial_connected = True
            self.get_logger().info(f'Connected to ESP32 on {self.serial_port}')
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.serial_thread.start()
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {str(e)}')
            self.serial_connected = False
    
    def read_serial_loop(self):
        """Continuously read from serial in separate thread"""
        while self.running and self.serial_connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.parse_battery_data(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {str(e)}')
                self.attempt_reconnect()
    
    def parse_battery_data(self, data):
        """Parse JSON data from ESP32
        Expected format: {"voltage": 12.5, "current": 2.3}
        """
        # Keep simulation values authoritative until timeout
        if self.is_simulation_active():
            return

        try:
            battery_data = json.loads(data)
            
            if 'voltage' in battery_data:
                self.current_voltage = float(battery_data['voltage'])
            
            if 'current' in battery_data:
                self.current_current = float(battery_data['current'])
            
            # Calculate percentage based on voltage
            self.current_percentage = self.calculate_percentage(self.current_voltage)
            
        except json.JSONDecodeError:
            # Try parsing simple format: "V:12.5,A:2.3"
            try:
                parts = data.split(',')
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':')
                        if key.strip().upper() == 'V':
                            self.current_voltage = float(value.strip())
                        elif key.strip().upper() == 'A':
                            self.current_current = float(value.strip())
                
                self.current_percentage = self.calculate_percentage(self.current_voltage)
            except Exception as e:
                self.get_logger().debug(f'Failed to parse battery data: {data}')
    
    def calculate_percentage(self, voltage):
        """Calculate battery percentage from voltage"""
        if voltage >= self.max_voltage:
            return 100.0
        elif voltage <= self.min_voltage:
            return 0.0
        else:
            # Linear interpolation
            percentage = ((voltage - self.min_voltage) / 
                         (self.max_voltage - self.min_voltage)) * 100.0
            return max(0.0, min(100.0, percentage))
    
    def attempt_reconnect(self):
        """Attempt to reconnect to serial"""
        self.serial_connected = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
        
        self.get_logger().warn('Attempting to reconnect to ESP32...')
        import time
        time.sleep(2)
        self.connect_serial()
    
    def publish_status(self):
        """Publish battery status"""
        simulation_active = self.is_simulation_active()

        if self.use_internal_dummy:
            self.generate_internal_dummy_data()

        if not self.use_internal_dummy and not self.serial_connected and not simulation_active:
            status_msg = String()
            status_msg.data = 'disconnected'
            self.battery_status_pub.publish(status_msg)
            return
        
        # Publish voltage
        voltage_msg = Float32()
        voltage_msg.data = self.current_voltage
        self.battery_voltage_pub.publish(voltage_msg)
        
        # Publish current
        current_msg = Float32()
        current_msg.data = self.current_current
        self.battery_current_pub.publish(current_msg)
        
        # Publish percentage
        percentage_msg = Float32()
        percentage_msg.data = self.current_percentage
        self.battery_percentage_pub.publish(percentage_msg)
        
        # Publish power (V * A)
        power_msg = Float32()
        power_msg.data = self.current_voltage * self.current_current
        self.battery_power_pub.publish(power_msg)
        
        # Determine status
        if self.use_internal_dummy:
            status = 'discharging'
        elif simulation_active:
            status = 'simulation'
        elif self.current_voltage >= self.max_voltage * 0.98:
            status = 'full'
        elif self.current_voltage <= self.critical_voltage:
            status = 'critical'
        elif self.current_voltage <= self.low_voltage:
            status = 'low'
        else:
            status = 'normal'
        
        status_msg = String()
        status_msg.data = status
        self.battery_status_pub.publish(status_msg)
        self.publish_battery_mqtt(status)
        
        # Publish low alert
        low_alert_msg = Bool()
        if self.current_voltage <= self.critical_voltage:
            low_alert_msg.data = True
            self.get_logger().warn(
                f'CRITICAL BATTERY: {self.current_voltage:.2f}V ({self.current_percentage:.1f}%) - FAILSAFE REQUIRED!'
            )
        elif self.current_voltage <= self.low_voltage:
            low_alert_msg.data = True
            self.get_logger().warn(
                f'Low Battery Warning: {self.current_voltage:.2f}V ({self.current_percentage:.1f}%)'
            )
        else:
            low_alert_msg.data = False
        
        self.battery_low_alert_pub.publish(low_alert_msg)
        
        # Log info periodically (every 10 seconds)
        if int(self.get_clock().now().seconds_nanoseconds()[0]) % 10 == 0:
            self.get_logger().info(
                f'Battery: {self.current_voltage:.2f}V | {self.current_current:.2f}A | '
                f'{self.current_percentage:.1f}% | {status.upper()}'
            )
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.running = False

        if self.mqtt_client is not None:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception:
                pass

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SeanoBatteryNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
