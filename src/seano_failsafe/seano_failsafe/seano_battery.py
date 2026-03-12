#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import serial
import json
import threading


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
        
        self.serial_port = self.get_parameter('failsafe.battery.serial_port').value
        self.baudrate = self.get_parameter('failsafe.battery.baudrate').value
        self.check_interval = self.get_parameter('failsafe.battery.check_interval').value
        self.min_voltage = self.get_parameter('failsafe.battery.min_voltage').value
        self.max_voltage = self.get_parameter('failsafe.battery.max_voltage').value
        self.low_voltage = self.get_parameter('failsafe.battery.low_voltage_threshold').value
        self.critical_voltage = self.get_parameter('failsafe.battery.critical_voltage_threshold').value
        
        # Battery state
        self.current_voltage = 0.0
        self.current_current = 0.0
        self.current_percentage = 0.0
        self.serial_connected = False
        
        # Serial connection
        self.serial_conn = None
        self.serial_thread = None
        self.running = True
        
        # Initialize serial connection
        self.connect_serial()
        
        # Timer for publishing status
        self.timer = self.create_timer(self.check_interval, self.publish_status)
        
        self.get_logger().info('SEANO Battery Monitor Node Started')
        self.get_logger().info(f'Serial Port: {self.serial_port} @ {self.baudrate}')
        self.get_logger().info(f'Voltage Range: {self.min_voltage}V - {self.max_voltage}V')
        self.get_logger().info(f'Low Voltage: {self.low_voltage}V | Critical: {self.critical_voltage}V')
        self.get_logger().info('SEANO Battery Monitor Node Started')
        self.get_logger().info(f'Serial Port: {self.serial_port} @ {self.baudrate}')
        self.get_logger().info(f'Voltage Range: {self.min_voltage}V - {self.max_voltage}V')
        self.get_logger().info(f'Low Voltage: {self.low_voltage}V | Critical: {self.critical_voltage}V')
    
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
        if not self.serial_connected:
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
        if self.current_voltage >= self.max_voltage * 0.98:
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
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SeanoBatteryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
