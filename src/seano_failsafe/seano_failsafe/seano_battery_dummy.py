#!/usr/bin/env python3

import json
import math
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node


class SeanoBatteryDummyNode(Node):
    def __init__(self):
        super().__init__('seano_battery_dummy')

        # Core dummy profile
        self.declare_parameter('failsafe.battery_dummy.publish_interval', 1.0)
        self.declare_parameter('failsafe.battery_dummy.battery_id', 1)
        self.declare_parameter('failsafe.battery_dummy.start_percentage', 100.0)
        self.declare_parameter('failsafe.battery_dummy.discharge_rate_pct_per_min', 0.35)
        self.declare_parameter('failsafe.battery_dummy.max_voltage', 14.6)
        self.declare_parameter('failsafe.battery_dummy.min_voltage', 11.2)
        self.declare_parameter('failsafe.battery_dummy.min_current', 2.0)
        self.declare_parameter('failsafe.battery_dummy.max_current', 5.8)
        self.declare_parameter('failsafe.battery_dummy.cell_count', 4)
        self.declare_parameter('failsafe.battery_dummy.base_temperature_c', 31.0)
        self.declare_parameter('failsafe.battery_dummy.temp_rise_per_amp', 0.75)

        # Shared identity and MQTT config
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('mqtt.broker', 'localhost')
        self.declare_parameter('mqtt.port', 1883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', True)

        self.publish_interval = float(self.get_parameter('failsafe.battery_dummy.publish_interval').value)
        self.battery_id = int(self.get_parameter('failsafe.battery_dummy.battery_id').value)
        self.start_percentage = float(self.get_parameter('failsafe.battery_dummy.start_percentage').value)
        self.discharge_rate_pct_per_min = float(
            self.get_parameter('failsafe.battery_dummy.discharge_rate_pct_per_min').value
        )
        self.max_voltage = float(self.get_parameter('failsafe.battery_dummy.max_voltage').value)
        self.min_voltage = float(self.get_parameter('failsafe.battery_dummy.min_voltage').value)
        self.min_current = float(self.get_parameter('failsafe.battery_dummy.min_current').value)
        self.max_current = float(self.get_parameter('failsafe.battery_dummy.max_current').value)
        self.cell_count = max(1, int(self.get_parameter('failsafe.battery_dummy.cell_count').value))
        self.base_temperature_c = float(self.get_parameter('failsafe.battery_dummy.base_temperature_c').value)
        self.temp_rise_per_amp = float(self.get_parameter('failsafe.battery_dummy.temp_rise_per_amp').value)

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

        self.battery_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/battery'
        self.simulation_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/simulation/battery'

        self.start_time = time.time()
        self.last_percentage = min(100.0, max(0.0, self.start_percentage))
        self.mqtt_connected = False

        self.client = mqtt.Client()
        if self.mqtt_username:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        if self.mqtt_use_tls:
            self.client.tls_set()
            self.client.tls_insecure_set(self.mqtt_tls_insecure)

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect

        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.client.loop_start()
        except Exception as err:
            self.get_logger().error(f'Failed to connect MQTT: {err}')

        self.timer = self.create_timer(self.publish_interval, self.publish_dummy_payload)

        self.get_logger().info('SEANO battery dummy generator started')
        self.get_logger().info(
            f'Topic battery={self.battery_topic} | simulation={self.simulation_topic} | '
            f'voltage={self.min_voltage:.2f}-{self.max_voltage:.2f}V'
        )

    def on_connect(self, _client, _userdata, _flags, rc):
        self.mqtt_connected = rc == 0
        if self.mqtt_connected:
            self.get_logger().info(f'MQTT connected to {self.mqtt_broker}:{self.mqtt_port}')
        else:
            self.get_logger().error(f'MQTT connect failed with code {rc}')

    def on_disconnect(self, _client, _userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            self.get_logger().warn('MQTT disconnected unexpectedly')

    def calc_percentage(self, elapsed_sec: float) -> float:
        drop = self.discharge_rate_pct_per_min * (elapsed_sec / 60.0)
        pct = self.start_percentage - drop
        pct = max(0.0, min(100.0, pct))
        # Make sure the trend is monotonic descending.
        self.last_percentage = min(self.last_percentage, pct)
        return self.last_percentage

    def calc_voltage(self, percentage: float) -> float:
        soc = max(0.0, min(1.0, percentage / 100.0))
        # A mild non-linear discharge curve: flatter near full, steeper near empty.
        curve = (0.12 * soc) + (0.88 * (soc ** 0.62))
        return self.min_voltage + ((self.max_voltage - self.min_voltage) * curve)

    def calc_current(self, elapsed_sec: float, percentage: float) -> float:
        soc = max(0.0, min(1.0, percentage / 100.0))
        dynamic = 0.42 + (0.20 * math.sin(elapsed_sec / 22.0)) + (0.18 * (1.0 - soc))
        ratio = max(0.0, min(1.0, dynamic))
        return self.min_current + ((self.max_current - self.min_current) * ratio)

    def calc_temperature(self, elapsed_sec: float, current: float) -> float:
        return self.base_temperature_c + (current * self.temp_rise_per_amp) + (0.7 * math.sin(elapsed_sec / 40.0))

    def calc_cell_voltages(self, total_voltage: float, percentage: float) -> list:
        base = total_voltage / float(self.cell_count)
        imbalance = 0.016 * (1.0 - (percentage / 100.0))
        offsets = [imbalance * (i - ((self.cell_count - 1) / 2.0)) for i in range(self.cell_count)]
        cells = [base + off for off in offsets]
        correction = total_voltage - sum(cells)
        cells[-1] += correction
        return [round(v, 3) for v in cells]

    def publish_dummy_payload(self):
        elapsed = time.time() - self.start_time
        percentage = self.calc_percentage(elapsed)
        voltage = self.calc_voltage(percentage)
        current = self.calc_current(elapsed, percentage)
        temperature = self.calc_temperature(elapsed, current)

        if percentage <= 10.0:
            status = 'critical'
        elif percentage <= 25.0:
            status = 'low'
        else:
            status = 'discharging'

        payload = {
            'battery_id': self.battery_id,
            'percentage': round(percentage, 1),
            'voltage': round(voltage, 2),
            'current': round(current, 2),
            'temperature': round(temperature, 1),
            'status': status,
            'cell_voltages': self.calc_cell_voltages(voltage, percentage),
            'cell_count': self.cell_count,
        }

        if not self.mqtt_connected:
            self.get_logger().warn('Skip publish: MQTT not connected yet')
            return

        payload_raw = json.dumps(payload)

        # Dashboard/general battery stream
        self.client.publish(self.battery_topic, payload_raw, qos=self.mqtt_qos, retain=False)

        # Feed seano_battery simulation listener to keep ROS failsafe pipeline active.
        self.client.publish(self.simulation_topic, payload_raw, qos=self.mqtt_qos, retain=False)

        if int(elapsed) % 5 == 0:
            self.get_logger().info(
                f'Dummy battery sent: {payload["percentage"]:.1f}% | {payload["voltage"]:.2f}V | '
                f'{payload["current"]:.2f}A'
            )

    def destroy_node(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SeanoBatteryDummyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()