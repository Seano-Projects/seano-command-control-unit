import csv
import json
import os
import random
from datetime import datetime
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_TZ = ZoneInfo('Asia/Jakarta')


class _DailyCsvWriter:
    def __init__(self, log_dir: str, prefix: str, fieldnames: list):
        self._log_dir = log_dir
        self._prefix = prefix
        self._fieldnames = fieldnames
        self._date = None
        self._fh = None
        self._writer = None
        os.makedirs(log_dir, exist_ok=True)

    def _rotate(self):
        today = datetime.now().strftime('%Y%m%d')
        if today == self._date:
            return
        if self._fh:
            self._fh.close()
        self._date = today
        path = os.path.join(self._log_dir, f'{self._prefix}_{today}.csv')
        first = not os.path.exists(path)
        self._fh = open(path, 'a', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._fh, fieldnames=self._fieldnames, extrasaction='ignore')
        if first:
            self._writer.writeheader()

    def write(self, row: dict):
        self._rotate()
        self._writer.writerow(row)
        self._fh.flush()

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None


_ADCP_FIELDS = [
    'log_timestamp', 'vehicle_code', 'sensor',
    'current_speed_ms', 'current_direction_deg', 'water_depth_m',
]


class ADCPSensorNode(Node):
    def __init__(self):
        super().__init__('adcp_sensor_node')

        self.declare_parameter('publish_topic', 'oceanography/adcp')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')

        topic = self.get_parameter('publish_topic').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)
        self.vehicle_code = self.get_parameter('vehicle.id').value

        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._adcp_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'oceanography'),
            'adcp_log',
            _ADCP_FIELDS,
        )

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.timer = self.create_timer(period, self.publish_measurement)

        self.get_logger().info(f'ADCP sensor node started on topic: {topic}')

    def publish_measurement(self):
        payload = {
            'sensor': 'ADCP',
            'current_speed_ms': round(random.uniform(0.0, 2.5), 2),
            'current_direction_deg': round(random.uniform(0.0, 359.9), 1),
            'water_depth_m': round(random.uniform(2.0, 250.0), 2),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)

        log_ts = datetime.now(_TZ).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]
        self._adcp_csv.write({
            'log_timestamp': log_ts,
            'vehicle_code': self.vehicle_code,
            **payload,
        })

    def destroy_node(self):
        self._adcp_csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ADCPSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
