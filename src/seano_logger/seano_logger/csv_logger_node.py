import csv
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Topics to log: (ros_topic, subfolder, csv_filename_prefix)
TOPICS_TO_LOG = [
    # Telemetry utama
    ('telemetry',              'telemetry',    'telemetry'),
    # Mission
    ('mission/status',         'mission',      'mission_status'),
    ('mission/waypoint_reached', 'mission',    'waypoint_reached'),
    # Failsafe
    ('failsafe/alert',         'failsafe',     'failsafe_alert'),
    # Anti-theft
    ('anti_theft/alert',       'anti_theft',   'anti_theft_alert'),
    # Command dari web
    ('command_status',         'command',      'command_status'),
    ('waypoint_status',        'command',      'waypoint_status'),
    # Oceanography / Sensor
    ('oceanography/ctd',       'oceanography', 'oceanography_ctd'),
    ('oceanography/adcp',      'oceanography', 'oceanography_adcp'),
    ('oceanography/sbes',      'oceanography', 'oceanography_sbes'),
]


class CsvLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger_node')

        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        log_dir_raw = self.get_parameter('logger.log_dir').value
        self.log_dir = os.path.expanduser(log_dir_raw)
        os.makedirs(self.log_dir, exist_ok=True)

        # Session timestamp — satu sesi = satu set file CSV baru
        session_ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        self._entries = {}  # topic → {'fh', 'writer', 'filepath'}

        for topic, subfolder, prefix in TOPICS_TO_LOG:
            sub_dir = os.path.join(self.log_dir, subfolder)
            os.makedirs(sub_dir, exist_ok=True)
            filepath = os.path.join(sub_dir, f'{prefix}_{session_ts}.csv')
            fh = open(filepath, 'w', newline='', encoding='utf-8')
            self._entries[topic] = {
                'fh': fh,
                'writer': None,
                'filepath': filepath,
            }
            self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self._callback(msg, t),
                10,
            )
            self.get_logger().info(f'Logging [{topic}] → {filepath}')

        self.get_logger().info(f'CSV Logger Node started | session: {session_ts}')

    # ------------------------------------------------------------------
    def _flatten(self, obj, prefix=''):
        """Rekursif flatten nested dict/list ke key dot-notation."""
        result = {}
        if isinstance(obj, dict):
            for k, v in obj.items():
                full_key = f'{prefix}.{k}' if prefix else str(k)
                result.update(self._flatten(v, full_key))
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                full_key = f'{prefix}[{i}]'
                result.update(self._flatten(v, full_key))
        else:
            result[prefix] = obj
        return result

    def _callback(self, msg: String, topic: str):
        now = datetime.now()
        entry = self._entries[topic]

        try:
            data = json.loads(msg.data)
            flat = self._flatten(data)
        except Exception:
            flat = {'raw': msg.data}

        # Kolom waktu selalu di depan
        row = {
            'datetime': now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'timestamp': now.timestamp(),
        }
        row.update(flat)

        # Inisiasi writer saat baris pertama masuk (header dinamis dari JSON)
        if entry['writer'] is None:
            fieldnames = list(row.keys())
            entry['writer'] = csv.DictWriter(
                entry['fh'],
                fieldnames=fieldnames,
                extrasaction='ignore',
            )
            entry['writer'].writeheader()

        entry['writer'].writerow(row)
        entry['fh'].flush()

    # ------------------------------------------------------------------
    def destroy_node(self):
        for entry in self._entries.values():
            try:
                entry['fh'].close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsvLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
