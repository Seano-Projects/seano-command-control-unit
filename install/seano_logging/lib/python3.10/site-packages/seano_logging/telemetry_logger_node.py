import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
import json
from datetime import datetime, timezone


class TelemetryLoggerNode(Node):

    def __init__(self):
        super().__init__('telemetry_logger')

        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('logging.enable', True)
        self.declare_parameter('logging.path', '/tmp')
        self.declare_parameter('logging.format', 'csv')

        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.enable = self.get_parameter('logging.enable').value
        self.log_path = self.get_parameter('logging.path').value
        self.log_format = self.get_parameter('logging.format').value

        if not self.enable:
            self.get_logger().info('Logging disabled')
            return

        os.makedirs(self.log_path, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f"{self.vehicle_id}_telemetry_{timestamp}.csv"
        self.filepath = os.path.join(self.log_path, self.filename)

        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp', 'vehicle_id', 'armed', 'mode', 'system_mode',
            'latitude', 'longitude', 'altitude',
            'roll', 'pitch', 'yaw', 'heading'
        ])

        self.create_subscription(
            String,
            'telemetry',
            self.telemetry_callback,
            10
        )

        self.get_logger().info(f"Logging to {self.filepath}")

    def telemetry_callback(self, msg):
        try:
            data = json.loads(msg.data)
            now = datetime.now(timezone.utc).isoformat()
            
            self.writer.writerow([
                now,
                data.get('vehicle_id', self.vehicle_id),
                data.get('armed', False),
                data.get('mode', 'UNKNOWN'),
                data.get('system_mode', 'unknown'),
                data.get('position', {}).get('latitude', 0.0),
                data.get('position', {}).get('longitude', 0.0),
                data.get('position', {}).get('altitude', 0.0),
                data.get('attitude', {}).get('roll', 0.0),
                data.get('attitude', {}).get('pitch', 0.0),
                data.get('attitude', {}).get('yaw', 0.0),
                data.get('attitude', {}).get('heading', 0.0)
            ])
            self.file.flush()
        except json.JSONDecodeError:
            now = datetime.now(timezone.utc).isoformat()
            self.writer.writerow([now, self.vehicle_id, msg.data, '', '', '', '', '', '', '', '', ''])
            self.file.flush()


def main():
    rclpy.init()
    node = TelemetryLoggerNode()
    rclpy.spin(node)
    node.file.close()
    node.destroy_node()
    rclpy.shutdown()
