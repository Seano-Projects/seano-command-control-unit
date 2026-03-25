import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SBESSensorNode(Node):
    def __init__(self):
        super().__init__('sbes_sensor_node')

        self.declare_parameter('publish_topic', 'oceanography/sbes')
        self.declare_parameter('publish_rate_hz', 1.0)

        topic = self.get_parameter('publish_topic').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.timer = self.create_timer(period, self.publish_measurement)

        self.get_logger().info(f'SBES sensor node started on topic: {topic}')

    def publish_measurement(self):
        payload = {
            'sensor': 'SBES',
            'depth_m': round(random.uniform(0.3, 180.0), 2),
            'seafloor_confidence_percent': round(random.uniform(75.0, 99.9), 1),
            'sound_velocity_ms': round(random.uniform(1450.0, 1540.0), 1),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = SBESSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
