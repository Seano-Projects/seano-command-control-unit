import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ADCPSensorNode(Node):
    def __init__(self):
        super().__init__('adcp_sensor_node')

        self.declare_parameter('publish_topic', 'oceanography/adcp')
        self.declare_parameter('publish_rate_hz', 1.0)

        topic = self.get_parameter('publish_topic').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)

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


def main():
    rclpy.init()
    node = ADCPSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
