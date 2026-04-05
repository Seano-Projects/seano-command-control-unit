import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import ssl
import json


class MqttBridgeNode(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('mqtt.broker', 'localhost')
        self.declare_parameter('mqtt.port', 1883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)

        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.qos = int(self.get_parameter('mqtt.qos').value)

        self.mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/telemetry"
        self.failsafe_mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/failsafe"

        self.client = mqtt.Client()

        if self.mqtt_username:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.client.tls_insecure_set(True)

        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.client.loop_start()
            self.get_logger().info(f"Connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}")
            self.get_logger().info(f"Publish topic: {self.mqtt_topic}")
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")
            raise SystemExit

        self.create_subscription(
            String,
            'telemetry',
            self.telemetry_callback,
            10
        )
        self.create_subscription(
            String,
            'failsafe/alert',
            self.failsafe_callback,
            10
        )

    def telemetry_callback(self, msg):
        self.client.publish(
            self.mqtt_topic,
            msg.data,
            qos=self.qos
        )

    def failsafe_callback(self, msg):
        self.client.publish(
            self.failsafe_mqtt_topic,
            msg.data,
            qos=1,
            retain=False
        )


def main():
    rclpy.init()
    node = MqttBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
