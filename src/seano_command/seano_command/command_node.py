import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandLong, SetMode
import paho.mqtt.client as mqtt
import ssl
import json


class CommandNode(Node):

    def __init__(self):
        super().__init__('command_node')

        # Declare parameters
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

        # Get parameters
        self.vehicle_id   = self.get_parameter('vehicle.id').value
        self.mqtt_broker  = self.get_parameter('mqtt.broker').value
        self.mqtt_port    = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic   = self.get_parameter('mqtt.base_topic').value
        self.qos          = int(self.get_parameter('mqtt.qos').value)
        self.keepalive    = int(self.get_parameter('mqtt.keepalive').value)
        self.use_tls      = bool(self.get_parameter('mqtt.use_tls').value)
        self.tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)

        # MQTT topics
        self.command_topic = f"{self.base_topic}/{self.vehicle_id}/command"
        self.status_topic  = f"{self.base_topic}/{self.vehicle_id}/command/response"

        # Setup MQTT client
        self.client = mqtt.Client()
        self.client.on_connect    = self.on_connect
        self.client.on_message    = self.on_message
        self.client.on_disconnect = self.on_disconnect

        if self.mqtt_username:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        if self.use_tls:
            self.client.tls_set(cert_reqs=ssl.CERT_NONE)
            self.client.tls_insecure_set(self.tls_insecure)

        self.client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, keepalive=self.keepalive)
            self.client.loop_start()
            self.get_logger().info(f"Connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")
            raise SystemExit

        # ROS2 service clients for MAVROS
        self.command_client  = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ROS2 publisher for command status
        self.status_publisher = self.create_publisher(String, 'command_status', 10)

        self.get_logger().info(f"Command node aktif — vehicle: {self.vehicle_id}")
        self.get_logger().info(f"MQTT topic: {self.command_topic}")

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(self.command_topic, qos=self.qos)
            self.get_logger().info(f"MQTT subscribe: {self.command_topic}")
        else:
            self.get_logger().error(f"MQTT connect gagal, rc={rc}")

    def on_disconnect(self, client, userdata, rc, properties=None):
        if rc != 0:
            self.get_logger().warn("MQTT disconnected unexpectedly, reconnecting...")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            self.handle_command(payload)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def handle_command(self, payload):
        if not isinstance(payload, dict):
            self.publish_status("Invalid command payload", False)
            return

        command_type = str(payload.get('command', '')).strip().upper()
        self.get_logger().info(f"Received command: {command_type}")

        if command_type == 'ARM':
            self.send_arm_command(True, force=False)
        elif command_type == 'FORCE_ARM':
            self.send_arm_command(True, force=True)
        elif command_type == 'DISARM':
            self.send_arm_command(False, force=False)
        elif command_type == 'FORCE_DISARM':
            self.send_arm_command(False, force=True)
        elif command_type == 'AUTO':
            self.send_mode_command('AUTO')
        elif command_type == 'MANUAL':
            self.send_mode_command('MANUAL')
        elif command_type == 'HOLD':
            self.send_mode_command('HOLD')
        elif command_type == 'LOITER':
            self.send_mode_command('LOITER')
        elif command_type == 'RTL':
            self.send_mode_command('RTL')
        else:
            self.get_logger().warn(f"Unknown command: {command_type}")
            self.publish_status(f"Unknown command: {command_type}", False)

    def send_arm_command(self, arm, force=False):
        if not self.wait_for_mavros_service(self.command_client, '/mavros/cmd/command'):
            self.publish_status("MAVROS service unavailable", False)
            return

        req = CommandLong.Request()
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.param1  = 1.0 if arm else 0.0
        if force and arm:
            req.param2 = 2989.0
        elif force and (not arm):
            req.param2 = 21196.0
        else:
            req.param2 = 0.0

        cmd_name = ('FORCE_ARM' if force else 'ARM') if arm else ('FORCE_DISARM' if force else 'DISARM')
        self.get_logger().info(f"Sending {cmd_name} command")

        future = self.command_client.call_async(req)
        future.add_done_callback(lambda f: self.command_response_callback(f, cmd_name))

    def send_mode_command(self, mode):
        if not self.wait_for_mavros_service(self.set_mode_client, '/mavros/set_mode'):
            self.publish_status("Set mode service unavailable", False)
            return

        req = SetMode.Request()
        req.custom_mode = mode

        self.get_logger().info(f"Sending mode change to {mode}")
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.mode_response_callback(f, mode))

    def command_response_callback(self, future, command_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{command_name} successful")
                self.publish_status(f"{command_name} successful", True)
            else:
                self.get_logger().error(f"{command_name} failed: result={response.result}")
                self.publish_status(f"{command_name} failed: code {response.result}", False)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.publish_status(f"{command_name} error: {e}", False)

    def mode_response_callback(self, future, mode_name):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Mode change to {mode_name} successful")
                self.publish_status(f"Mode changed to {mode_name}", True)
            else:
                self.get_logger().error(f"Mode change to {mode_name} failed")
                self.publish_status(f"Mode change to {mode_name} failed", False)
        except Exception as e:
            self.get_logger().error(f"Set mode service call failed: {e}")
            self.publish_status(f"Mode change error: {e}", False)

    def publish_status(self, message, success):
        data = json.dumps({
            "status": "success" if success else "error",
            "message": message,
            "vehicle_id": self.vehicle_id,
        })
        msg = String()
        msg.data = data
        self.status_publisher.publish(msg)
        self.client.publish(self.status_topic, data, qos=self.qos)

    def wait_for_mavros_service(self, client, service_name: str, retries: int = 5, timeout_sec: float = 1.0) -> bool:
        for attempt in range(1, retries + 1):
            if client.wait_for_service(timeout_sec=timeout_sec):
                return True
            self.get_logger().warn(f"Waiting for {service_name} ({attempt}/{retries})")
        self.get_logger().error(f"{service_name} not available after {retries} retries")
        return False

    def destroy_node(self):
        self.client.loop_stop()
        self.client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
