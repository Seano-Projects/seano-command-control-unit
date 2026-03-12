import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, CommandLong, SetMode
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

        # Get parameters
        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.qos = int(self.get_parameter('mqtt.qos').value)

        # MQTT topics
        self.command_topic = f"{self.base_topic}/{self.vehicle_id}/command"
        self.waypoint_topic = f"{self.base_topic}/{self.vehicle_id}/waypoint"
        self.status_topic = f"{self.base_topic}/{self.vehicle_id}/status"

        # Setup MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        if self.mqtt_username:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.client.tls_insecure_set(True)

        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.client.loop_start()
            self.get_logger().info(f"Connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")
            raise SystemExit

        # ROS2 service clients for MAVROS
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ROS2 publisher for command status
        self.status_publisher = self.create_publisher(String, 'command_status', 10)

        self.get_logger().info(f"Command node started for vehicle: {self.vehicle_id}")
        self.get_logger().info(f"Listening to: {self.command_topic}, {self.waypoint_topic}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected successfully")
            # Subscribe to command and waypoint topics
            client.subscribe(self.command_topic, qos=self.qos)
            client.subscribe(self.waypoint_topic, qos=self.qos)
            self.get_logger().info(f"Subscribed to: {self.command_topic}")
            self.get_logger().info(f"Subscribed to: {self.waypoint_topic}")
        else:
            self.get_logger().error(f"MQTT connection failed with code: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            
            if msg.topic == self.command_topic:
                self.handle_command(payload)
            elif msg.topic == self.waypoint_topic:
                self.handle_waypoint(payload)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def handle_command(self, payload):
        """Handle command messages from MQTT"""
        command_type = payload.get('command', '')
        self.get_logger().info(f"Received command: {command_type}")

        if command_type == 'ARM':
            self.send_arm_command(True, force=False)
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

    def handle_waypoint(self, payload):
        """Handle waypoint upload from MQTT"""
        waypoints_data = payload.get('waypoints', [])
        self.get_logger().info(f"Received {len(waypoints_data)} waypoints")

        if not waypoints_data:
            self.publish_status("No waypoints provided", False)
            return

        # Create waypoint list
        waypoint_list = []
        for idx, wp_data in enumerate(waypoints_data):
            wp = Waypoint()
            wp.frame = wp_data.get('frame', 3)  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command = wp_data.get('command', 16)  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = (idx == 0)
            wp.autocontinue = True
            wp.param1 = float(wp_data.get('param1', 0.0))
            wp.param2 = float(wp_data.get('param2', 0.0))
            wp.param3 = float(wp_data.get('param3', 0.0))
            wp.param4 = float(wp_data.get('param4', 0.0))
            wp.x_lat = float(wp_data.get('latitude', 0.0))
            wp.y_long = float(wp_data.get('longitude', 0.0))
            wp.z_alt = float(wp_data.get('altitude', 0.0))
            waypoint_list.append(wp)

        # Call waypoint push service
        self.push_waypoints(waypoint_list)

    def send_arm_command(self, arm, force=False):
        """Send ARM/DISARM command via MAVROS"""
        if not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("MAVROS command service not available")
            self.publish_status("MAVROS service unavailable", False)
            return

        req = CommandLong.Request()
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.param1 = 1.0 if arm else 0.0
        req.param2 = 21196.0 if (not arm and force) else 0.0  # Force disarm magic number
        
        cmd_name = 'ARM' if arm else ('FORCE_DISARM' if force else 'DISARM')
        self.get_logger().info(f"Sending {cmd_name} command (force={force})")
        
        future = self.command_client.call_async(req)
        future.add_done_callback(
            lambda f: self.command_response_callback(f, cmd_name)
        )

    def send_mode_command(self, mode):
        """Send mode change command via SetMode service"""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("MAVROS set_mode service not available")
            self.publish_status("Set mode service unavailable", False)
            return

        req = SetMode.Request()
        req.custom_mode = mode
        
        self.get_logger().info(f"Sending mode change to {mode}")
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.mode_response_callback(f, mode)
        )

    def push_waypoints(self, waypoints):
        """Push waypoints to MAVROS"""
        if not self.waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("MAVROS waypoint service not available")
            self.publish_status("Waypoint service unavailable", False)
            return

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints

        future = self.waypoint_push_client.call_async(req)
        future.add_done_callback(
            lambda f: self.waypoint_response_callback(f, len(waypoints))
        )

    def command_response_callback(self, future, command_name):
        """Handle command service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{command_name} command successful")
                self.publish_status(f"{command_name} successful", True)
            else:
                self.get_logger().error(f"{command_name} command failed: result={response.result}")
                self.publish_status(f"{command_name} failed: code {response.result}", False)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.publish_status(f"{command_name} error: {e}", False)

    def mode_response_callback(self, future, mode_name):
        """Handle set mode service response"""
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

    def waypoint_response_callback(self, future, wp_count):
        """Handle waypoint push response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully uploaded {wp_count} waypoints")
                self.publish_status(f"Uploaded {wp_count} waypoints", True)
            else:
                self.get_logger().error("Waypoint upload failed")
                self.publish_status("Waypoint upload failed", False)
        except Exception as e:
            self.get_logger().error(f"Waypoint service call failed: {e}")
            self.publish_status(f"Waypoint error: {e}", False)

    def publish_status(self, message, success):
        """Publish command execution status"""
        status_msg = String()
        status_msg.data = json.dumps({
            "status": "success" if success else "error",
            "message": message,
            "vehicle_id": self.vehicle_id
        })
        self.status_publisher.publish(status_msg)
        
        # Also publish to MQTT
        self.client.publish(
            self.status_topic,
            status_msg.data,
            qos=self.qos
        )

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
