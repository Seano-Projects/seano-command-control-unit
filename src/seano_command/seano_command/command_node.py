import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, CommandLong, SetMode
import paho.mqtt.client as mqtt
import ssl
import json
from typing import Any, Dict, List, Optional


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
        self.declare_parameter('mission.auto_set_home_from_first_waypoint', True)

        # Get parameters
        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.qos = int(self.get_parameter('mqtt.qos').value)
        self.keepalive = int(self.get_parameter('mqtt.keepalive').value)
        self.use_tls = bool(self.get_parameter('mqtt.use_tls').value)
        self.tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)
        self.auto_set_home_from_first_wp = bool(
            self.get_parameter('mission.auto_set_home_from_first_waypoint').value
        )

        # MQTT topics
        self.command_topic = f"{self.base_topic}/{self.vehicle_id}/command"
        self.waypoint_topic = f"{self.base_topic}/{self.vehicle_id}/waypoint"
        self.status_topic = f"{self.base_topic}/{self.vehicle_id}/status"

        # Setup MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
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
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ROS2 publisher for command status
        self.status_publisher = self.create_publisher(String, 'command_status', 10)

        self.get_logger().info(f"Command node started for vehicle: {self.vehicle_id}")
        self.get_logger().info(f"Listening to: {self.command_topic}, {self.waypoint_topic}")
        self.get_logger().info(
            f"Auto set home from first waypoint: {self.auto_set_home_from_first_wp}"
        )

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info("MQTT connected successfully")
            # Subscribe to command and waypoint topics
            client.subscribe(self.command_topic, qos=self.qos)
            client.subscribe(self.waypoint_topic, qos=self.qos)
            self.get_logger().info(f"Subscribed to: {self.command_topic}")
            self.get_logger().info(f"Subscribed to: {self.waypoint_topic}")
        else:
            self.get_logger().error(f"MQTT connection failed with code: {rc}")

    def on_disconnect(self, client, userdata, rc, properties=None):
        if rc != 0:
            self.get_logger().warn("MQTT disconnected unexpectedly, reconnecting...")

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

    def handle_waypoint(self, payload):
        """Handle waypoint upload from MQTT"""
        waypoints_data = self.extract_waypoints(payload)
        self.get_logger().info(f"Received {len(waypoints_data)} waypoints")

        if not waypoints_data:
            self.publish_status("No waypoints provided", False)
            return

        waypoint_list: List[Waypoint] = []
        for idx, wp_data in enumerate(waypoints_data):
            wp = self.build_waypoint(wp_data, idx == 0)
            if wp is not None:
                waypoint_list.append(wp)

        if not waypoint_list:
            self.publish_status("No valid waypoints to upload", False)
            return

        if self.should_set_home_from_first_waypoint(payload):
            first_wp = waypoint_list[0]
            self.set_home_then_push(first_wp, waypoint_list)
            return

        # Call waypoint push service
        self.push_waypoints(waypoint_list)

    def should_set_home_from_first_waypoint(self, payload: Any) -> bool:
        """Determine if home should be set from first waypoint before mission upload."""
        if not isinstance(payload, dict):
            return self.auto_set_home_from_first_wp

        override = payload.get('set_home_from_first_waypoint')
        if override is None:
            return self.auto_set_home_from_first_wp

        if isinstance(override, bool):
            return override

        if isinstance(override, str):
            return override.strip().lower() in ('1', 'true', 'yes', 'on')

        return bool(override)

    def set_home_then_push(self, first_wp: Waypoint, waypoint_list: List[Waypoint]):
        """Set FCU home to first waypoint, then upload mission only if set-home succeeds."""
        if not self.wait_for_mavros_service(self.command_client, '/mavros/cmd/command'):
            self.publish_status("Set home failed: MAVROS command service unavailable", False)
            return

        req = CommandLong.Request()
        req.command = 179  # MAV_CMD_DO_SET_HOME
        req.param1 = 0.0   # Use specified coordinates (not current location)
        req.param5 = float(first_wp.x_lat)
        req.param6 = float(first_wp.y_long)
        req.param7 = float(first_wp.z_alt)

        self.get_logger().info(
            f"Setting home from first waypoint: lat={req.param5}, lon={req.param6}, alt={req.param7}"
        )

        future = self.command_client.call_async(req)
        future.add_done_callback(
            lambda f: self.set_home_response_callback(f, waypoint_list)
        )

    def set_home_response_callback(self, future, waypoint_list: List[Waypoint]):
        """Handle set-home response, then continue mission upload if successful."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Set home successful, continuing waypoint upload")
                self.push_waypoints(waypoint_list)
            else:
                self.get_logger().error(f"Set home failed: result={response.result}")
                self.publish_status(f"Set home failed: code {response.result}", False)
        except Exception as e:
            self.get_logger().error(f"Set home service call failed: {e}")
            self.publish_status(f"Set home error: {e}", False)

    def extract_waypoints(self, payload: Any) -> List[Dict[str, Any]]:
        """Accept both {'waypoints': [...]} and direct list payloads."""
        if isinstance(payload, dict):
            waypoint_obj = payload.get('waypoints')
            if isinstance(waypoint_obj, list):
                return [wp for wp in waypoint_obj if isinstance(wp, dict)]

            if 'latitude' in payload or 'lat' in payload:
                return [payload]
            return []

        if isinstance(payload, list):
            return [wp for wp in payload if isinstance(wp, dict)]

        return []

    def build_waypoint(self, wp_data: Dict[str, Any], is_current: bool) -> Optional[Waypoint]:
        """Convert incoming dict to MAVROS waypoint with basic validation."""
        latitude = wp_data.get('latitude', wp_data.get('lat'))
        longitude = wp_data.get('longitude', wp_data.get('lon', wp_data.get('lng')))
        altitude = wp_data.get('altitude', wp_data.get('alt', 0.0))

        if latitude is None or longitude is None:
            self.get_logger().warn("Waypoint skipped: latitude/longitude missing")
            return None

        try:
            lat = float(latitude)
            lon = float(longitude)
            alt = float(altitude)
        except (TypeError, ValueError):
            self.get_logger().warn("Waypoint skipped: invalid numeric latitude/longitude/altitude")
            return None

        if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
            self.get_logger().warn("Waypoint skipped: latitude/longitude out of range")
            return None

        wp = Waypoint()
        wp.frame = int(wp_data.get('frame', 3))
        wp.command = int(wp_data.get('command', 16))
        wp.is_current = is_current
        wp.autocontinue = bool(wp_data.get('autocontinue', True))
        wp.param1 = float(wp_data.get('param1', 0.0))
        wp.param2 = float(wp_data.get('param2', 0.0))
        wp.param3 = float(wp_data.get('param3', 0.0))
        wp.param4 = float(wp_data.get('param4', 0.0))
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        return wp

    def send_arm_command(self, arm, force=False):
        """Send ARM/DISARM command via MAVROS"""
        if not self.wait_for_mavros_service(self.command_client, '/mavros/cmd/command'):
            self.publish_status("MAVROS service unavailable", False)
            return

        req = CommandLong.Request()
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.param1 = 1.0 if arm else 0.0
        if force and arm:
            req.param2 = 2989.0  # Force arm magic number
        elif force and (not arm):
            req.param2 = 21196.0  # Force disarm magic number
        else:
            req.param2 = 0.0

        if arm:
            cmd_name = 'FORCE_ARM' if force else 'ARM'
        else:
            cmd_name = 'FORCE_DISARM' if force else 'DISARM'
        self.get_logger().info(f"Sending {cmd_name} command (force={force})")
        
        future = self.command_client.call_async(req)
        future.add_done_callback(
            lambda f: self.command_response_callback(f, cmd_name)
        )

    def send_mode_command(self, mode):
        """Send mode change command via SetMode service"""
        if not self.wait_for_mavros_service(self.set_mode_client, '/mavros/set_mode'):
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
        if not self.wait_for_mavros_service(self.waypoint_push_client, '/mavros/mission/push'):
            self.publish_status("Waypoint service unavailable", False)
            return

        self.get_logger().info(f"Uploading {len(waypoints)} waypoints to MAVROS")

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

    def wait_for_mavros_service(self, client, service_name: str, retries: int = 5, timeout_sec: float = 1.0) -> bool:
        """Wait with retries to avoid transient MAVROS startup timing issues."""
        for attempt in range(1, retries + 1):
            if client.wait_for_service(timeout_sec=timeout_sec):
                return True
            self.get_logger().warn(
                f"Waiting for {service_name} ({attempt}/{retries})"
            )

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
