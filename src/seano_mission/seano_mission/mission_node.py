import json
import math
import os
import ssl
from typing import Any, Dict, List, Optional

import paho.mqtt.client as mqtt
import rclpy
from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import HomePosition, State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import CommandLong, WaypointPush
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('vehicle.id', 'USV-001')
        self.vehicle_id = self.get_parameter('vehicle.id').value

        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', True)
        self.declare_parameter('mission.auto_set_home_from_first_waypoint', True)

        # Mission state
        self.current_waypoint_seq = 0
        self.total_waypoints = 0
        self.waypoints = []
        self.mission_active = False
        self.mavros_connected = False
        self.armed = False
        self.mode = 'UNKNOWN'

        # Home position
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0

        # Last reached waypoint
        self.last_reached_seq = -1

        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.mqtt_base_topic = self.get_parameter('mqtt.base_topic').value
        self.mqtt_qos = int(self.get_parameter('mqtt.qos').value)
        self.mqtt_keepalive = int(self.get_parameter('mqtt.keepalive').value)
        self.mqtt_use_tls = bool(self.get_parameter('mqtt.use_tls').value)
        self.mqtt_tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)
        self.auto_set_home_from_first_waypoint = bool(
            self.get_parameter('mission.auto_set_home_from_first_waypoint').value
        )

        self.waypoint_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/waypoint'
        self.waypoint_status_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/waypoint/status'

        self._mqtt_connected = False

        self._cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self._wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe MAVROS state
        self.create_subscription(State, '/mavros/state', self.state_callback, sensor_qos)

        # Subscribe waypoint reached event
        self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            sensor_qos
        )

        # Subscribe waypoint list (current mission loaded)
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoint_list_callback,
            sensor_qos
        )

        # Subscribe home position
        self.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self.home_callback,
            sensor_qos
        )

        # Publisher — mission status JSON
        self.pub_status = self.create_publisher(String, 'mission/status', 10)

        # Publisher — waypoint reached event JSON
        self.pub_reached = self.create_publisher(String, 'mission/waypoint_reached', 10)

        # Publisher — waypoint upload ACK JSON
        self.pub_waypoint_status = self.create_publisher(String, 'waypoint_status', 10)

        self._mqtt_client = self._create_mqtt_client()
        self._start_mqtt()

        # Timer publish status tiap 2 detik
        self.create_timer(2.0, self.publish_status)
        self.create_timer(10.0, self._reconnect_mqtt_if_needed)

        self.get_logger().info('Mission Node started')
        self.get_logger().info(f'Vehicle ID: {self.vehicle_id}')
        self.get_logger().info(f'Waypoint MQTT topic: {self.waypoint_topic}')
        self.get_logger().info(f'Waypoint ACK topic: {self.waypoint_status_topic}')

    def state_callback(self, msg: State):
        self.mavros_connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode
        self.mission_active = msg.armed and msg.mode in ('AUTO', 'GUIDED')

    def waypoint_reached_callback(self, msg: WaypointReached):
        self.last_reached_seq = msg.wp_seq
        self.get_logger().info(f'Waypoint reached: #{msg.wp_seq}')

        # Publish event
        data = {
            'vehicle_id': self.vehicle_id,
            'event': 'waypoint_reached',
            'wp_seq': msg.wp_seq,
            'total': self.total_waypoints,
            'remaining': max(0, self.total_waypoints - msg.wp_seq - 1),
        }
        msg_out = String()
        msg_out.data = json.dumps(data)
        self.pub_reached.publish(msg_out)

    def waypoint_list_callback(self, msg: WaypointList):
        self.current_waypoint_seq = msg.current_seq
        self.total_waypoints = len(msg.waypoints)
        self.waypoints = []

        for i, wp in enumerate(msg.waypoints):
            self.waypoints.append({
                'seq': i,
                'frame': wp.frame,
                'command': wp.command,
                'is_current': wp.is_current,
                'autocontinue': wp.autocontinue,
                'lat': round(wp.x_lat, 6),
                'lon': round(wp.y_long, 6),
                'alt': round(wp.z_alt, 2),
                'param1': wp.param1,  # e.g. acceptance radius for NAV_WAYPOINT
            })

    def home_callback(self, msg: HomePosition):
        self.home_lat = round(msg.geo.latitude, 6)
        self.home_lon = round(msg.geo.longitude, 6)
        self.home_alt = round(msg.geo.altitude, 2)

    def _create_mqtt_client(self):
        client_id = f'mission_{self.vehicle_id}_{os.getpid()}'
        try:
            from paho.mqtt.client import CallbackAPIVersion

            client = mqtt.Client(
                callback_api_version=CallbackAPIVersion.VERSION1,
                client_id=client_id,
            )
        except ImportError:
            client = mqtt.Client(client_id=client_id)

        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)

        if self.mqtt_use_tls:
            client.tls_set(cert_reqs=ssl.CERT_NONE)
            client.tls_insecure_set(self.mqtt_tls_insecure)

        client.reconnect_delay_set(min_delay=1, max_delay=30)
        client.on_connect = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        client.on_message = self._on_mqtt_message
        return client

    def _start_mqtt(self):
        try:
            self._mqtt_client.loop_start()
            self._mqtt_client.connect_async(
                self.mqtt_broker,
                self.mqtt_port,
                keepalive=self.mqtt_keepalive,
            )
            self.get_logger().info(
                f'MQTT connect scheduled: {self.mqtt_broker}:{self.mqtt_port}'
            )
        except Exception as exc:
            self.get_logger().error(f'MQTT startup failed: {exc}')

    def _reconnect_mqtt_if_needed(self):
        if self._mqtt_connected:
            return
        try:
            self._mqtt_client.reconnect()
        except Exception:
            pass

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = (rc == 0)
        if self._mqtt_connected:
            client.subscribe(self.waypoint_topic, qos=self.mqtt_qos)
            self.get_logger().info(f'MQTT subscribed: {self.waypoint_topic}')
            self.get_logger().info(f'Waypoint ACK topic: {self.waypoint_status_topic}')
        else:
            self.get_logger().warn(f'MQTT connect failed rc={rc}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected unexpectedly rc={rc}')

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8', errors='ignore'))
        except Exception as exc:
            self._publish_waypoint_ack('FAILED', f'Payload JSON tidak valid: {exc}')
            return

        self._handle_waypoint_upload(payload)

    def _handle_waypoint_upload(self, payload: Any):
        waypoints_data = self._extract_waypoints(payload)
        if not waypoints_data:
            self._publish_waypoint_ack('FAILED', 'Tidak ada waypoint yang diberikan')
            return

        should_set_home = self._should_set_home(payload)
        waypoint_list: List[Waypoint] = []

        for idx, wp_data in enumerate(waypoints_data):
            wp = self._build_waypoint(wp_data, idx == 0)
            if wp is not None:
                waypoint_list.append(wp)

        if not waypoint_list:
            self._publish_waypoint_ack('FAILED', 'Tidak ada waypoint valid untuk diupload')
            return

        if should_set_home:
            self._set_home_then_push(waypoint_list)
        else:
            self._push_waypoints(waypoint_list)

    def _should_set_home(self, payload: Any) -> bool:
        if not isinstance(payload, dict):
            return self.auto_set_home_from_first_waypoint

        override = payload.get('set_home_from_first_waypoint')
        if override is None:
            return self.auto_set_home_from_first_waypoint
        if isinstance(override, bool):
            return override
        if isinstance(override, str):
            return override.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(override)

    def _extract_waypoints(self, payload: Any) -> List[Dict[str, Any]]:
        if isinstance(payload, dict):
            waypoints = payload.get('waypoints')
            if isinstance(waypoints, list):
                return [wp for wp in waypoints if isinstance(wp, dict)]
            if 'lat' in payload or 'latitude' in payload:
                return [payload]
            return []
        if isinstance(payload, list):
            return [wp for wp in payload if isinstance(wp, dict)]
        return []

    def _build_waypoint(self, wp_data: Dict[str, Any], is_current: bool) -> Optional[Waypoint]:
        latitude = wp_data.get('lat', wp_data.get('latitude'))
        longitude = wp_data.get('lon', wp_data.get('longitude', wp_data.get('lng')))
        altitude = wp_data.get('alt', wp_data.get('altitude', 0.0))

        if latitude is None or longitude is None:
            self.get_logger().warn('Waypoint dilewati: lat/lon kosong')
            return None

        try:
            lat = float(latitude)
            lon = float(longitude)
            alt = float(altitude)
        except (TypeError, ValueError):
            self.get_logger().warn('Waypoint dilewati: lat/lon/alt harus numerik')
            return None

        if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
            self.get_logger().warn('Waypoint dilewati: koordinat di luar rentang valid')
            return None

        waypoint = Waypoint()
        waypoint.frame = int(wp_data.get('frame', 3))
        waypoint.command = int(wp_data.get('command', 16))
        waypoint.is_current = is_current
        waypoint.autocontinue = bool(wp_data.get('autocontinue', True))
        waypoint.param1 = float(wp_data.get('param1', 0.0))
        waypoint.param2 = float(wp_data.get('param2', 0.0))
        waypoint.param3 = float(wp_data.get('param3', 0.0))
        waypoint.param4 = float(wp_data.get('param4', 0.0))
        waypoint.x_lat = lat
        waypoint.y_long = lon
        waypoint.z_alt = alt
        return waypoint

    def _set_home_then_push(self, waypoint_list: List[Waypoint]):
        if not self._wait_for_service(self._cmd_client, '/mavros/cmd/command'):
            self._publish_waypoint_ack('FAILED', 'Service set home tidak tersedia')
            return

        first_wp = waypoint_list[0]
        request = CommandLong.Request()
        request.command = 179  # MAV_CMD_DO_SET_HOME
        request.param1 = 0.0
        request.param5 = float(first_wp.x_lat)
        request.param6 = float(first_wp.y_long)
        request.param7 = float(first_wp.z_alt)

        future = self._cmd_client.call_async(request)
        future.add_done_callback(lambda f: self._set_home_callback(f, waypoint_list))

    def _set_home_callback(self, future, waypoint_list: List[Waypoint]):
        try:
            response = future.result()
            if getattr(response, 'success', False):
                self._push_waypoints(waypoint_list)
            else:
                self._publish_waypoint_ack('FAILED', 'Set home ditolak oleh flight controller')
        except Exception as exc:
            self._publish_waypoint_ack('FAILED', f'Set home gagal: {exc}')

    def _push_waypoints(self, waypoint_list: List[Waypoint]):
        if not self._wait_for_service(self._wp_push_client, '/mavros/mission/push'):
            self._publish_waypoint_ack('FAILED', 'Service upload waypoint tidak tersedia')
            return

        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoint_list

        future = self._wp_push_client.call_async(request)
        future.add_done_callback(lambda f: self._push_callback(f, len(waypoint_list)))

    def _push_callback(self, future, waypoint_count: int):
        try:
            response = future.result()
            if getattr(response, 'success', False):
                self.total_waypoints = waypoint_count
                self.current_waypoint_seq = 0
                self._publish_waypoint_ack('SUCCESS', 'Waypoint upload completed')
            else:
                self._publish_waypoint_ack('FAILED', 'Upload waypoint ditolak oleh flight controller')
        except Exception as exc:
            self._publish_waypoint_ack('FAILED', f'Waypoint upload gagal: {exc}')

    def _publish_waypoint_ack(self, status: str, message: str):
        payload = {
            'vehicle_code': self.vehicle_id,
            'status': status,
            'message': message,
            'command': 'WAYPOINT_UPLOAD',
        }
        payload_json = json.dumps(payload, separators=(',', ':'))

        ros_msg = String()
        ros_msg.data = payload_json
        self.pub_waypoint_status.publish(ros_msg)

        if self._mqtt_connected:
            self._mqtt_client.publish(
                self.waypoint_status_topic,
                payload_json,
                qos=self.mqtt_qos,
                retain=False,
            )

        if status == 'SUCCESS':
            self.get_logger().info(f'Waypoint ACK published: {payload_json}')
        else:
            self.get_logger().warn(f'Waypoint ACK published: {payload_json}')

    def _wait_for_service(self, client, service_name: str, retries: int = 5, timeout: float = 1.0) -> bool:
        for _ in range(retries):
            if client.wait_for_service(timeout_sec=timeout):
                return True
        self.get_logger().error(f'{service_name} tidak tersedia')
        return False

    def _distance_to_wp(self, wp_lat, wp_lon, cur_lat, cur_lon):
        """Hitung jarak haversine dari posisi sekarang ke waypoint (meter)"""
        if cur_lat == 0.0 and cur_lon == 0.0:
            return None
        R = 6371000
        phi1 = math.radians(cur_lat)
        phi2 = math.radians(wp_lat)
        dphi = math.radians(wp_lat - cur_lat)
        dlam = math.radians(wp_lon - cur_lon)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)), 1)

    def publish_status(self):
        current_wp = None
        for wp in self.waypoints:
            if wp['seq'] == self.current_waypoint_seq:
                current_wp = wp
                break

        data = {
            'vehicle_id': self.vehicle_id,
            'connected': self.mavros_connected,
            'armed': self.armed,
            'mode': self.mode,
            'mission_active': self.mission_active,
            'current_wp_seq': self.current_waypoint_seq,
            'total_waypoints': self.total_waypoints,
            'last_reached_seq': self.last_reached_seq,
            'remaining_waypoints': max(0, self.total_waypoints - self.current_waypoint_seq - 1),
            'home': {
                'lat': self.home_lat,
                'lon': self.home_lon,
                'alt': self.home_alt,
            },
            'current_waypoint': current_wp,
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub_status.publish(msg)

    def destroy_node(self):
        try:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
