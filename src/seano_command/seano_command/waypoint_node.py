#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_node — menerima waypoint dari MQTT lalu upload ke MAVROS.

MQTT topic yang disubscribe:
  seano/{vehicle_id}/waypoint

Format payload (3 bentuk diterima):

  Bentuk A — object dengan key "waypoints":
    {
      "set_home_from_first_waypoint": true,
      "waypoints": [
        {"lat": -6.2001, "lon": 106.8167, "alt": 5.0},
        {"lat": -6.2005, "lon": 106.8172, "alt": 5.0}
      ]
    }

  Bentuk B — array langsung:
    [{"lat": -6.2001, "lon": 106.8167, "alt": 5.0}, ...]

  Bentuk C — satu waypoint object:
    {"lat": -6.2001, "lon": 106.8167, "alt": 5.0}

Status hasil upload dipublish ke:
  MQTT : seano/{vehicle_id}/waypoint/response
  ROS  : waypoint_status  (std_msgs/String berisi JSON)
"""

import json
import ssl
from typing import Any, Dict, List, Optional

import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import CommandLong, WaypointPush
from rclpy.node import Node
from std_msgs.msg import String


class WaypointNode(Node):

    def __init__(self) -> None:
        super().__init__('waypoint_node')

        # ── Parameter ──────────────────────────────────────────────────────
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

        # ── Ambil nilai parameter ──────────────────────────────────────────
        self._vehicle_id   = self.get_parameter('vehicle.id').value
        self._broker       = self.get_parameter('mqtt.broker').value
        self._port         = int(self.get_parameter('mqtt.port').value)
        self._username     = self.get_parameter('mqtt.username').value
        self._password     = self.get_parameter('mqtt.password').value
        self._base_topic   = self.get_parameter('mqtt.base_topic').value
        self._qos          = int(self.get_parameter('mqtt.qos').value)
        self._keepalive    = int(self.get_parameter('mqtt.keepalive').value)
        self._use_tls      = bool(self.get_parameter('mqtt.use_tls').value)
        self._tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)
        self._auto_set_home = bool(
            self.get_parameter('mission.auto_set_home_from_first_waypoint').value
        )

        self._waypoint_topic = f"{self._base_topic}/{self._vehicle_id}/waypoint"
        self._status_topic   = f"{self._base_topic}/{self._vehicle_id}/waypoint/response"

        # ── ROS service clients + publisher ───────────────────────────────
        self._wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self._cmd_client     = self.create_client(CommandLong, '/mavros/cmd/command')
        self._status_pub     = self.create_publisher(String, 'waypoint_status', 10)

        # ── MQTT client ────────────────────────────────────────────────────
        self._client = mqtt.Client()
        self._client.on_connect    = self._on_connect
        self._client.on_message    = self._on_message
        self._client.on_disconnect = self._on_disconnect

        if self._username:
            self._client.username_pw_set(self._username, self._password)

        if self._use_tls:
            self._client.tls_set(cert_reqs=ssl.CERT_NONE)
            self._client.tls_insecure_set(self._tls_insecure)

        self._client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self._client.connect(self._broker, self._port, keepalive=self._keepalive)
            self._client.loop_start()
            self.get_logger().info(
                f"Waypoint node terhubung ke MQTT {self._broker}:{self._port}"
            )
        except Exception as exc:
            self.get_logger().error(f"Gagal koneksi MQTT: {exc}")
            raise SystemExit

        self.get_logger().info(
            f"Waypoint node aktif — vehicle: {self._vehicle_id} | "
            f"MQTT topic: {self._waypoint_topic} | "
            f"auto_set_home: {self._auto_set_home}"
        )

    # ── MQTT callbacks ────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc, properties=None) -> None:
        if rc == 0:
            client.subscribe(self._waypoint_topic, qos=self._qos)
            self.get_logger().info(f"MQTT subscribe: {self._waypoint_topic}")
        else:
            self.get_logger().error(f"MQTT connect gagal, rc={rc}")

    def _on_disconnect(self, client, userdata, rc, properties=None) -> None:
        if rc != 0:
            self.get_logger().warn("MQTT terputus, mencoba reconnect...")

    def _on_message(self, client, userdata, msg) -> None:
        try:
            payload = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            self.get_logger().error(f"Payload tidak valid: {exc}")
            return

        self._handle_waypoint(payload)

    # ── Logika waypoint ───────────────────────────────────────────────────

    def _handle_waypoint(self, payload: Any) -> None:
        waypoints_data = self._extract_waypoints(payload)
        self.get_logger().info(f"Menerima {len(waypoints_data)} waypoint")

        if not waypoints_data:
            self._publish_status("Tidak ada waypoint yang diberikan", False)
            return

        waypoint_list: List[Waypoint] = []
        for idx, wp_data in enumerate(waypoints_data):
            wp = self._build_waypoint(wp_data, idx == 0)
            if wp is not None:
                waypoint_list.append(wp)

        if not waypoint_list:
            self._publish_status("Tidak ada waypoint valid untuk diupload", False)
            return

        if self._should_set_home(payload):
            self._set_home_then_push(waypoint_list[0], waypoint_list)
        else:
            self._push_waypoints(waypoint_list)

    def _should_set_home(self, payload: Any) -> bool:
        if not isinstance(payload, dict):
            return self._auto_set_home

        override = payload.get('set_home_from_first_waypoint')
        if override is None:
            return self._auto_set_home
        if isinstance(override, bool):
            return override
        if isinstance(override, str):
            return override.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(override)

    def _extract_waypoints(self, payload: Any) -> List[Dict[str, Any]]:
        if isinstance(payload, dict):
            wps = payload.get('waypoints')
            if isinstance(wps, list):
                return [wp for wp in wps if isinstance(wp, dict)]
            if 'latitude' in payload or 'lat' in payload:
                return [payload]
            return []
        if isinstance(payload, list):
            return [wp for wp in payload if isinstance(wp, dict)]
        return []

    def _build_waypoint(self, wp_data: Dict[str, Any], is_current: bool) -> Optional[Waypoint]:
        latitude  = wp_data.get('latitude', wp_data.get('lat'))
        longitude = wp_data.get('longitude', wp_data.get('lon', wp_data.get('lng')))
        altitude  = wp_data.get('altitude', wp_data.get('alt', 0.0))

        if latitude is None or longitude is None:
            self.get_logger().warn("Waypoint dilewati: latitude/longitude tidak ada")
            return None

        try:
            lat = float(latitude)
            lon = float(longitude)
            alt = float(altitude)
        except (TypeError, ValueError):
            self.get_logger().warn("Waypoint dilewati: nilai lat/lon/alt bukan numerik")
            return None

        if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
            self.get_logger().warn("Waypoint dilewati: lat/lon di luar rentang")
            return None

        wp = Waypoint()
        wp.frame        = int(wp_data.get('frame', 3))
        wp.command      = int(wp_data.get('command', 16))
        wp.is_current   = is_current
        wp.autocontinue = bool(wp_data.get('autocontinue', True))
        wp.param1       = float(wp_data.get('param1', 0.0))
        wp.param2       = float(wp_data.get('param2', 0.0))
        wp.param3       = float(wp_data.get('param3', 0.0))
        wp.param4       = float(wp_data.get('param4', 0.0))
        wp.x_lat        = lat
        wp.y_long       = lon
        wp.z_alt        = alt
        return wp

    def _set_home_then_push(self, first_wp: Waypoint, waypoint_list: List[Waypoint]) -> None:
        if not self._wait_for_service(self._cmd_client, '/mavros/cmd/command'):
            self._publish_status("Set home gagal: service MAVROS tidak tersedia", False)
            return

        req = CommandLong.Request()
        req.command = 179   # MAV_CMD_DO_SET_HOME
        req.param1  = 0.0   # gunakan koordinat yang diberikan
        req.param5  = float(first_wp.x_lat)
        req.param6  = float(first_wp.y_long)
        req.param7  = float(first_wp.z_alt)

        self.get_logger().info(
            f"Set home dari waypoint pertama: "
            f"lat={req.param5}, lon={req.param6}, alt={req.param7}"
        )

        future = self._cmd_client.call_async(req)
        future.add_done_callback(
            lambda f: self._set_home_callback(f, waypoint_list)
        )

    def _set_home_callback(self, future, waypoint_list: List[Waypoint]) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Set home berhasil, lanjut upload waypoint")
                self._push_waypoints(waypoint_list)
            else:
                self.get_logger().error(f"Set home gagal: result={response.result}")
                self._publish_status(f"Set home gagal: kode {response.result}", False)
        except Exception as exc:
            self.get_logger().error(f"Set home error: {exc}")
            self._publish_status(f"Set home error: {exc}", False)

    def _push_waypoints(self, waypoints: List[Waypoint]) -> None:
        if not self._wait_for_service(self._wp_push_client, '/mavros/mission/push'):
            self._publish_status("Service waypoint tidak tersedia", False)
            return

        self.get_logger().info(f"Upload {len(waypoints)} waypoint ke MAVROS")

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = waypoints

        future = self._wp_push_client.call_async(req)
        future.add_done_callback(
            lambda f: self._push_callback(f, len(waypoints))
        )

    def _push_callback(self, future, wp_count: int) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Berhasil upload {wp_count} waypoint")
                self._publish_status(f"Upload {wp_count} waypoint berhasil", True)
            else:
                self.get_logger().error("Upload waypoint gagal")
                self._publish_status("Upload waypoint gagal", False)
        except Exception as exc:
            self.get_logger().error(f"Waypoint service error: {exc}")
            self._publish_status(f"Waypoint error: {exc}", False)

    # ── Helper ────────────────────────────────────────────────────────────

    def _publish_status(self, message: str, success: bool) -> None:
        data = json.dumps({
            "status": "success" if success else "error",
            "message": message,
            "vehicle_id": self._vehicle_id,
        })
        msg = String()
        msg.data = data
        self._status_pub.publish(msg)
        self._client.publish(self._status_topic, data, qos=self._qos)

    def _wait_for_service(self, client, name: str, retries: int = 5, timeout: float = 1.0) -> bool:
        for attempt in range(1, retries + 1):
            if client.wait_for_service(timeout_sec=timeout):
                return True
            self.get_logger().warn(f"Menunggu {name} ({attempt}/{retries})")
        self.get_logger().error(f"{name} tidak tersedia setelah {retries} percobaan")
        return False

    def destroy_node(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
