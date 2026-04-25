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
import csv
import os
import queue
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime
from zoneinfo import ZoneInfo

import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import CommandLong, WaypointPush
from rclpy.node import Node
from std_msgs.msg import String


_TZ = ZoneInfo('Asia/Jakarta')


def _now_iso() -> str:
    return datetime.now(_TZ).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]


def _duration_ms(start_iso: str, end_iso: str) -> int:
    try:
        fmt = '%Y-%m-%dT%H:%M:%S.%f'
        t0 = datetime.strptime(start_iso.rstrip('Z'), fmt)
        t1 = datetime.strptime(end_iso.rstrip('Z'), fmt)
        return int((t1 - t0).total_seconds() * 1000)
    except Exception:
        return -1


class _DailyCsvWriter:
    """CSV writer dengan daily rotation — file baru setiap hari (YYYYMMDD)."""

    def __init__(self, log_dir: str, prefix: str, fieldnames: list):
        self._log_dir = log_dir
        self._prefix = prefix
        self._fieldnames = fieldnames
        self._date = None
        self._fh = None
        self._writer = None
        os.makedirs(log_dir, exist_ok=True)

    def _rotate(self):
        today = datetime.now().strftime('%Y%m%d')
        if today == self._date:
            return
        if self._fh:
            self._fh.close()
        self._date = today
        path = os.path.join(self._log_dir, f'{self._prefix}_{today}.csv')
        first = not os.path.exists(path)
        self._fh = open(path, 'a', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._fh, fieldnames=self._fieldnames, extrasaction='ignore')
        if first:
            self._writer.writeheader()

    def write(self, row: dict):
        self._rotate()
        self._writer.writerow(row)
        self._fh.flush()

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None


_WAYPOINT_FIELDS = [
    'waypoint_received_timestamp',
    'vehicle_code',
    'waypoint_count',
    'set_home_from_first',
    'mavlink_upload_start',
    'mavlink_upload_end',
    'execution_result',
    'execution_message',
    'status_publish_timestamp',
    'duration_ms',
]


class WaypointNode(Node):

    def __init__(self) -> None:
        super().__init__('waypoint_node')

        # ── Parameter ──────────────────────────────────────────────────────
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('transport.mode', 'mqtt')
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
        self.declare_parameter('mission.auto_append_rtl_on_complete', True)
        self.declare_parameter('api.base_url', 'https://api.seano.cloud')
        self.declare_parameter('api.auth.type', 'none')
        self.declare_parameter('api.auth.api_key', '')
        self.declare_parameter('api.auth.jwt', '')
        self.declare_parameter('api.timeout_sec', 5.0)
        self.declare_parameter('api.queue_size', 100)
        self.declare_parameter('api.mission_poll_interval_sec', 3.0)
        self.declare_parameter('api.mission_poll_limit', 1)

        # ── Ambil nilai parameter ──────────────────────────────────────────
        self._vehicle_id   = self.get_parameter('vehicle.id').value
        self._transport_mode = str(self.get_parameter('transport.mode').value).strip().lower()
        if self._transport_mode not in ('mqtt', 'api', 'both'):
            self._transport_mode = 'mqtt'
        self._enable_mqtt = self._transport_mode in ('mqtt', 'both')
        self._enable_api = self._transport_mode in ('api', 'both')
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
        self._auto_append_rtl = bool(
            self.get_parameter('mission.auto_append_rtl_on_complete').value
        )

        self._waypoint_topic = f"{self._base_topic}/{self._vehicle_id}/waypoint"
        self._status_topic   = f"{self._base_topic}/{self._vehicle_id}/waypoint/response"

        # ── ROS service clients + publisher ───────────────────────────────
        self._wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self._cmd_client     = self.create_client(CommandLong, '/mavros/cmd/command')
        self._status_pub     = self.create_publisher(String, 'waypoint_status', 10)

        # ── Waypoint log context ───────────────────────────────────────────
        self._wp_log: dict = {}  # running context, overwritten per request

        # ── CSV Logger ─────────────────────────────────────────────────────
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._waypoint_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'command'),
            'waypoint_log',
            _WAYPOINT_FIELDS,
        )

        # ── MQTT client ────────────────────────────────────────────────────
        self._client = None
        if self._enable_mqtt:
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
                self._client.loop_start()
                # connect_async keeps node alive when DNS/network is not ready at boot.
                self._client.connect_async(self._broker, self._port, keepalive=self._keepalive)
                self.get_logger().info(
                    f"Waypoint MQTT connect scheduled: {self._broker}:{self._port}"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"Gagal startup MQTT (node tetap jalan, retry otomatis): {exc}"
                )
        else:
            self.get_logger().info('MQTT disabled (transport.mode=api)')

        # ── API config ─────────────────────────────────────────────────────
        self._api_base_url = str(self.get_parameter('api.base_url').value).rstrip('/')
        self._api_auth_type = str(self.get_parameter('api.auth.type').value).strip().lower()
        self._api_key = str(self.get_parameter('api.auth.api_key').value)
        if not self._api_key:
            self._api_key = os.getenv('SEANO_API_KEY', '')
        self._api_jwt = str(self.get_parameter('api.auth.jwt').value)
        if not self._api_jwt:
            self._api_jwt = os.getenv('SEANO_API_JWT', '')
        self._api_timeout_sec = float(self.get_parameter('api.timeout_sec').value)
        self._api_queue_size = int(self.get_parameter('api.queue_size').value)
        self._api_poll_interval = float(self.get_parameter('api.mission_poll_interval_sec').value)
        self._api_poll_limit = int(self.get_parameter('api.mission_poll_limit').value)
        self._api_queue = None
        self._api_thread = None
        self._api_running = False
        self._api_poll_thread = None
        self._api_poll_running = False
        self._api_auth_warned = False
        self._last_waypoint_id = None
        self._api_wp_pending = None

        if self._enable_api and not self._api_base_url:
            self.get_logger().warn('API enabled but api.base_url is empty, disabling API')
            self._enable_api = False

        if self._enable_api:
            self._start_api_worker()
            self._start_api_polling()

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

        received_ts = _now_iso()
        self._wp_log = {
            'waypoint_received_timestamp': received_ts,
            'vehicle_code': self._vehicle_id,
            'mavlink_upload_start': '',
            'mavlink_upload_end': '',
        }
        self._handle_waypoint(payload)

    # ── Logika waypoint ───────────────────────────────────────────────────

    def _handle_waypoint(self, payload: Any) -> None:
        waypoints_data = self._extract_waypoints(payload)
        self.get_logger().info(f"📍 Menerima {len(waypoints_data)} waypoint")

        self._wp_log['waypoint_count'] = len(waypoints_data)
        self._wp_log['set_home_from_first'] = self._should_set_home(payload)

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

        if self._should_append_rtl(payload) and waypoint_list[-1].command != 20:
            waypoint_list.append(self._build_rtl_waypoint())
            self.get_logger().info("Menambahkan RTL sebagai item terakhir mission")

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

    def _should_append_rtl(self, payload: Any) -> bool:
        if not isinstance(payload, dict):
            return self._auto_append_rtl

        override = payload.get('auto_rtl_on_complete')
        if override is None:
            override = payload.get('append_rtl_on_complete')
        if override is None:
            return self._auto_append_rtl
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

    def _build_rtl_waypoint(self) -> Waypoint:
        wp = Waypoint()
        wp.frame = 3
        wp.command = 20  # MAV_CMD_NAV_RETURN_TO_LAUNCH
        wp.is_current = False
        wp.autocontinue = True
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
        self._wp_log['mavlink_upload_start'] = _now_iso()

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = waypoints

        future = self._wp_push_client.call_async(req)
        future.add_done_callback(
            lambda f: self._push_callback(f, len(waypoints))
        )

    def _push_callback(self, future, wp_count: int) -> None:
        self._wp_log['mavlink_upload_end'] = _now_iso()
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
        status_publish_ts = _now_iso()
        data = json.dumps({
            "status": "success" if success else "error",
            "message": message,
            "vehicle_id": self._vehicle_id,
        })
        msg = String()
        msg.data = data
        self._status_pub.publish(msg)
        if self._client is not None:
            self._client.publish(self._status_topic, data, qos=self._qos)

        # Write waypoint log
        ctx = self._wp_log
        self._waypoint_csv.write({
            'waypoint_received_timestamp': ctx.get('waypoint_received_timestamp', ''),
            'vehicle_code': ctx.get('vehicle_code', self._vehicle_id),
            'waypoint_count': ctx.get('waypoint_count', ''),
            'set_home_from_first': ctx.get('set_home_from_first', ''),
            'mavlink_upload_start': ctx.get('mavlink_upload_start', ''),
            'mavlink_upload_end': ctx.get('mavlink_upload_end', ''),
            'execution_result': 'SUCCESS' if success else 'FAILED',
            'execution_message': message,
            'status_publish_timestamp': status_publish_ts,
            'duration_ms': _duration_ms(ctx.get('waypoint_received_timestamp', ''), status_publish_ts),
        })

        if self._enable_api and self._api_wp_pending:
            status = 'ok' if success else 'error'
            payload = {
                'vehicle_code': ctx.get('vehicle_code', self._vehicle_id),
                'waypoint_log_id': self._api_wp_pending,
                'status': status,
                'message': message,
                'timestamp': status_publish_ts,
            }
            self._api_enqueue('POST', '/waypoint-acks', payload)
            self._api_wp_pending = None

    def _wait_for_service(self, client, name: str, retries: int = 5, timeout: float = 1.0) -> bool:
        for attempt in range(1, retries + 1):
            if client.wait_for_service(timeout_sec=timeout):
                return True
            self.get_logger().warn(f"Menunggu {name} ({attempt}/{retries})")
        self.get_logger().error(f"{name} tidak tersedia setelah {retries} percobaan")
        return False

    def destroy_node(self) -> None:
        self._waypoint_csv.close()
        if self._client is not None:
            self._client.loop_stop()
            self._client.disconnect()

        if self._api_poll_running:
            self._api_poll_running = False
            if self._api_poll_thread is not None:
                self._api_poll_thread.join(timeout=2.0)

        if self._api_running:
            self._api_running = False
            if self._api_queue is not None:
                try:
                    self._api_queue.put_nowait(None)
                except queue.Full:
                    pass
            if self._api_thread is not None:
                self._api_thread.join(timeout=2.0)
        super().destroy_node()

    def _api_headers(self):
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'curl/8.5.0',
        }
        if self._api_auth_type == 'apikey':
            if self._api_key:
                headers['X-API-Key'] = self._api_key
            elif not self._api_auth_warned:
                self.get_logger().warn('API auth type apikey but api key is empty')
                self._api_auth_warned = True
        elif self._api_auth_type == 'jwt':
            if self._api_jwt:
                headers['Authorization'] = f'Bearer {self._api_jwt}'
            elif not self._api_auth_warned:
                self.get_logger().warn('API auth type jwt but token is empty')
                self._api_auth_warned = True
        return headers

    def _start_api_worker(self):
        self._api_queue = queue.Queue(maxsize=max(1, self._api_queue_size))
        self._api_running = True
        self._api_thread = threading.Thread(target=self._api_worker_loop, daemon=True)
        self._api_thread.start()

    def _api_worker_loop(self):
        while self._api_running:
            try:
                item = self._api_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if item is None:
                break

            method, endpoint, payload = item
            try:
                self._api_post_json(method, endpoint, payload)
            finally:
                self._api_queue.task_done()

    def _api_enqueue(self, method: str, endpoint: str, payload: dict):
        if not self._enable_api or self._api_queue is None:
            return
        try:
            self._api_queue.put_nowait((method, endpoint, payload))
        except queue.Full:
            self.get_logger().warn('API queue full, dropping payload')

    def _api_post_json(self, method: str, endpoint: str, payload: dict):
        if not self._api_base_url:
            return

        url = f"{self._api_base_url}{endpoint}"
        body = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        req = urllib.request.Request(url, data=body, headers=self._api_headers(), method=method)

        try:
            with urllib.request.urlopen(req, timeout=self._api_timeout_sec) as resp:
                _ = resp.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='ignore')
            self.get_logger().warn(
                f"API {method} {endpoint} failed: {exc.code} {exc.reason} {detail[:200]}"
            )
        except Exception as exc:
            self.get_logger().warn(f"API {method} {endpoint} error: {exc}")

    def _api_get_json(self, endpoint: str):
        if not self._api_base_url:
            return None

        url = f"{self._api_base_url}{endpoint}"
        req = urllib.request.Request(url, headers=self._api_headers(), method='GET')
        try:
            with urllib.request.urlopen(req, timeout=self._api_timeout_sec) as resp:
                data = resp.read().decode('utf-8', errors='ignore')
                return json.loads(data) if data else None
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='ignore')
            self.get_logger().warn(
                f"API GET {endpoint} failed: {exc.code} {exc.reason} {detail[:200]}"
            )
        except Exception as exc:
            self.get_logger().warn(f"API GET {endpoint} error: {exc}")
        return None

    def _start_api_polling(self):
        self._api_poll_running = True
        self._api_poll_thread = threading.Thread(target=self._api_poll_loop, daemon=True)
        self._api_poll_thread.start()

    def _api_poll_loop(self):
        interval = max(0.5, self._api_poll_interval)
        limit = max(1, self._api_poll_limit)
        while self._api_poll_running:
            try:
                query = f"/missions/pending-upload?vehicle_code={self._vehicle_id}&limit={limit}"
                payload = self._api_get_json(query)
                if isinstance(payload, dict):
                    data = payload.get('data')
                    if isinstance(data, list) and data:
                        item = data[0]
                        waypoint_log_id = item.get('waypoint_log_id') or item.get('id')
                        if waypoint_log_id and waypoint_log_id == self._last_waypoint_id:
                            time.sleep(interval)
                            continue
                        self._last_waypoint_id = waypoint_log_id
                        self._api_wp_pending = waypoint_log_id
                        mission_payload = item.get('payload') if isinstance(item, dict) else None
                        if mission_payload is None:
                            mission_payload = item

                        received_ts = _now_iso()
                        self._wp_log = {
                            'waypoint_received_timestamp': received_ts,
                            'vehicle_code': self._vehicle_id,
                            'mavlink_upload_start': '',
                            'mavlink_upload_end': '',
                        }
                        self._handle_waypoint(mission_payload)
            except Exception as exc:
                self.get_logger().warn(f"API mission polling error: {exc}")
            time.sleep(interval)


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
