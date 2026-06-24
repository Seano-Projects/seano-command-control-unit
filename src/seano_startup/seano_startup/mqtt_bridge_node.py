import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import paho.mqtt.client as mqtt
import ssl
import json
import csv
import os
import queue
import threading
import time
import urllib.error
import urllib.request
import math
from datetime import datetime
from zoneinfo import ZoneInfo
from functools import partial

_ID_TZ_WIB = ZoneInfo('Asia/Jakarta')
_ID_TZ_WITA = ZoneInfo('Asia/Makassar')
_ID_TZ_WIT = ZoneInfo('Asia/Jayapura')
_DEFAULT_TZ = _ID_TZ_WIB


def _gps_ok(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value > 0
    if isinstance(value, str):
        return value.strip().lower() in {'true', '1', 'yes', 'ok'}
    return False


def _coords_valid(lat, lon) -> bool:
    try:
        lat_f = float(lat)
        lon_f = float(lon)
    except (TypeError, ValueError):
        return False
    if not (-90.0 <= lat_f <= 90.0 and -180.0 <= lon_f <= 180.0):
        return False
    if abs(lat_f) < 1e-6 and abs(lon_f) < 1e-6:
        return False
    return True


def _resolve_timezone(lat=None, lon=None, gps_ok=None):
    if not _gps_ok(gps_ok) or not _coords_valid(lat, lon):
        return _DEFAULT_TZ
    lon_f = float(lon)
    if lon_f < 112.5:
        return _ID_TZ_WIB
    if lon_f < 127.5:
        return _ID_TZ_WITA
    return _ID_TZ_WIT


def _has_valid_coords(lat=None, lon=None, gps_ok=None) -> bool:
    return _gps_ok(gps_ok) and _coords_valid(lat, lon)


def _now_iso(lat=None, lon=None, gps_ok=None) -> str:
    tz = _resolve_timezone(lat, lon, gps_ok)
    return datetime.now(tz).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]


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


_TELEMETRY_FIELDS = [
    'date_time', 'vehicle_code',
    'battery_voltage', 'battery_current', 'battery_percentage',
    'rssi', 'mode', 'latitude', 'longitude', 'altitude',
    'heading', 'armed', 'gps_ok', 'system_status',
    'speed', 'roll', 'pitch', 'yaw', 'temperature_system',
    'mqtt_publish_timestamp',
]

class MqttBridgeNode(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('transport.mode', 'mqtt')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.raw_topic_suffix', 'raw')
        self.declare_parameter('mqtt.alert_topic', '')
        self.declare_parameter('telemetry.gps_alert_interval_sec', 30.0)
        self.declare_parameter('telemetry.gps_alert_max_duration_sec', 3600.0)
        self.declare_parameter('validation.force_gps_fix_for_publish', True)
        self.declare_parameter('validation.allow_fallback_coords_without_gps', False)
        self.declare_parameter('validation.fallback_latitude', 0.0)
        self.declare_parameter('validation.fallback_longitude', 0.0)
        self.declare_parameter('validation.fallback_motion_enabled', True)
        self.declare_parameter('validation.fallback_motion_radius_m', 25.0)
        self.declare_parameter('validation.fallback_motion_period_sec', 180.0)
        self.declare_parameter('validation.fallback_motion_north_scale', 0.7)
        self.declare_parameter('api.base_url', 'https://api.seano.cloud')
        self.declare_parameter('api.auth.type', 'none')
        self.declare_parameter('api.auth.api_key', '')
        self.declare_parameter('api.auth.jwt', '')
        self.declare_parameter('api.timeout_sec', 5.0)
        self.declare_parameter('api.queue_size', 200)

        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.transport_mode = str(self.get_parameter('transport.mode').value).strip().lower()
        if self.transport_mode not in ('mqtt', 'api', 'both'):
            self.get_logger().warn(
                f"Unknown transport.mode='{self.transport_mode}', fallback to 'mqtt'"
            )
            self.transport_mode = 'mqtt'
        self._enable_mqtt = self.transport_mode in ('mqtt', 'both')
        self._enable_api = self.transport_mode in ('api', 'both')
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.qos = int(self.get_parameter('mqtt.qos').value)
        self.raw_topic_suffix = self.get_parameter('mqtt.raw_topic_suffix').value
        self.alert_mqtt_topic = str(self.get_parameter('mqtt.alert_topic').value).strip()
        self.gps_alert_interval_sec = max(
            1.0,
            float(self.get_parameter('telemetry.gps_alert_interval_sec').value)
        )
        self.gps_alert_max_duration_sec = max(
            0.0,
            float(self.get_parameter('telemetry.gps_alert_max_duration_sec').value)
        )
        self.force_gps_fix_for_publish = bool(
            self.get_parameter('validation.force_gps_fix_for_publish').value
        )
        self.allow_fallback_coords_without_gps = bool(
            self.get_parameter('validation.allow_fallback_coords_without_gps').value
        )
        self.fallback_latitude = float(
            self.get_parameter('validation.fallback_latitude').value
        )
        self.fallback_longitude = float(
            self.get_parameter('validation.fallback_longitude').value
        )
        self.fallback_motion_enabled = bool(
            self.get_parameter('validation.fallback_motion_enabled').value
        )
        self.fallback_motion_radius_m = max(
            0.0,
            float(self.get_parameter('validation.fallback_motion_radius_m').value)
        )
        self.fallback_motion_period_sec = max(
            20.0,
            float(self.get_parameter('validation.fallback_motion_period_sec').value)
        )
        self.fallback_motion_north_scale = max(
            0.1,
            min(1.0, float(self.get_parameter('validation.fallback_motion_north_scale').value))
        )
        self._fallback_motion_t0 = time.monotonic()
        if not self.alert_mqtt_topic:
            self.alert_mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/alert"
        self._gps_alert_last_ts = 0.0
        self._gps_alert_start_ts = 0.0
        self._gps_ready_prev = False

        self.mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/telemetry"
        self.failsafe_mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/failsafe"
        self.mission_status_mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/mission/status"
        self.mission_reached_mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/mission/waypoint_reached"
        self.raw_mqtt_topic = f"{self.base_topic}/{self.vehicle_id}/{self.raw_topic_suffix}"

        # CSV Logger
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._telemetry_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'telemetry'),
            'telemetry_log',
            _TELEMETRY_FIELDS,
        )

        self.client = None
        self._mqtt_connected = False
        self._last_connect_log = ''

        if self._enable_mqtt:
            self.client = mqtt.Client()
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect

            if self.mqtt_username:
                self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

            self.client.tls_set(cert_reqs=ssl.CERT_REQUIRED)

            self.client.loop_start()
            self._connect_mqtt(initial=True)
        else:
            self.get_logger().info('MQTT disabled (transport.mode=api)')

        # API config
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
        self._api_queue = None
        self._api_thread = None
        self._api_running = False
        self._api_auth_warned = False
        self._last_api_drop_log = 0.0

        if self._enable_api and not self._api_base_url:
            self.get_logger().warn('API enabled but api.base_url is empty, disabling API')
            self._enable_api = False

        if self._enable_api:
            self._start_api_worker()

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
        self.create_subscription(
            String,
            'mission/status',
            self.mission_status_callback,
            10
        )
        self.create_subscription(
            String,
            'mission/waypoint_reached',
            self.mission_reached_callback,
            10
        )
        self.create_subscription(
            String,
            'oceanography/ctd',
            partial(self.raw_string_callback, source='sensor'),
            10
        )
        self.create_subscription(
            String,
            'command_status',
            partial(self.raw_string_callback, source='command_status'),
            10
        )
        self.create_subscription(
            String,
            'waypoint_status',
            partial(self.raw_string_callback, source='waypoint_status'),
            10
        )
        self.create_subscription(
            String,
            'communication/status',
            partial(self.raw_string_callback, source='communication'),
            10
        )
        self.create_subscription(
            String,
            'anti_theft/alert',
            partial(self.raw_string_callback, source='anti_theft'),
            10
        )
        self.create_subscription(
            String,
            'raw/log',
            self.raw_log_callback,
            50
        )
        self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            50
        )

        # Reconnect watchdog to survive transient DNS/network failures after reboot.
        if self._enable_mqtt:
            self.create_timer(10.0, self._reconnect_if_needed)
        self.get_logger().info(
            f"force_gps_fix_for_publish={self.force_gps_fix_for_publish}"
        )
        self.get_logger().info(
            f"allow_fallback_coords_without_gps={self.allow_fallback_coords_without_gps}"
        )
        self.get_logger().info(
            f"fallback_motion_enabled={self.fallback_motion_enabled}, "
            f"radius_m={self.fallback_motion_radius_m}, "
            f"period_sec={self.fallback_motion_period_sec}"
        )

    def _offset_m_to_coords(self, center_lat, center_lon, east_m, north_m):
        # Convert local EN offset (meters) to lat/lon around center point.
        lat_scale = 111320.0
        lon_scale = 111320.0 * max(0.1, math.cos(math.radians(float(center_lat))))
        lat = float(center_lat) + (north_m / lat_scale)
        lon = float(center_lon) + (east_m / lon_scale)
        return lat, lon

    def _next_fallback_coords(self):
        center_lat = self.fallback_latitude
        center_lon = self.fallback_longitude
        if not self.fallback_motion_enabled or self.fallback_motion_radius_m <= 0.0:
            return round(center_lat, 7), round(center_lon, 7)

        t = time.monotonic() - self._fallback_motion_t0
        omega = (2.0 * math.pi) / self.fallback_motion_period_sec

        # Smooth pseudo-route that keeps moving but remains in a bounded area.
        east_m = self.fallback_motion_radius_m * math.sin(omega * t)
        north_m = (
            self.fallback_motion_radius_m
            * self.fallback_motion_north_scale
            * math.sin((omega * 0.63 * t) + 0.9)
        )
        lat, lon = self._offset_m_to_coords(center_lat, center_lon, east_m, north_m)
        return round(lat, 7), round(lon, 7)

    def _maybe_publish_gps_alert(self, lat=None, lon=None, gps_ok=None):
        gps_ready = _has_valid_coords(lat, lon, gps_ok)
        if gps_ready:
            self._gps_ready_prev = True
            self._gps_alert_last_ts = 0.0
            self._gps_alert_start_ts = 0.0
            return

        if self._gps_ready_prev:
            self._gps_ready_prev = False
            self._gps_alert_last_ts = 0.0
            self._gps_alert_start_ts = 0.0

        now = time.monotonic()
        if self._gps_alert_start_ts == 0.0:
            self._gps_alert_start_ts = now
        if self.gps_alert_max_duration_sec > 0.0:
            if (now - self._gps_alert_start_ts) >= self.gps_alert_max_duration_sec:
                return
        if (now - self._gps_alert_last_ts) < self.gps_alert_interval_sec:
            return

        payload = {
            'vehicle_code': self.vehicle_id,
            'message': 'GPS no fix; telemetry/sensor skipped',
            'severity': 'warning',
            'alert_type': 'GPS',
        }
        if _coords_valid(lat, lon):
            payload['latitude'] = round(float(lat), 6)
            payload['longitude'] = round(float(lon), 6)

        if self.client is not None:
            self.client.publish(
                self.alert_mqtt_topic,
                json.dumps(payload, separators=(',', ':')),
                qos=self.qos,
                retain=False
            )

        if self._enable_api:
            alert_payload = self._normalize_alert(payload, 'gps', payload['message'])
            self._api_enqueue('POST', '/alerts', alert_payload)

        self._gps_alert_last_ts = now

    def _connect_mqtt(self, initial=False):
        if not self._enable_mqtt or self.client is None:
            return
        try:
            self.client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=60)
            if initial:
                self.get_logger().info(
                    f"Connecting to MQTT broker {self.mqtt_broker}:{self.mqtt_port} ..."
                )
        except Exception as e:
            msg = f"MQTT connect pending: {e}"
            if msg != self._last_connect_log:
                self.get_logger().warn(msg)
                self._last_connect_log = msg

    def _reconnect_if_needed(self):
        if self._mqtt_connected:
            return
        self._connect_mqtt(initial=False)

    def _on_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = (rc == 0)
        if self._mqtt_connected:
            self._last_connect_log = ''
            self.get_logger().info(f"Connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}")
            self.get_logger().info(f"Publish topic: {self.mqtt_topic}")
            self.get_logger().info(f"Publish topic: {self.failsafe_mqtt_topic}")
            self.get_logger().info(f"Publish topic: {self.mission_status_mqtt_topic}")
            self.get_logger().info(f"Publish topic: {self.mission_reached_mqtt_topic}")
            self.get_logger().info(f"Publish topic: {self.raw_mqtt_topic}")
        else:
            self.get_logger().warn(f"MQTT connect failed rc={rc}; retrying")

    def _on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(f"MQTT disconnected unexpectedly rc={rc}; retrying")

    def telemetry_callback(self, msg):
        row = self._safe_json(msg.data)
        if not isinstance(row, dict):
            return

        lat = row.get('latitude')
        lon = row.get('longitude')
        gps_ok = row.get('gps_ok')
        if self.force_gps_fix_for_publish:
            self._maybe_publish_gps_alert(lat, lon, gps_ok)
            has_fix = _has_valid_coords(lat, lon, gps_ok)
            if (
                not has_fix
                and self.allow_fallback_coords_without_gps
                and _coords_valid(self.fallback_latitude, self.fallback_longitude)
            ):
                fb_lat, fb_lon = self._next_fallback_coords()
                row['latitude'] = fb_lat
                row['longitude'] = fb_lon
                row['gps_ok'] = False
                lat = row['latitude']
                lon = row['longitude']
                gps_ok = row['gps_ok']
                has_fix = True
            if not has_fix:
                return

        publish_payload = json.dumps(row)
        row['mqtt_publish_timestamp'] = _now_iso(lat, lon, gps_ok)
        self._telemetry_csv.write(row)

        if self._mqtt_connected:
            self.client.publish(
                self.mqtt_topic,
                publish_payload,
                qos=self.qos
            )
            self._publish_raw(publish_payload)

        if self._enable_api:
            self._api_enqueue('POST', '/vehicle-logs', row)
            self._api_enqueue(
                'POST',
                '/raw-logs',
                {
                    'vehicle_code': row.get('vehicle_code', self.vehicle_id),
                    'logs': publish_payload,
                }
            )

    def failsafe_callback(self, msg):
        if self._mqtt_connected:
            self.client.publish(
                self.failsafe_mqtt_topic,
                msg.data,
                qos=1,
                retain=False
            )
            self._publish_raw(msg.data)
        if self._enable_api:
            data = self._safe_json(msg.data)
            alert_payload = self._normalize_alert(data, 'failsafe', msg.data)
            self._api_enqueue('POST', '/alerts', alert_payload)

    def mission_status_callback(self, msg):
        if self._mqtt_connected:
            self.client.publish(
                self.mission_status_mqtt_topic,
                msg.data,
                qos=self.qos,
                retain=False
            )
            self._publish_raw(msg.data)

    def mission_reached_callback(self, msg):
        if self._mqtt_connected:
            self.client.publish(
                self.mission_reached_mqtt_topic,
                msg.data,
                qos=self.qos,
                retain=False
            )
            self._publish_raw(msg.data)
        if self._enable_api:
            data = self._safe_json(msg.data) or {}
            if isinstance(data, dict):
                if 'vehicle_code' not in data and 'vehicle_id' not in data:
                    data['vehicle_code'] = self.vehicle_id
                if 'timestamp' not in data:
                    data['timestamp'] = _now_iso()
                self._api_enqueue('POST', '/missions/waypoint-reached', data)

    def raw_string_callback(self, msg: String, source: str = 'raw'):
        if source == 'sensor':
            data = self._safe_json(msg.data)
            if not isinstance(data, dict) or not data.get('sensor_code'):
                return
            lat = data.get('latitude')
            lon = data.get('longitude')
            gps_ok = data.get('gps_ok')

            if self.force_gps_fix_for_publish:
                has_fix = _has_valid_coords(lat, lon, gps_ok)
                if (
                    not has_fix
                    and self.allow_fallback_coords_without_gps
                    and _coords_valid(self.fallback_latitude, self.fallback_longitude)
                ):
                    fb_lat, fb_lon = self._next_fallback_coords()
                    data['latitude'] = fb_lat
                    data['longitude'] = fb_lon
                    data['gps_ok'] = False
                    has_fix = True
                if not has_fix:
                    return

            payload = json.dumps(data)

            self._publish_raw(payload)

            if not self._enable_api:
                return

            body = {
                'vehicle_code': data.get('vehicle_code', self.vehicle_id),
                'sensor_code': data.get('sensor_code'),
                'data': payload,
            }
            self._api_enqueue('POST', '/sensor-logs', body)
            self._api_enqueue(
                'POST',
                '/raw-logs',
                {
                    'vehicle_code': data.get('vehicle_code', self.vehicle_id),
                    'logs': payload,
                }
            )
            return

        self._publish_raw(msg.data)

        if not self._enable_api:
            return

        payload = msg.data
        data = self._safe_json(payload)

        if source == 'anti_theft':
            alert_payload = self._normalize_alert(data, 'antitheft', payload)
            self._api_enqueue('POST', '/alerts', alert_payload)
            return

    def _publish_raw(self, payload: str):
        if self._mqtt_connected and payload:
            self.client.publish(
                self.raw_mqtt_topic,
                payload,
                qos=self.qos,
                retain=False
            )

    def raw_log_callback(self, msg: String):
        # Forward payload as-is so backend can parse JSON or plain text.
        self._publish_raw(msg.data.strip())

    def rosout_callback(self, msg: Log):
        # Fallback raw stream from ROS logs in compact JSON format.
        payload = {
            'vehicle_code': self.vehicle_id,
            'message': f"[{msg.name}] {msg.msg}",
            'level': int(msg.level),
            'sec': int(msg.stamp.sec),
            'nanosec': int(msg.stamp.nanosec),
        }
        raw = json.dumps(payload, separators=(',', ':'))
        self._publish_raw(raw)

    def _safe_json(self, payload: str):
        if not payload:
            return None
        try:
            return json.loads(payload)
        except Exception:
            return None

    def _normalize_alert(self, data, alert_type: str, fallback_message: str):
        if isinstance(data, dict):
            payload = dict(data)
        else:
            payload = {'message': fallback_message}

        if 'vehicle_code' not in payload and 'vehicle_id' not in payload:
            payload['vehicle_code'] = self.vehicle_id
        if alert_type and 'alert_type' not in payload:
            payload['alert_type'] = alert_type
        if 'message' not in payload and fallback_message:
            payload['message'] = fallback_message
        return payload

    def _start_api_worker(self):
        self._api_queue = queue.Queue(maxsize=max(1, self._api_queue_size))
        self._api_running = True
        self._api_thread = threading.Thread(target=self._api_worker_loop, daemon=True)
        self._api_thread.start()
        self.get_logger().info('API worker started')

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
            now = time.time()
            if (now - self._last_api_drop_log) >= 5.0:
                self.get_logger().warn('API queue full, dropping payloads')
                self._last_api_drop_log = now

    def _api_post_json(self, method: str, endpoint: str, payload: dict):
        if not self._api_base_url:
            return

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

        url = f"{self._api_base_url}{endpoint}"
        body = json.dumps(payload, separators=(',', ':')).encode('utf-8')

        req = urllib.request.Request(url, data=body, headers=headers, method=method)
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

    def destroy_node(self):
        self._telemetry_csv.close()
        if self._api_running:
            self._api_running = False
            if self._api_queue is not None:
                try:
                    self._api_queue.put_nowait(None)
                except queue.Full:
                    pass
            if self._api_thread is not None:
                self._api_thread.join(timeout=2.0)

        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
