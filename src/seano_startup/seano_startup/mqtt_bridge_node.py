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
from datetime import datetime
from zoneinfo import ZoneInfo
from functools import partial

_TZ = ZoneInfo('Asia/Jakarta')


def _now_iso() -> str:
    return datetime.now(_TZ).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]


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
    'usv_timestamp', 'vehicle_code',
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
        self.declare_parameter('mqtt.broker', 'localhost')
        self.declare_parameter('mqtt.port', 1883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.raw_topic_suffix', 'raw')
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

            self.client.tls_set(cert_reqs=ssl.CERT_NONE)
            self.client.tls_insecure_set(True)

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

    def _connect_mqtt(self, initial=False):
        if not self._enable_mqtt or self.client is None:
            return
        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
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
        mqtt_publish_ts = _now_iso()
        if self._mqtt_connected:
            self.client.publish(
                self.mqtt_topic,
                msg.data,
                qos=self.qos
            )
            self._publish_raw(msg.data)
        row = self._safe_json(msg.data)
        if isinstance(row, dict):
            row['mqtt_publish_timestamp'] = mqtt_publish_ts
            self._telemetry_csv.write(row)
            if self._enable_api:
                self._api_enqueue('POST', '/vehicle-logs', row)
                self._api_enqueue(
                    'POST',
                    '/raw-logs',
                    {
                        'vehicle_code': row.get('vehicle_code', self.vehicle_id),
                        'logs': msg.data,
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
        self._publish_raw(msg.data)

        if not self._enable_api:
            return

        payload = msg.data
        data = self._safe_json(payload)

        if source == 'sensor':
            if isinstance(data, dict) and data.get('sensor_code'):
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
