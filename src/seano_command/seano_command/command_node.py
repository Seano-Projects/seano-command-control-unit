import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from mavros_msgs.srv import CommandLong, SetMode
from mavros_msgs.msg import StatusText, State
from sensor_msgs.msg import NavSatFix
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

_TZ = ZoneInfo('Asia/Jakarta')


def _now_iso() -> str:
    return datetime.now(_TZ).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]


def _now_iso_utc() -> str:
    return datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')


def _duration_ms(start_iso: str, end_iso: str) -> int:
    try:
        fmt = '%Y-%m-%dT%H:%M:%S.%f'
        t0 = datetime.strptime(start_iso.rstrip('Z'), fmt)
        t1 = datetime.strptime(end_iso.rstrip('Z'), fmt)
        return int((t1 - t0).total_seconds() * 1000)
    except Exception:
        return -1


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


_COMMAND_FIELDS = [
    'command_received_timestamp',
    'vehicle_code',
    'command',
    'mavlink_sent_timestamp',
    'execution_result',
    'execution_message',
    'command_response_timestamp',
    'duration_ms',
]


_MAV_RESULT_LABELS = {
    0: 'ACCEPTED',
    1: 'TEMPORARILY_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
}

class CommandNode(Node):

    def __init__(self):
        super().__init__('command_node')

        # Declare parameters
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('transport.mode', 'mqtt')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', False)
        self.declare_parameter('api.base_url', 'https://api.seano.cloud')
        self.declare_parameter('api.auth.type', 'none')
        self.declare_parameter('api.auth.api_key', '')
        self.declare_parameter('api.auth.jwt', '')
        self.declare_parameter('api.timeout_sec', 5.0)
        self.declare_parameter('api.queue_size', 100)
        self.declare_parameter('api.command_poll_interval_sec', 2.0)
        self.declare_parameter('api.command_poll_limit', 1)
        self.declare_parameter('arming.require_gps_fix', True)
        self.declare_parameter('arming.gps_topic', '/mavros/global_position/global')
        self.declare_parameter('arming.gps_fix_min', 1)
        self.declare_parameter('arming.gps_fix_timeout_sec', 2.0)
        self.declare_parameter('arming.require_valid_coords', True)

        # Get parameters
        self.vehicle_id   = self.get_parameter('vehicle.id').value
        self.transport_mode = str(self.get_parameter('transport.mode').value).strip().lower()
        if self.transport_mode not in ('mqtt', 'api', 'both'):
            self.get_logger().warn(
                f"Unknown transport.mode='{self.transport_mode}', fallback to 'mqtt'"
            )
            self.transport_mode = 'mqtt'
        self._enable_mqtt = self.transport_mode in ('mqtt', 'both')
        self._enable_api = self.transport_mode in ('api', 'both')
        self.mqtt_broker  = self.get_parameter('mqtt.broker').value
        self.mqtt_port    = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic   = self.get_parameter('mqtt.base_topic').value
        self.qos          = int(self.get_parameter('mqtt.qos').value)
        self.keepalive    = int(self.get_parameter('mqtt.keepalive').value)
        self.use_tls      = bool(self.get_parameter('mqtt.use_tls').value)
        self.tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)

        self._require_gps_fix = bool(self.get_parameter('arming.require_gps_fix').value)
        self._gps_topic = str(self.get_parameter('arming.gps_topic').value).strip() or \
            '/mavros/global_position/global'
        self._gps_fix_min = int(self.get_parameter('arming.gps_fix_min').value)
        self._gps_fix_timeout_sec = max(
            0.1,
            float(self.get_parameter('arming.gps_fix_timeout_sec').value),
        )
        self._require_valid_coords = bool(
            self.get_parameter('arming.require_valid_coords').value
        )

        # MQTT topics
        self.command_topic = f"{self.base_topic}/{self.vehicle_id}/command"
        self.status_topic  = f"{self.base_topic}/{self.vehicle_id}/command/response"

        # Setup MQTT client
        self.client = None
        if self._enable_mqtt:
            self.client = mqtt.Client()
            self.client.on_connect    = self.on_connect
            self.client.on_message    = self.on_message
            self.client.on_disconnect = self.on_disconnect

            if self.mqtt_username:
                self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

            if self.use_tls:
                self.client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
                self.client.tls_insecure_set(self.tls_insecure)

            self.client.reconnect_delay_set(min_delay=1, max_delay=30)

            try:
                self.client.loop_start()
                # connect_async keeps node alive when DNS/network is not ready at boot.
                self.client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=self.keepalive)
                self.get_logger().info(
                    f"MQTT connect scheduled to {self.mqtt_broker}:{self.mqtt_port}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"MQTT startup connect failed (will keep retrying): {e}"
                )
        else:
            self.get_logger().info('MQTT disabled (transport.mode=api)')

        # ROS2 service clients for MAVROS
        self.command_client  = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # MAVROS STATUSTEXT (pre-arm reason, etc.)
        self._statustext_lock = threading.Lock()
        self._last_statustext = ''
        self._last_statustext_ts = 0.0
        self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self._statustext_callback,
            10
        )

        # MAVROS state (armed flag)
        self._state_lock = threading.Lock()
        self._armed_state = None
        self._state_ts = 0.0
        self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            10
        )

        # GPS fix for pre-arm guard
        self._gps_lock = threading.Lock()
        self._gps_fix_status = None
        self._gps_lat = None
        self._gps_lon = None
        self._gps_ts = 0.0
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            NavSatFix,
            self._gps_topic,
            self._gps_callback,
            sensor_qos,
        )

        # ROS2 publisher for command status
        self.status_publisher = self.create_publisher(String, 'command_status', 10)

        # Command log context (shared between MQTT thread → ROS spin thread)
        self._cmd_log_lock = threading.Lock()
        self._cmd_log_ctx: dict = {}

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
        self._api_poll_interval = float(self.get_parameter('api.command_poll_interval_sec').value)
        self._api_poll_limit = int(self.get_parameter('api.command_poll_limit').value)
        self._api_queue = None
        self._api_thread = None
        self._api_running = False
        self._api_poll_thread = None
        self._api_poll_running = False
        self._api_auth_warned = False
        self._last_command_id = None

        if self._enable_api and not self._api_base_url:
            self.get_logger().warn('API enabled but api.base_url is empty, disabling API')
            self._enable_api = False

        if self._enable_api:
            self._start_api_worker()
            self._start_api_polling()

        # CSV Logger
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._command_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'command'),
            'command_log',
            _COMMAND_FIELDS,
        )

        self.get_logger().info(f"Command node aktif — vehicle: {self.vehicle_id}")
        if self._enable_mqtt:
            self.get_logger().info(f"MQTT topic: {self.command_topic}")
        if self._enable_api:
            self.get_logger().info(f"API polling: {self._api_base_url}/commands/pending")

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
            self.handle_command(payload, source='mqtt')
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def handle_command(self, payload, source='mqtt', request_id=None):
        if not isinstance(payload, dict):
            self._finalize_command(None, False, "Invalid command payload")
            return

        command_received_ts = _now_iso()
        command_type = str(payload.get('command', '')).strip().upper()
        req_id = request_id or payload.get('request_id')
        req_id = str(req_id).strip() if req_id is not None else ''
        ctx_id = req_id or f"{command_type}_{command_received_ts}"
        request_id_value = req_id or ctx_id
        self.get_logger().info(f"⚡ Received command: {command_type}")

        with self._cmd_log_lock:
            self._cmd_log_ctx[ctx_id] = {
                'command_received_timestamp': command_received_ts,
                'vehicle_code': self.vehicle_id,
                'command': command_type,
                'request_id': request_id_value,
                'source': source,
                'mavlink_sent_timestamp': '',
            }

        if command_type == 'ARM':
            if self._require_gps_fix and not self._gps_ready():
                self._finalize_command(
                    ctx_id,
                    False,
                    'Pre-arm check failed: GPS no fix',
                )
                return
            self.send_arm_command(True, force=False, ctx_id=ctx_id)
        elif command_type == 'FORCE_ARM':
            self.send_arm_command(True, force=True, ctx_id=ctx_id)
        elif command_type == 'DISARM':
            self.send_arm_command(False, force=False, ctx_id=ctx_id)
        elif command_type == 'FORCE_DISARM':
            self.send_arm_command(False, force=True, ctx_id=ctx_id)
        elif command_type == 'AUTO':
            self.send_mode_command('AUTO', ctx_id=ctx_id)
        elif command_type == 'MANUAL':
            self.send_mode_command('MANUAL', ctx_id=ctx_id)
        elif command_type == 'HOLD':
            self.send_mode_command('HOLD', ctx_id=ctx_id)
        elif command_type == 'LOITER':
            self.send_mode_command('LOITER', ctx_id=ctx_id)
        elif command_type == 'RTL':
            self.send_mode_command('RTL', ctx_id=ctx_id)
        else:
            self.get_logger().warn(f"Unknown command: {command_type}")
            self._finalize_command(ctx_id, False, f"Unknown command: {command_type}")

    def send_arm_command(self, arm, force=False, ctx_id=None):
        cmd_name = ('FORCE_ARM' if force else 'ARM') if arm else ('FORCE_DISARM' if force else 'DISARM')
        if not self.wait_for_mavros_service(self.command_client, '/mavros/cmd/command'):
            self._finalize_command(ctx_id, False, 'MAVROS service unavailable')
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

        self.get_logger().info(f"Sending {cmd_name} command")
        mavlink_sent_ts = _now_iso()
        with self._cmd_log_lock:
            if ctx_id in self._cmd_log_ctx:
                self._cmd_log_ctx[ctx_id]['mavlink_sent_timestamp'] = mavlink_sent_ts

        future = self.command_client.call_async(req)
        future.add_done_callback(lambda f: self.command_response_callback(f, cmd_name, ctx_id))

    def send_mode_command(self, mode, ctx_id=None):
        if not self.wait_for_mavros_service(self.set_mode_client, '/mavros/set_mode'):
            self._finalize_command(ctx_id, False, 'MAVROS set_mode service unavailable')
            return

        req = SetMode.Request()
        req.custom_mode = mode

        self.get_logger().info(f"Sending mode change to {mode}")
        mavlink_sent_ts = _now_iso()
        with self._cmd_log_lock:
            if ctx_id in self._cmd_log_ctx:
                self._cmd_log_ctx[ctx_id]['mavlink_sent_timestamp'] = mavlink_sent_ts

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.mode_response_callback(f, mode, ctx_id))

    def command_response_callback(self, future, command_name, ctx_id):
        try:
            response = future.result()
            success = bool(getattr(response, 'success', False))
            if success:
                if command_name in {'ARM', 'FORCE_ARM', 'DISARM', 'FORCE_DISARM'}:
                    self._confirm_arm_state_async(command_name, ctx_id)
                else:
                    self.get_logger().info(f"{command_name} successful")
                    self._finalize_command(ctx_id, True, f'{command_name} successful')
            else:
                reason = self._recent_statustext()
                result_code = int(getattr(response, 'result', -1))
                code_label = _MAV_RESULT_LABELS.get(result_code, f'code {result_code}')
                if reason:
                    msg = f'{command_name} failed: {reason}'
                else:
                    msg = f'{command_name} failed: {code_label}'
                self.get_logger().error(f"{command_name} failed: result={result_code}")
                self._finalize_command(ctx_id, False, msg)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self._finalize_command(ctx_id, False, f"{command_name} error: {e}")

    def mode_response_callback(self, future, mode_name, ctx_id):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Mode change to {mode_name} successful")
                self._finalize_command(ctx_id, True, f'Mode changed to {mode_name}')
            else:
                self.get_logger().error(f"Mode change to {mode_name} failed")
                self._finalize_command(ctx_id, False, f'Mode change to {mode_name} failed')
        except Exception as e:
            self.get_logger().error(f"Set mode service call failed: {e}")
            self._finalize_command(ctx_id, False, f"Mode change error: {e}")

    def _finalize_command(self, ctx_id, success: bool, message: str):
        response_ts = _now_iso()
        ack_ts = _now_iso_utc()
        ctx = {}
        if ctx_id is not None:
            with self._cmd_log_lock:
                ctx = self._cmd_log_ctx.pop(ctx_id, {})

        self.publish_status(ctx, message, success, ack_ts)
        self._write_command_log_from_ctx(ctx, response_ts, success, message)
        self._send_api_ack(ctx, success, message, ack_ts)

    def _write_command_log_from_ctx(self, ctx: dict, response_ts: str, success: bool, message: str):
        result = 'SUCCESS' if success else 'FAILED'
        row = {
            'command_received_timestamp': ctx.get('command_received_timestamp', ''),
            'vehicle_code': ctx.get('vehicle_code', self.vehicle_id),
            'command': ctx.get('command', ''),
            'mavlink_sent_timestamp': ctx.get('mavlink_sent_timestamp', ''),
            'execution_result': result,
            'execution_message': message,
            'command_response_timestamp': response_ts,
            'duration_ms': _duration_ms(ctx.get('command_received_timestamp', ''), response_ts),
        }
        self._command_csv.write(row)

    def _send_api_ack(self, ctx: dict, success: bool, message: str, response_ts: str):
        if not self._enable_api:
            return
        request_id = ctx.get('request_id')
        command_name = ctx.get('command')
        if not request_id or not command_name:
            return

        status = 'ok' if success else 'error'
        payload = {
            'vehicle_code': ctx.get('vehicle_code', self.vehicle_id),
            'request_id': request_id,
            'command': command_name,
            'status': status,
            'message': message,
            'timestamp': response_ts,
        }
        self._api_enqueue('POST', '/command-acks', payload)

    def publish_status(self, ctx: dict, message: str, success: bool, ack_ts: str):
        data = json.dumps({
            "status": "success" if success else "error",
            "message": message,
            "vehicle_id": self.vehicle_id,
        })
        msg = String()
        msg.data = data
        self.status_publisher.publish(msg)
        if self.client is not None:
            request_id = ctx.get('request_id') if ctx else None
            command_name = ctx.get('command') if ctx else None
            if request_id and command_name:
                ack_payload = {
                    'request_id': request_id,
                    'command': command_name,
                    'status': 'ok' if success else 'error',
                    'message': message,
                    'timestamp': ack_ts,
                }
                self.client.publish(
                    self.status_topic,
                    json.dumps(ack_payload),
                    qos=self.qos,
                )

    def _statustext_callback(self, msg: StatusText) -> None:
        text = getattr(msg, 'text', '')
        if not text:
            return
        with self._statustext_lock:
            self._last_statustext = str(text)
            self._last_statustext_ts = time.monotonic()

    def _state_callback(self, msg: State) -> None:
        with self._state_lock:
            self._armed_state = bool(msg.armed)
            self._state_ts = time.monotonic()

    def _gps_callback(self, msg: NavSatFix) -> None:
        with self._gps_lock:
            self._gps_fix_status = int(msg.status.status)
            self._gps_lat = msg.latitude
            self._gps_lon = msg.longitude
            self._gps_ts = time.monotonic()

    def _recent_statustext(self, max_age_sec: float = 5.0) -> str:
        with self._statustext_lock:
            if not self._last_statustext:
                return ''
            if (time.monotonic() - self._last_statustext_ts) > max_age_sec:
                return ''
            return self._last_statustext

    def _gps_ready(self) -> bool:
        now = time.monotonic()
        with self._gps_lock:
            fix = self._gps_fix_status
            lat = self._gps_lat
            lon = self._gps_lon
            ts = self._gps_ts

        if fix is None:
            return False
        if (now - ts) > self._gps_fix_timeout_sec:
            return False
        if fix < self._gps_fix_min:
            return False
        if self._require_valid_coords and not _coords_valid(lat, lon):
            return False
        return True

    def _confirm_arm_state_async(self, command_name: str, ctx_id: str, timeout_sec: float = 4.0):
        thread = threading.Thread(
            target=self._confirm_arm_state,
            args=(command_name, ctx_id, timeout_sec),
            daemon=True,
        )
        thread.start()

    def _confirm_arm_state(self, command_name: str, ctx_id: str, timeout_sec: float):
        desired = command_name in {'ARM', 'FORCE_ARM'}
        start = time.monotonic()
        while (time.monotonic() - start) < timeout_sec:
            with self._state_lock:
                armed = self._armed_state
            if armed is not None and armed == desired:
                self.get_logger().info(f"{command_name} successful")
                self._finalize_command(ctx_id, True, f'{command_name} successful')
                return
            time.sleep(0.2)

        reason = self._recent_statustext()
        if reason:
            msg = f'{command_name} failed: {reason}'
        else:
            msg = f'{command_name} failed: armed state not reached'
        self._finalize_command(ctx_id, False, msg)

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
                query = f"/commands/pending?vehicle_code={self.vehicle_id}&limit={limit}"
                payload = self._api_get_json(query)
                if isinstance(payload, dict):
                    data = payload.get('data')
                    if isinstance(data, list) and data:
                        cmd = data[0]
                        request_id = cmd.get('request_id') or cmd.get('id')
                        if request_id and request_id == self._last_command_id:
                            time.sleep(interval)
                            continue
                        self._last_command_id = request_id
                        self.handle_command(cmd, source='api', request_id=request_id)
            except Exception as exc:
                self.get_logger().warn(f"API command polling error: {exc}")
            time.sleep(interval)

    def wait_for_mavros_service(self, client, service_name: str, retries: int = 5, timeout_sec: float = 1.0) -> bool:
        for attempt in range(1, retries + 1):
            if client.wait_for_service(timeout_sec=timeout_sec):
                return True
            self.get_logger().warn(f"Waiting for {service_name} ({attempt}/{retries})")
        self.get_logger().error(f"{service_name} not available after {retries} retries")
        return False

    def destroy_node(self):
        self._command_csv.close()
        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()

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
