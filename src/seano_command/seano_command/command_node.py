import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandLong, SetMode
import paho.mqtt.client as mqtt
import ssl
import json
import csv
import os
import threading
from datetime import datetime
from zoneinfo import ZoneInfo

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

        # ROS2 service clients for MAVROS
        self.command_client  = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ROS2 publisher for command status
        self.status_publisher = self.create_publisher(String, 'command_status', 10)

        # Command log context (shared between MQTT thread → ROS spin thread)
        self._cmd_log_lock = threading.Lock()
        self._cmd_log_ctx: dict = {}

        # CSV Logger
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._command_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'command'),
            'command_log',
            _COMMAND_FIELDS,
        )

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

        command_received_ts = _now_iso()
        command_type = str(payload.get('command', '')).strip().upper()
        self.get_logger().info(f"⚡ Received command: {command_type}")

        with self._cmd_log_lock:
            self._cmd_log_ctx[command_type] = {
                'command_received_timestamp': command_received_ts,
                'vehicle_code': self.vehicle_id,
                'command': command_type,
                'mavlink_sent_timestamp': '',
            }

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
            self._write_command_log(command_type, '', 'FAILED', f'Unknown command: {command_type}')

    def send_arm_command(self, arm, force=False):
        cmd_name = ('FORCE_ARM' if force else 'ARM') if arm else ('FORCE_DISARM' if force else 'DISARM')
        if not self.wait_for_mavros_service(self.command_client, '/mavros/cmd/command'):
            self.publish_status("MAVROS service unavailable", False)
            self._write_command_log(cmd_name, '', 'FAILED', 'MAVROS service unavailable')
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
            if cmd_name in self._cmd_log_ctx:
                self._cmd_log_ctx[cmd_name]['mavlink_sent_timestamp'] = mavlink_sent_ts

        future = self.command_client.call_async(req)
        future.add_done_callback(lambda f: self.command_response_callback(f, cmd_name))

    def send_mode_command(self, mode):
        if not self.wait_for_mavros_service(self.set_mode_client, '/mavros/set_mode'):
            self.publish_status("Set mode service unavailable", False)
            self._write_command_log(mode, '', 'FAILED', 'MAVROS set_mode service unavailable')
            return

        req = SetMode.Request()
        req.custom_mode = mode

        self.get_logger().info(f"Sending mode change to {mode}")
        mavlink_sent_ts = _now_iso()
        with self._cmd_log_lock:
            if mode in self._cmd_log_ctx:
                self._cmd_log_ctx[mode]['mavlink_sent_timestamp'] = mavlink_sent_ts

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.mode_response_callback(f, mode))

    def command_response_callback(self, future, command_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{command_name} successful")
                self.publish_status(f"{command_name} successful", True)
                self._write_command_log(command_name, _now_iso(), 'SUCCESS', f'{command_name} successful')
            else:
                msg = f'{command_name} failed: code {response.result}'
                self.get_logger().error(f"{command_name} failed: result={response.result}")
                self.publish_status(f"{command_name} failed: code {response.result}", False)
                self._write_command_log(command_name, _now_iso(), 'FAILED', msg)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.publish_status(f"{command_name} error: {e}", False)
            self._write_command_log(command_name, _now_iso(), 'FAILED', str(e))

    def mode_response_callback(self, future, mode_name):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Mode change to {mode_name} successful")
                self.publish_status(f"Mode changed to {mode_name}", True)
                self._write_command_log(mode_name, _now_iso(), 'SUCCESS', f'Mode changed to {mode_name}')
            else:
                self.get_logger().error(f"Mode change to {mode_name} failed")
                self.publish_status(f"Mode change to {mode_name} failed", False)
                self._write_command_log(mode_name, _now_iso(), 'FAILED', f'Mode change to {mode_name} failed')
        except Exception as e:
            self.get_logger().error(f"Set mode service call failed: {e}")
            self.publish_status(f"Mode change error: {e}", False)
            self._write_command_log(mode_name, _now_iso(), 'FAILED', str(e))

    def _write_command_log(self, command_name: str, response_ts: str, result: str, message: str):
        if not response_ts:
            response_ts = _now_iso()
        with self._cmd_log_lock:
            ctx = self._cmd_log_ctx.pop(command_name, {})
        row = {
            'command_received_timestamp': ctx.get('command_received_timestamp', ''),
            'vehicle_code': ctx.get('vehicle_code', self.vehicle_id),
            'command': ctx.get('command', command_name),
            'mavlink_sent_timestamp': ctx.get('mavlink_sent_timestamp', ''),
            'execution_result': result,
            'execution_message': message,
            'command_response_timestamp': response_ts,
            'duration_ms': _duration_ms(ctx.get('command_received_timestamp', ''), response_ts),
        }
        self._command_csv.write(row)

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
        self._command_csv.close()
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
