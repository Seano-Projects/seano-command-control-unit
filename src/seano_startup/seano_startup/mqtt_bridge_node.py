import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import paho.mqtt.client as mqtt
import ssl
import json
import csv
import os
from datetime import datetime
from zoneinfo import ZoneInfo

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
        self.declare_parameter('mqtt.broker', 'localhost')
        self.declare_parameter('mqtt.port', 1883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.raw_topic_suffix', 'raw')

        self.vehicle_id = self.get_parameter('vehicle.id').value
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

        self.client = mqtt.Client()
        self._mqtt_connected = False
        self._last_connect_log = ''

        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect

        if self.mqtt_username:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.client.tls_insecure_set(True)

        self.client.loop_start()
        self._connect_mqtt(initial=True)

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
            self.raw_string_callback,
            10
        )
        self.create_subscription(
            String,
            'command_status',
            self.raw_string_callback,
            10
        )
        self.create_subscription(
            String,
            'waypoint_status',
            self.raw_string_callback,
            10
        )
        self.create_subscription(
            String,
            'communication/status',
            self.raw_string_callback,
            10
        )
        self.create_subscription(
            String,
            'anti_theft/alert',
            self.raw_string_callback,
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
        self.create_timer(10.0, self._reconnect_if_needed)

    def _connect_mqtt(self, initial=False):
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
        try:
            row = json.loads(msg.data)
            row['mqtt_publish_timestamp'] = mqtt_publish_ts
            self._telemetry_csv.write(row)
        except Exception:
            pass

    def failsafe_callback(self, msg):
        if self._mqtt_connected:
            self.client.publish(
                self.failsafe_mqtt_topic,
                msg.data,
                qos=1,
                retain=False
            )
            self._publish_raw(msg.data)

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

    def raw_string_callback(self, msg: String):
        self._publish_raw(msg.data)

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
        self._publish_raw(json.dumps(payload, separators=(',', ':')))

    def destroy_node(self):
        self._telemetry_csv.close()
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
