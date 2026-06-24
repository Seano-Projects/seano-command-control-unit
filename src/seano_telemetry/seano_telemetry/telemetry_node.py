import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, VfrHud
from sensor_msgs.msg import NavSatFix, Imu
import paho.mqtt.client as mqtt
import ssl
import json
import glob
import os
import time
from datetime import datetime
from zoneinfo import ZoneInfo


_ID_TZ_WIB = ZoneInfo('Asia/Jakarta')
_ID_TZ_WITA = ZoneInfo('Asia/Makassar')
_ID_TZ_WIT = ZoneInfo('Asia/Jayapura')
_DEFAULT_TZ = _ID_TZ_WIB


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


def _resolve_timezone(lat, lon, gps_ok: bool):
    if not gps_ok or not _coords_valid(lat, lon):
        return _DEFAULT_TZ
    if lon < 112.5:
        return _ID_TZ_WIB
    if lon < 127.5:
        return _ID_TZ_WITA
    return _ID_TZ_WIT


def _now_iso_local(lat, lon, gps_ok: bool) -> str:
    tz = _resolve_timezone(lat, lon, gps_ok)
    return datetime.now(tz).isoformat(timespec='milliseconds')


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Declare parameters
        self.declare_parameter('system.mode', 'unknown')
        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.battery_topic', '')
        self.declare_parameter('mqtt.alert_topic', '')
        self.declare_parameter('mqtt.alert_qos', 0)
        self.declare_parameter('telemetry.gps_alert_interval_sec', 30.0)
        self.declare_parameter('telemetry.gps_alert_max_duration_sec', 3600.0)
        self.declare_parameter('telemetry.gps_fix_min', 1)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('communication.ethernet_interface', '')
        self.declare_parameter('telemetry.altitude_reference', 'relative')
        self.declare_parameter('telemetry.relative_alt_max_abs_m', 200.0)

        self.system_mode = self.get_parameter('system.mode').value
        self.vehicle_id = str(self.get_parameter('vehicle.id').value)
        self.mqtt_base_topic = str(self.get_parameter('mqtt.base_topic').value).strip() or 'seano'
        self.mqtt_battery_topic = self.get_parameter('mqtt.battery_topic').value
        self.mqtt_alert_topic = str(self.get_parameter('mqtt.alert_topic').value).strip()
        self.mqtt_alert_qos = int(self.get_parameter('mqtt.alert_qos').value)
        self.gps_alert_interval_sec = max(
            1.0,
            float(self.get_parameter('telemetry.gps_alert_interval_sec').value)
        )
        self.gps_alert_max_duration_sec = max(
            0.0,
            float(self.get_parameter('telemetry.gps_alert_max_duration_sec').value)
        )
        self.gps_fix_min = max(
            0,
            int(self.get_parameter('telemetry.gps_fix_min').value)
        )
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.mqtt_battery_topic = self.get_parameter('mqtt.battery_topic').value
        self.mqtt_battery_topic_alt = None
        self.net_iface = str(self.get_parameter('communication.ethernet_interface').value).strip()
        self.altitude_reference = str(
            self.get_parameter('telemetry.altitude_reference').value
        ).strip().lower()
        self.relative_alt_max_abs_m = max(
            10.0,
            float(self.get_parameter('telemetry.relative_alt_max_abs_m').value)
        )
        if self.altitude_reference not in ('relative', 'msl', 'baro'):
            self.get_logger().warn(
                f"Unknown telemetry.altitude_reference='{self.altitude_reference}', fallback to 'relative'"
            )
            self.altitude_reference = 'relative'
        if not self.mqtt_alert_topic:
            self.mqtt_alert_topic = f"{self.mqtt_base_topic}/{self.vehicle_id}/alert"

        self._gps_alert_last_ts = 0.0
        self._gps_alert_start_ts = 0.0
        self._gps_ready_prev = False
        if not self.mqtt_battery_topic:
            self.mqtt_battery_topic = f"seano/{self.vehicle_id}/battery"
            self.mqtt_battery_topic_alt = f"seano/{self.vehicle_id}/Battery"
        elif self.mqtt_battery_topic.endswith('/battery'):
            self.mqtt_battery_topic_alt = self.mqtt_battery_topic[:-8] + '/Battery'
        elif self.mqtt_battery_topic.endswith('/Battery'):
            self.mqtt_battery_topic_alt = self.mqtt_battery_topic[:-8] + '/battery'

        # Data from MAVROS
        self.armed = False
        self.mode = "UNKNOWN"
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_msl = None
        self.altitude_rel = None
        self.altitude_baro = None
        self._altitude_msl_home = None
        self._altitude_baro_home = None
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Additional telemetry data
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_percentage = 0
        self.speed = 0.0
        self.gps_fix_status = None
        self.gps_ok_raw = False
        self.gps_ok = False
        self.system_status = "UNKNOWN"
        self.net_rx_mbps = 0.0
        self.net_tx_mbps = 0.0
        self._net_prev = None
        self._net_prev_time = None
        self._net_warned = False

        # QoS Profile for MAVROS topics (BEST_EFFORT to match MAVROS)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to MAVROS topics
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            sensor_qos
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos
        )

        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Subscribe to VFR HUD for speed
        self.vfr_sub = self.create_subscription(
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_callback,
            sensor_qos
        )


        # Publisher for telemetry (JSON format)
        self.publisher_ = self.create_publisher(
            String,
            'telemetry',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_telemetry)

        self.mqtt_client = mqtt.Client()
        self._mqtt_connected = False
        if self.mqtt_username:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        self.mqtt_client.on_connect = self._mqtt_on_connect
        self.mqtt_client.on_disconnect = self._mqtt_on_disconnect
        self.mqtt_client.on_message = self._mqtt_on_message
        self.mqtt_client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=60)
        self.mqtt_client.loop_start()

        self.get_logger().info(f'Telemetry node started')
        self.get_logger().info(f'Vehicle ID   : {self.vehicle_id}')
        self.get_logger().info(f'System Mode : {self.system_mode}')
        self.get_logger().info(f'Battery MQTT Topic : {self.mqtt_battery_topic}')
        if self.net_iface:
            self.get_logger().info(f'Network Interface : {self.net_iface}')
        self.get_logger().info(f'Altitude Reference : {self.altitude_reference}')
        if self.altitude_reference == 'relative':
            self.get_logger().info(
                f'Relative Altitude Max Abs : {self.relative_alt_max_abs_m} m'
            )

    def state_callback(self, msg):
        """Callback for MAVROS state (armed, mode)"""
        prev_armed = self.armed
        prev_mode = self.mode
        self.armed = msg.armed
        self.mode = msg.mode
        
        # Determine system status based on connection and mode
        if msg.connected:
            self.system_status = "OK"
        else:
            self.system_status = "DISCONNECTED"

        # Immediately publish telemetry when armed/mode changes — no waiting for timer
        if self.armed != prev_armed or self.mode != prev_mode:
            self.publish_telemetry()

    def gps_callback(self, msg):
        """Callback for GPS position"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude_msl = msg.altitude
        
        # Check GPS fix status
        # NavSatFix status: -1=no fix, 0=fix, 1=SBAS fix, 2=GBAS fix
        status = int(msg.status.status)
        self.gps_fix_status = status
        self.gps_ok_raw = status >= 0
        self.gps_ok = status >= self.gps_fix_min

    def rel_alt_callback(self, msg):
        """Callback for relative altitude (home-referenced)."""
        self.altitude_rel = msg.data

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = (rc == 0)
        if rc == 0:
            client.subscribe(self.mqtt_battery_topic, qos=1)
            self.get_logger().info(f'Subscribed battery MQTT: {self.mqtt_battery_topic}')
            if self.mqtt_battery_topic_alt and self.mqtt_battery_topic_alt != self.mqtt_battery_topic:
                client.subscribe(self.mqtt_battery_topic_alt, qos=1)
                self.get_logger().info(
                    f'Subscribed battery MQTT (alt): {self.mqtt_battery_topic_alt}'
                )
        else:
            self.get_logger().warn(f'Battery MQTT connect failed rc={rc}')

    def _mqtt_on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False

    def _as_float(self, data, keys, default):
        for key in keys:
            if key in data and data[key] is not None:
                try:
                    return float(data[key])
                except Exception:
                    continue
        return default

    def _mqtt_on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8', errors='ignore').strip()
        if not payload:
            return

        try:
            data = json.loads(payload)
        except Exception:
            return

        self.battery_voltage = self._as_float(
            data,
            ['battery_voltage', 'voltage', 'volt', 'v'],
            self.battery_voltage,
        )

        self.battery_current = self._as_float(
            data,
            ['battery_current', 'current', 'amp', 'current_a'],
            0.0,
        )

        pct = self._as_float(
            data,
            ['battery_percentage', 'percentage', 'soc', 'capacity'],
            self.battery_percentage,
        )
        if pct <= 1.0:
            pct = pct * 100.0
        self.battery_percentage = round(max(0.0, min(100.0, pct)), 1)

    def _maybe_publish_gps_alert(self):
        gps_ready = self.gps_ok and _coords_valid(self.latitude, self.longitude)
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
            'message': 'GPS no fix',
            'severity': 'warning',
            'alert_type': 'GPS',
        }
        if _coords_valid(self.latitude, self.longitude):
            payload['latitude'] = round(self.latitude, 6)
            payload['longitude'] = round(self.longitude, 6)

        if self.mqtt_client is not None:
            self.mqtt_client.publish(
                self.mqtt_alert_topic,
                json.dumps(payload),
                qos=self.mqtt_alert_qos
            )
            if self._mqtt_connected:
                self.get_logger().info(
                    f'GPS alert sent to MQTT: {self.mqtt_alert_topic}'
                )
            else:
                self.get_logger().warn(
                    f'GPS alert queued (MQTT not connected): {self.mqtt_alert_topic}'
                )
        self._gps_alert_last_ts = now

    def vfr_callback(self, msg):
        """Callback for VFR HUD (speed)"""
        self.speed = msg.groundspeed
        self.altitude_baro = msg.altitude

    def _read_jetson_temperature(self):
        """Read CPU temperature from Jetson thermal zone"""
        try:
            paths = glob.glob('/sys/class/thermal/thermal_zone*/temp')
            temps = []
            for path in paths:
                with open(path, 'r') as f:
                    temps.append(int(f.read().strip()))
            if temps:
                return max(temps) / 1000.0  # milicelsius -> celsius
        except Exception:
            pass
        return 0.0

    def _read_net_bytes(self):
        if not self.net_iface:
            return None

        base = f"/sys/class/net/{self.net_iface}/statistics"
        rx_path = os.path.join(base, 'rx_bytes')
        tx_path = os.path.join(base, 'tx_bytes')

        try:
            with open(rx_path, 'r') as f:
                rx_bytes = int(f.read().strip())
            with open(tx_path, 'r') as f:
                tx_bytes = int(f.read().strip())
            return rx_bytes, tx_bytes
        except Exception as exc:
            if not self._net_warned:
                self.get_logger().warn(
                    f"Network interface '{self.net_iface}' not readable: {exc}"
                )
                self._net_warned = True
        return None

    def _update_network_rate(self):
        sample = self._read_net_bytes()
        now = time.monotonic()

        if sample is None:
            self.net_rx_mbps = 0.0
            self.net_tx_mbps = 0.0
            return

        if self._net_prev is None or self._net_prev_time is None:
            self._net_prev = sample
            self._net_prev_time = now
            return

        dt = now - self._net_prev_time
        if dt <= 0.0:
            return

        rx_delta = sample[0] - self._net_prev[0]
        tx_delta = sample[1] - self._net_prev[1]

        if rx_delta < 0 or tx_delta < 0:
            self._net_prev = sample
            self._net_prev_time = now
            return

        self.net_rx_mbps = (rx_delta * 8.0) / (1_000_000.0 * dt)
        self.net_tx_mbps = (tx_delta * 8.0) / (1_000_000.0 * dt)
        self._net_prev = sample
        self._net_prev_time = now

    def imu_callback(self, msg):
        """Callback for IMU data"""
        # Convert quaternion to euler angles (simplified)
        q = msg.orientation
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = self._rad_to_deg(self._atan2(sinr_cosp, cosr_cosp))
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.pitch = self._rad_to_deg(self._copysign(3.14159 / 2, sinp))
        else:
            self.pitch = self._rad_to_deg(self._asin(sinp))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = self._rad_to_deg(self._atan2(siny_cosp, cosy_cosp))
        
        # Normalize yaw to 0-360
        if self.yaw < 0:
            self.yaw += 360.0
        
        self.heading = self.yaw

    def _rad_to_deg(self, rad):
        return rad * 180.0 / 3.14159

    def _atan2(self, y, x):
        import math
        return math.atan2(y, x)

    def _asin(self, x):
        import math
        return math.asin(x)

    def _copysign(self, x, y):
        import math
        return math.copysign(x, y)

    def publish_telemetry(self):
        """Publish telemetry data in JSON format"""
        self._update_network_rate()
        self._maybe_publish_gps_alert()
        if self.altitude_reference == 'relative':
            altitude = self.altitude_rel
            if altitude is not None and abs(float(altitude)) > self.relative_alt_max_abs_m:
                altitude = None
            if altitude is None:
                if self.altitude_baro is not None:
                    if self._altitude_baro_home is None:
                        self._altitude_baro_home = self.altitude_baro
                    altitude = self.altitude_baro - self._altitude_baro_home
            if altitude is None:
                if self.altitude_msl is not None:
                    if self._altitude_msl_home is None:
                        self._altitude_msl_home = self.altitude_msl
                    altitude = self.altitude_msl - self._altitude_msl_home
        elif self.altitude_reference == 'msl':
            altitude = self.altitude_msl
            if altitude is None:
                altitude = self.altitude_baro
            if altitude is None:
                altitude = self.altitude_rel
        else:  # baro
            altitude = self.altitude_baro
            if altitude is None:
                altitude = self.altitude_rel
            if altitude is None:
                altitude = self.altitude_msl
        if altitude is None:
            altitude = 0.0
        altitude = max(0.0, float(altitude))

        telemetry_data = {
            "vehicle_code": self.vehicle_id,
            "date_time": _now_iso_local(self.latitude, self.longitude, self.gps_ok_raw),
            "battery_voltage": round(self.battery_voltage, 1),
            "battery_current": round(self.battery_current, 1),
            "battery_percentage": self.battery_percentage,
            "latitude": round(self.latitude, 6),
            "longitude": round(self.longitude, 6),
            "altitude": round(altitude, 1),
            "heading": round(self.heading, 1),
            "armed": self.armed,
            "gps_ok": self.gps_ok,
            "system_status": self.system_status,
            "mode": self.mode,
            "speed": round(self.speed, 1),
            "roll": round(self.roll, 1),
            "pitch": round(self.pitch, 1),
            "yaw": round(self.yaw, 1),
            "temperature_system": f"{round(self._read_jetson_temperature(), 1)}",
            "network_iface": self.net_iface,
            "download_mbps": round(self.net_rx_mbps, 2),
            "upload_mbps": round(self.net_tx_mbps, 2)
        }
        
        msg = String()
        msg.data = json.dumps(telemetry_data)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
