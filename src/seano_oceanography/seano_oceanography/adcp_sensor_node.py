import csv
import json
import math
import os
import ssl
from datetime import datetime
from zoneinfo import ZoneInfo

import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.msg import VfrHud
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Beam geometry constant — standard Teledyne RDI Janus 20° beam angle
# ---------------------------------------------------------------------------
_ID_TZ_WIB = ZoneInfo('Asia/Jakarta')
_ID_TZ_WITA = ZoneInfo('Asia/Makassar')
_ID_TZ_WIT = ZoneInfo('Asia/Jayapura')
_DEFAULT_TZ = _ID_TZ_WIB

_BEAM_ANGLE_RAD = math.radians(20.0)
_BEAM_SIN = math.sin(_BEAM_ANGLE_RAD)


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


def _resolve_timezone(lat, lon, gps_ok: bool, fallback_tz):
    if not gps_ok or not _coords_valid(lat, lon):
        return fallback_tz
    lon_f = float(lon)
    if lon_f < 112.5:
        return _ID_TZ_WIB
    if lon_f < 127.5:
        return _ID_TZ_WITA
    return _ID_TZ_WIT


def _safe_zoneinfo(name: str, logger=None):
    try:
        return ZoneInfo(name)
    except Exception:
        if logger is not None:
            logger.warn(f'Invalid timezone: {name}, fallback to Asia/Jakarta')
        return _DEFAULT_TZ


class _DailyCsvWriter:
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


_ADCP_FIELDS = [
    'date_time', 'vehicle_code', 'sensor_code', 'sensor',
    'latitude', 'longitude', 'altitude', 'heading_deg', 'gps_ok',
    'ensemble_no', 'temperature_c',
    'v1_ms', 'v2_ms', 'v3_ms', 'v4_ms',
    'current_speed_ms', 'current_direction_deg', 'water_depth_m',
    'mqtt_publish_timestamp',
]


class ADCPSensorNode(Node):
    def __init__(self):
        super().__init__('adcp_sensor_node')

        self.declare_parameter('oceanography.adcp.publish_topic', 'oceanography/adcp')
        self.declare_parameter('oceanography.adcp.publish_rate_hz', 1.0)
        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('transport.mode', 'mqtt')
        self.declare_parameter('oceanography.adcp.sensor_code', 'ADCP-WORKHORSE')
        self.declare_parameter('oceanography.adcp.timezone', 'Asia/Jakarta')
        self.declare_parameter('oceanography.adcp.gps_topic', '/mavros/global_position/global')
        self.declare_parameter('oceanography.adcp.vfr_topic', '/mavros/vfr_hud')
        self.declare_parameter('oceanography.adcp.default_latitude', 0.0)
        self.declare_parameter('oceanography.adcp.default_longitude', 0.0)
        self.declare_parameter('oceanography.adcp.default_altitude', 0.0)
        self.declare_parameter('oceanography.adcp.max_water_depth_m', 250.0)
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        self.declare_parameter('validation.force_gps_fix_for_publish', True)
        self.declare_parameter('validation.allow_fallback_coords_without_gps', False)
        self.declare_parameter('validation.fallback_latitude', 0.0)
        self.declare_parameter('validation.fallback_longitude', 0.0)
        self.declare_parameter('validation.fallback_motion_enabled', True)
        self.declare_parameter('validation.fallback_motion_radius_m', 25.0)
        self.declare_parameter('validation.fallback_motion_period_sec', 180.0)
        self.declare_parameter('validation.fallback_motion_north_scale', 0.7)

        topic = self.get_parameter('oceanography.adcp.publish_topic').value
        rate_hz = float(self.get_parameter('oceanography.adcp.publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)

        self.vehicle_code = self.get_parameter('vehicle.id').value
        self.sensor_code = self.get_parameter('oceanography.adcp.sensor_code').value

        transport_mode = str(self.get_parameter('transport.mode').value).strip().lower()
        if transport_mode not in ('mqtt', 'api', 'both'):
            transport_mode = 'mqtt'
        self._enable_mqtt = transport_mode in ('mqtt', 'both')

        self.timezone_name = self.get_parameter('oceanography.adcp.timezone').value
        self.gps_topic = self.get_parameter('oceanography.adcp.gps_topic').value
        self.vfr_topic = self.get_parameter('oceanography.adcp.vfr_topic').value

        self.latitude = float(self.get_parameter('oceanography.adcp.default_latitude').value)
        self.longitude = float(self.get_parameter('oceanography.adcp.default_longitude').value)
        self.altitude = float(self.get_parameter('oceanography.adcp.default_altitude').value)
        self.gps_ok = False
        self.heading_deg = 0.0

        self.max_water_depth_m = max(
            float(self.get_parameter('oceanography.adcp.max_water_depth_m').value), 5.0
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
        self._fallback_motion_t0 = 0.0

        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.qos = int(self.get_parameter('mqtt.qos').value)
        self.mqtt_topic = f'{self.base_topic}/{self.vehicle_code}/{self.sensor_code}/data'

        self.ensemble_no = 0
        self._elapsed = 0.0
        self._period = period

        self.fallback_tz = _safe_zoneinfo(self.timezone_name, self.get_logger())

        # CSV Logger
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._adcp_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'oceanography'),
            'adcp_log',
            _ADCP_FIELDS,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, sensor_qos,
        )
        self.vfr_sub = self.create_subscription(
            VfrHud, self.vfr_topic, self.vfr_callback, sensor_qos,
        )
        self.timer = self.create_timer(period, self.publish_measurement)
        self._mqtt_connected = False
        self.mqtt_client = self._create_mqtt_client()

        self.get_logger().info(f'ADCP sensor node started on topic: {topic}')
        self.get_logger().info(f'ADCP MQTT topic: {self.mqtt_topic}')
        self.get_logger().info(f'ADCP GPS source: {self.gps_topic}')
        self.get_logger().info(f'ADCP heading source: {self.vfr_topic}')
        self.get_logger().info(
            f'ADCP force_gps_fix_for_publish={self.force_gps_fix_for_publish}'
        )
        self.get_logger().info(
            f'ADCP allow_fallback_coords_without_gps={self.allow_fallback_coords_without_gps}'
        )

    def _offset_m_to_coords(self, center_lat, center_lon, east_m, north_m):
        lat_scale = 111320.0
        lon_scale = 111320.0 * max(0.1, math.cos(math.radians(float(center_lat))))
        lat = float(center_lat) + (north_m / lat_scale)
        lon = float(center_lon) + (east_m / lon_scale)
        return lat, lon

    def _next_fallback_coords(self):
        if not _coords_valid(self.fallback_latitude, self.fallback_longitude):
            return None
        if self._fallback_motion_t0 <= 0.0:
            self._fallback_motion_t0 = self._elapsed
        if not self.fallback_motion_enabled or self.fallback_motion_radius_m <= 0.0:
            return round(self.fallback_latitude, 7), round(self.fallback_longitude, 7)

        t = max(0.0, self._elapsed - self._fallback_motion_t0)
        omega = (2.0 * math.pi) / self.fallback_motion_period_sec
        east_m = self.fallback_motion_radius_m * math.sin(omega * t)
        north_m = (
            self.fallback_motion_radius_m
            * self.fallback_motion_north_scale
            * math.sin((omega * 0.63 * t) + 0.9)
        )
        lat, lon = self._offset_m_to_coords(self.fallback_latitude, self.fallback_longitude, east_m, north_m)
        return round(lat, 7), round(lon, 7)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def gps_callback(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        self.gps_ok = msg.status.status >= 0

    def vfr_callback(self, msg: VfrHud):
        self.heading_deg = float(msg.heading)

    # ------------------------------------------------------------------
    # MQTT
    # ------------------------------------------------------------------

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = (rc == 0)
        if rc == 0:
            self.get_logger().info(
                f'ADCP connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}'
            )
        else:
            self.get_logger().warn(f'ADCP MQTT connect failed rc={rc}')

    def _mqtt_on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        self.get_logger().warn(f'ADCP MQTT disconnected rc={rc}, will auto-reconnect')

    def _create_mqtt_client(self):
        if not self._enable_mqtt:
            self.get_logger().info('ADCP MQTT disabled (transport.mode=api)')
            return None
        client = mqtt.Client()
        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)
        client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        client.on_connect = self._mqtt_on_connect
        client.on_disconnect = self._mqtt_on_disconnect
        client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=60)
        client.loop_start()
        return client

    # ------------------------------------------------------------------
    # Measurement simulation
    # ------------------------------------------------------------------

    def _simulate(self, t: float):
        """Return simulated ADCP measurement dict for time t (seconds)."""

        # Water temperature — surface layer with slow drift
        temperature_c = (
            28.5
            + 0.8 * math.sin(t * 0.04 + 0.3)
            + 0.4 * math.sin(t * 0.11 + 1.1)
        )

        # Water depth — slow tide-like variation
        depth_base = self.max_water_depth_m * 0.35
        water_depth_m = max(
            2.0,
            depth_base
            + 15.0 * math.sin(t * 0.006 + 0.5)
            + 5.0 * math.sin(t * 0.018 + 1.3),
        )

        # True current velocity components in Earth frame (m/s)
        u_east = (
            0.15 * math.sin(t * 0.07 + 0.2)
            + 0.08 * math.sin(t * 0.21 + 1.0)
        )
        v_north = (
            0.12 * math.cos(t * 0.07 + 0.4)
            + 0.06 * math.cos(t * 0.19 + 0.8)
        )

        current_speed_ms = math.sqrt(u_east ** 2 + v_north ** 2)
        # Compass convention: 0° = North, clockwise
        current_direction_deg = (math.degrees(math.atan2(u_east, v_north)) + 360.0) % 360.0

        # Rotate Earth → instrument frame using heading
        hdg_rad = math.radians(self.heading_deg)
        u_inst = u_east * math.cos(hdg_rad) + v_north * math.sin(hdg_rad)
        v_inst = -u_east * math.sin(hdg_rad) + v_north * math.cos(hdg_rad)

        # Beam velocities from instrument velocities (Janus 4-beam 20°)
        #   Beam 1 & 2 resolve u_inst; Beam 3 & 4 resolve v_inst
        v1 = -u_inst * _BEAM_SIN + 0.005 * math.sin(t * 1.3)
        v2 = u_inst * _BEAM_SIN + 0.004 * math.cos(t * 1.7)
        v3 = -v_inst * _BEAM_SIN + 0.004 * math.sin(t * 1.5 + 0.9)
        v4 = v_inst * _BEAM_SIN + 0.003 * math.cos(t * 1.9 + 1.2)

        return {
            'temperature_c': round(temperature_c, 2),
            'water_depth_m': round(water_depth_m, 2),
            'v1_ms': round(v1, 3),
            'v2_ms': round(v2, 3),
            'v3_ms': round(v3, 3),
            'v4_ms': round(v4, 3),
            'current_speed_ms': round(current_speed_ms, 3),
            'current_direction_deg': round(current_direction_deg, 1),
        }

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def publish_measurement(self):
        self._elapsed += self._period
        self.ensemble_no += 1

        publish_lat = self.latitude
        publish_lon = self.longitude
        publish_alt = self.altitude
        publish_gps_ok = self.gps_ok

        has_fix = self.gps_ok and _coords_valid(self.latitude, self.longitude)
        if self.force_gps_fix_for_publish and not has_fix:
            if self.allow_fallback_coords_without_gps:
                fb_coords = self._next_fallback_coords()
                if fb_coords is not None:
                    publish_lat, publish_lon = fb_coords
                    publish_gps_ok = False
                else:
                    self.get_logger().debug('ADCP: fallback coords invalid, skipping publish')
                    return
            else:
                self.get_logger().debug('ADCP: GPS not ready, skipping publish')
                return

        sim = self._simulate(self._elapsed)
        tz = _resolve_timezone(publish_lat, publish_lon, publish_gps_ok, self.fallback_tz)
        now_local = datetime.now(tz).isoformat()

        payload = {
            'date_time': now_local,
            'vehicle_code': self.vehicle_code,
            'sensor_code': self.sensor_code,
            'sensor': 'ADCP',
            'latitude': round(publish_lat, 7),
            'longitude': round(publish_lon, 7),
            'altitude': round(publish_alt, 2),
            'heading_deg': round(self.heading_deg, 1),
            'gps_ok': publish_gps_ok,
            'ensemble_no': self.ensemble_no,
            **sim,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)

        if self.mqtt_client is not None and self._mqtt_connected:
            mqtt_publish_ts = datetime.now(tz).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]
            self.mqtt_client.publish(self.mqtt_topic, msg.data, qos=self.qos)
            row = dict(payload)
            row['mqtt_publish_timestamp'] = mqtt_publish_ts
            self._adcp_csv.write(row)

    def destroy_node(self):
        self._adcp_csv.close()
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = ADCPSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
