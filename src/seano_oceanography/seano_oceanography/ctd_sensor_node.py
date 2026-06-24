import csv
import json
import math
import os
import random
import ssl
from datetime import datetime
from zoneinfo import ZoneInfo

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


_ID_TZ_WIB  = ZoneInfo('Asia/Jakarta')
_ID_TZ_WITA = ZoneInfo('Asia/Makassar')
_ID_TZ_WIT  = ZoneInfo('Asia/Jayapura')
_DEFAULT_TZ  = _ID_TZ_WIB


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


# ─── CSV Logger ────────────────────────────────────────────────────────────────

class _DailyCsvWriter:
    def __init__(self, log_dir: str, prefix: str, fieldnames: list):
        self._log_dir   = log_dir
        self._prefix    = prefix
        self._fieldnames = fieldnames
        self._date   = None
        self._fh     = None
        self._writer = None
        os.makedirs(log_dir, exist_ok=True)

    def _rotate(self):
        today = datetime.now().strftime('%Y%m%d')
        if today == self._date:
            return
        if self._fh:
            self._fh.close()
        self._date = today
        path  = os.path.join(self._log_dir, f'{self._prefix}_{today}.csv')
        first = not os.path.exists(path)
        self._fh     = open(path, 'a', newline='', encoding='utf-8')
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


_CTD_FIELDS = [
    'timestamp', 'vehicle_code', 'sensor_code',
    'latitude', 'longitude', 'altitude', 'gps_ok',
    'depth_m', 'pressure_m', 'temperature_c', 'conductivity_ms_cm',
    'salinity_psu', 'density_kg_m3', 'sound_velocity_ms',
    'mqtt_publish_timestamp',
]

_CTD_COLUMNS = ['depth', 'pressure', 'temperature', 'conductivity', 'salinity', 'density', 'sound_velocity']
_CTD_UNITS   = ['m',     'm',        '°C',          'mS/cm',        'PSU',      'kg/m³',   'm/s']


# ─── CTD Profile Simulation ────────────────────────────────────────────────────

def _simulate_profile(depth_m: float, progress: float) -> dict:
    """
    Realistic oceanographic profile using sigmoid thermocline / halocline.
    progress : 0.0–1.0, position along transect cycle — shifts thermocline depth
               and surface values so heatmap shows meaningful horizontal variation.
    """
    # ── Temperature ─────────────────────────────────────
    surface_temp  = 30.0 - 1.5 * progress                    # 30°C → 28.5°C
    bottom_temp   = 18.0 + 2.5 * math.sin(progress * math.pi)  # 18°C → 20.5°C
    therm_depth   = 40.0 + 25.0 * progress                   # thermocline: 40m → 65m
    therm_sharp   = 0.25
    temperature_c = bottom_temp + (surface_temp - bottom_temp) / (
        1.0 + math.exp(therm_sharp * (depth_m - therm_depth))
    )
    temperature_c += 0.08 * math.sin(depth_m / 8.0)          # small-scale noise

    # ── Salinity ─────────────────────────────────────────
    surface_sal     = 31.0 + 0.8 * progress                  # 31 → 31.8 PSU
    bottom_sal      = 34.5
    halocline_depth = 25.0 + 12.0 * progress                 # halocline: 25m → 37m
    salinity_psu    = surface_sal + (bottom_sal - surface_sal) / (
        1.0 + math.exp(-0.15 * (depth_m - halocline_depth))
    )
    salinity_psu += 0.05 * math.sin(depth_m / 5.0)

    # ── Derived parameters ───────────────────────────────
    pressure_m       = depth_m * 1.02
    conductivity     = 40.0 + 0.3 * salinity_psu + 0.2 * temperature_c
    density          = 1024.0 + 0.8 * (salinity_psu - 35.0) - 0.25 * (temperature_c - 25.0)
    sound_velocity   = (
        1480.0
        + 4.0  * (temperature_c - 25.0)
        + 1.5  * (salinity_psu  - 35.0)
        + 0.016 * depth_m
    )

    return {
        'depth_m':             round(depth_m,       2),
        'pressure_m':          round(pressure_m,    2),
        'temperature_c':       round(temperature_c, 2),
        'conductivity_ms_cm':  round(conductivity,  2),
        'salinity_psu':        round(salinity_psu,  2),
        'density_kg_m3':       round(density,       2),
        'sound_velocity_ms':   round(sound_velocity, 2),
    }


# ─── Main Node ─────────────────────────────────────────────────────────────────

class CTDSensorNode(Node):
    def __init__(self):
        super().__init__('ctd_sensor_node')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('oceanography.ctd.publish_topic',    'oceanography/ctd')
        self.declare_parameter('oceanography.ctd.publish_rate_hz',  0.1)
        self.declare_parameter('vehicle.id',                        'USV-001')
        self.declare_parameter('transport.mode',                    'mqtt')
        self.declare_parameter('oceanography.ctd.sensor_code',      'CTD-MIDAS-3000')
        self.declare_parameter('oceanography.ctd.timezone',         'Asia/Jakarta')
        self.declare_parameter('oceanography.ctd.gps_topic',        '/mavros/global_position/global')
        self.declare_parameter('oceanography.ctd.default_latitude',  0.0)
        self.declare_parameter('oceanography.ctd.default_longitude', 0.0)
        self.declare_parameter('oceanography.ctd.default_altitude',  0.0)
        self.declare_parameter('oceanography.ctd.max_depth_m',       133.0)
        self.declare_parameter('oceanography.ctd.cycle_seconds',     240.0)
        self.declare_parameter('mqtt.broker',                        'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port',                          8883)
        self.declare_parameter('mqtt.username',                      '')
        self.declare_parameter('mqtt.password',                      '')
        self.declare_parameter('mqtt.base_topic',                    'seano')
        self.declare_parameter('mqtt.qos',                           1)
        self.declare_parameter('validation.force_gps_fix_for_publish',          False)
        self.declare_parameter('validation.allow_fallback_coords_without_gps',  True)
        self.declare_parameter('validation.fallback_latitude',                  -6.87)
        self.declare_parameter('validation.fallback_longitude',                 107.58)
        self.declare_parameter('validation.fallback_motion_enabled',            True)
        self.declare_parameter('validation.fallback_motion_radius_m',           500.0)
        self.declare_parameter('validation.fallback_motion_step_min_m',         5.0)
        self.declare_parameter('validation.fallback_motion_step_max_m',         15.0)
        self.declare_parameter('logger.log_dir',                                '~/Seano_ws/ros_log')

        # ── Read params ─────────────────────────────────────────────────────────
        topic          = self.get_parameter('oceanography.ctd.publish_topic').value
        rate_hz        = float(self.get_parameter('oceanography.ctd.publish_rate_hz').value)
        period         = 1.0 / max(rate_hz, 0.01)

        self.vehicle_code  = self.get_parameter('vehicle.id').value
        self.sensor_code   = self.get_parameter('oceanography.ctd.sensor_code').value
        transport_mode     = str(self.get_parameter('transport.mode').value).strip().lower()
        if transport_mode not in ('mqtt', 'api', 'both'):
            transport_mode = 'mqtt'
        self._enable_mqtt  = transport_mode in ('mqtt', 'both')

        self.timezone_name = self.get_parameter('oceanography.ctd.timezone').value
        self.gps_topic     = self.get_parameter('oceanography.ctd.gps_topic').value
        self.latitude      = float(self.get_parameter('oceanography.ctd.default_latitude').value)
        self.longitude     = float(self.get_parameter('oceanography.ctd.default_longitude').value)
        self.altitude      = float(self.get_parameter('oceanography.ctd.default_altitude').value)
        self.gps_ok        = False
        self.max_depth_m   = max(float(self.get_parameter('oceanography.ctd.max_depth_m').value), 5.0)
        self.cycle_seconds = max(float(self.get_parameter('oceanography.ctd.cycle_seconds').value), 20.0)

        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port   = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic  = self.get_parameter('mqtt.base_topic').value
        self.qos         = int(self.get_parameter('mqtt.qos').value)
        self.mqtt_topic  = f'{self.base_topic}/{self.vehicle_code}/{self.sensor_code}/data'

        self.force_gps_fix_for_publish         = bool(self.get_parameter('validation.force_gps_fix_for_publish').value)
        self.allow_fallback_coords_without_gps = bool(self.get_parameter('validation.allow_fallback_coords_without_gps').value)
        self.fallback_latitude                 = float(self.get_parameter('validation.fallback_latitude').value)
        self.fallback_longitude                = float(self.get_parameter('validation.fallback_longitude').value)
        self.fallback_motion_enabled           = bool(self.get_parameter('validation.fallback_motion_enabled').value)
        self.fallback_motion_radius_m          = max(0.0, float(self.get_parameter('validation.fallback_motion_radius_m').value))
        self.fallback_motion_step_min_m        = float(self.get_parameter('validation.fallback_motion_step_min_m').value)
        self.fallback_motion_step_max_m        = float(self.get_parameter('validation.fallback_motion_step_max_m').value)

        # ── Internal state ──────────────────────────────────────────────────────
        self._start_clock   = None   # set on first publish, used for elapsed time
        self._fallback_pos  = None   # (lat, lon) current random-walk position
        self._mqtt_connected = False
        self.fallback_tz    = _safe_zoneinfo(self.timezone_name, self.get_logger())

        # ── CSV logger ──────────────────────────────────────────────────────────
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._ctd_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'oceanography'),
            'ctd_log',
            _CTD_FIELDS,
        )

        # ── ROS setup ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher_ = self.create_publisher(String, topic, 10)
        self.gps_sub    = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, sensor_qos)
        self.timer      = self.create_timer(period, self.publish_measurement)

        # ── MQTT ────────────────────────────────────────────────────────────────
        self.mqtt_client = self._create_mqtt_client()

        self.get_logger().info(f'CTD sensor node started — ROS topic: {topic}')
        self.get_logger().info(f'CTD MQTT topic: {self.mqtt_topic}')
        self.get_logger().info(f'CTD GPS source: {self.gps_topic}')
        self.get_logger().info(f'force_gps_fix={self.force_gps_fix_for_publish}  allow_fallback={self.allow_fallback_coords_without_gps}')

    # ── Helpers ──────────────────────────────────────────────────────────────────

    def _offset_m_to_coords(self, center_lat, center_lon, east_m, north_m):
        lat_scale = 111320.0
        lon_scale = 111320.0 * max(0.1, math.cos(math.radians(float(center_lat))))
        lat = float(center_lat) + (north_m / lat_scale)
        lon = float(center_lon) + (east_m  / lon_scale)
        return lat, lon

    def _dist_from_center_m(self, lat, lon):
        d_lat = (lat - self.fallback_latitude)  * 111320.0
        d_lon = (lon - self.fallback_longitude) * 111320.0 * math.cos(math.radians(self.fallback_latitude))
        return math.sqrt(d_lat ** 2 + d_lon ** 2)

    def _next_fallback_coords(self):
        """
        Random walk GPS — bergerak 5–15 m acak per step dari posisi sebelumnya.
        Dipantulkan kembali ke arah center kalau sudah melebihi fallback_motion_radius_m.
        """
        if not _coords_valid(self.fallback_latitude, self.fallback_longitude):
            return None

        if not self.fallback_motion_enabled or self.fallback_motion_radius_m <= 0.0:
            return round(self.fallback_latitude, 7), round(self.fallback_longitude, 7)

        # Init posisi awal
        if self._fallback_pos is None:
            self._fallback_pos = (self.fallback_latitude, self.fallback_longitude)

        lat, lon = self._fallback_pos

        step_m = random.uniform(
            self.fallback_motion_step_min_m,
            self.fallback_motion_step_max_m,
        )
        angle  = random.uniform(0.0, 2.0 * math.pi)
        east_m  = step_m * math.cos(angle)
        north_m = step_m * math.sin(angle)

        new_lat, new_lon = self._offset_m_to_coords(lat, lon, east_m, north_m)

        # Pantulkan ke arah center kalau keluar batas radius
        if self._dist_from_center_m(new_lat, new_lon) > self.fallback_motion_radius_m:
            d_lat = lat - self.fallback_latitude
            d_lon = lat - self.fallback_longitude
            angle_to_center = math.atan2(-d_lat, -d_lon)
            east_m  = step_m * math.cos(angle_to_center)
            north_m = step_m * math.sin(angle_to_center)
            new_lat, new_lon = self._offset_m_to_coords(lat, lon, east_m, north_m)

        self._fallback_pos = (new_lat, new_lon)
        return round(new_lat, 7), round(new_lon, 7)

    # ── GPS callback ─────────────────────────────────────────────────────────────

    def gps_callback(self, msg: NavSatFix):
        self.latitude  = msg.latitude
        self.longitude = msg.longitude
        self.altitude  = msg.altitude
        self.gps_ok    = msg.status.status >= 0

    # ── MQTT ─────────────────────────────────────────────────────────────────────

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = (rc == 0)
        if rc == 0:
            self.get_logger().info(f'CTD MQTT connected to {self.mqtt_broker}:{self.mqtt_port}')
        else:
            self.get_logger().warn(f'CTD MQTT connect failed rc={rc}')

    def _mqtt_on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        self.get_logger().warn(f'CTD MQTT disconnected rc={rc}, will auto-reconnect')

    def _create_mqtt_client(self):
        if not self._enable_mqtt:
            self.get_logger().info('CTD MQTT disabled (transport.mode=api)')
            return None
        client = mqtt.Client()
        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)
        # TLS hanya untuk port 8883
        if self.mqtt_port == 8883:
            client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        client.on_connect    = self._mqtt_on_connect
        client.on_disconnect = self._mqtt_on_disconnect
        client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=60)
        client.loop_start()
        return client

    # ── Publish ──────────────────────────────────────────────────────────────────

    def publish_measurement(self):
        # Elapsed time via ROS clock (reliable, tidak bergantung timer_period_ns)
        now_clock = self.get_clock().now()
        if self._start_clock is None:
            self._start_clock = now_clock
        elapsed = (now_clock - self._start_clock).nanoseconds / 1e9

        # ── Resolve coordinates ─────────────────────────────────────────────────
        has_fix = self.gps_ok and _coords_valid(self.latitude, self.longitude)

        if has_fix:
            publish_lat    = self.latitude
            publish_lon    = self.longitude
            publish_alt    = self.altitude
            publish_gps_ok = True
        elif self.allow_fallback_coords_without_gps:
            fb = self._next_fallback_coords()
            if fb is None:
                self.get_logger().debug('CTD: No valid fallback coordinates, skipping.')
                return
            publish_lat, publish_lon = fb
            publish_alt    = self.altitude
            publish_gps_ok = False
        else:
            if self.force_gps_fix_for_publish:
                self.get_logger().debug('CTD: GPS fix required but not available, skipping.')
                return
            publish_lat    = self.latitude
            publish_lon    = self.longitude
            publish_alt    = self.altitude
            publish_gps_ok = False

        # ── Simulate profile ────────────────────────────────────────────────────
        # progress 0→1 dalam satu cycle, menggeser thermocline depth & surface values
        progress = (elapsed % self.cycle_seconds) / self.cycle_seconds

        num_points = 133
        profile_data = []
        for i in range(num_points):
            depth_m = max(0.5, (i / (num_points - 1)) * self.max_depth_m)
            profile_data.append(_simulate_profile(depth_m, progress))

        # ── Build payload ───────────────────────────────────────────────────────
        tz        = _resolve_timezone(publish_lat, publish_lon, publish_gps_ok, self.fallback_tz)
        now_local = datetime.now(tz).isoformat()

        data_rows = [
            [
                p['depth_m'], p['pressure_m'], p['temperature_c'],
                p['conductivity_ms_cm'], p['salinity_psu'],
                p['density_kg_m3'], p['sound_velocity_ms'],
            ]
            for p in profile_data
        ]

        full_payload = {
            'timestamp':    now_local,
            'vehicle_code': self.vehicle_code,
            'sensor_code':  self.sensor_code,
            'latitude':     round(publish_lat, 7),
            'longitude':    round(publish_lon, 7),
            'altitude':     round(publish_alt, 2),
            'gps_ok':       publish_gps_ok,
            'columns':      _CTD_COLUMNS,
            'units':        _CTD_UNITS,
            'data':         data_rows,
        }

        payload_str = json.dumps(full_payload)

        # ── ROS publish ─────────────────────────────────────────────────────────
        msg      = String()
        msg.data = payload_str
        self.publisher_.publish(msg)

        # ── MQTT publish ────────────────────────────────────────────────────────
        if self.mqtt_client is not None and self._mqtt_connected:
            self.mqtt_client.publish(self.mqtt_topic, payload_str, qos=self.qos)

            mqtt_ts = datetime.now(tz).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]
            for point in profile_data:
                self._ctd_csv.write({
                    'timestamp':    now_local,
                    'vehicle_code': self.vehicle_code,
                    'sensor_code':  self.sensor_code,
                    'latitude':     round(publish_lat, 7),
                    'longitude':    round(publish_lon, 7),
                    'altitude':     round(publish_alt, 2),
                    'gps_ok':       publish_gps_ok,
                    'mqtt_publish_timestamp': mqtt_ts,
                    **point,
                })

        self.get_logger().debug(
            f'CTD published: lat={publish_lat:.6f} lon={publish_lon:.6f} '
            f'gps_ok={publish_gps_ok} depth_max={self.max_depth_m}m '
            f'progress={progress:.2f} elapsed={elapsed:.1f}s'
        )

    # ── Cleanup ──────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._ctd_csv.close()
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = CTDSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
