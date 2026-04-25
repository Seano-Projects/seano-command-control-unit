import csv
import json
import math
import os
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
import ssl
from std_msgs.msg import String


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


_CTD_FIELDS = [
    'date_time', 'vehicle_code', 'sensor_code', 'sensor',
    'latitude', 'longitude', 'altitude', 'gps_ok',
    'depth_m', 'pressure_m', 'temperature_c', 'conductivity_ms_cm',
    'salinity_psu', 'density_kg_m3', 'sound_velocity_ms',
    'mqtt_publish_timestamp',
]


class CTDSensorNode(Node):
    def __init__(self):
        super().__init__('ctd_sensor_node')

        self.declare_parameter('oceanography.ctd.publish_topic', 'oceanography/ctd')
        self.declare_parameter('oceanography.ctd.publish_rate_hz', 2.0)
        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('transport.mode', 'mqtt')
        self.declare_parameter('oceanography.ctd.sensor_code', 'CTD-MIDAS-3000')
        self.declare_parameter('oceanography.ctd.timezone', 'Asia/Jakarta')
        self.declare_parameter('oceanography.ctd.gps_topic', '/mavros/global_position/global')
        self.declare_parameter('oceanography.ctd.default_latitude', 0.0)
        self.declare_parameter('oceanography.ctd.default_longitude', 0.0)
        self.declare_parameter('oceanography.ctd.default_altitude', 0.0)
        self.declare_parameter('oceanography.ctd.max_depth_m', 120.0)
        self.declare_parameter('oceanography.ctd.cycle_seconds', 240.0)
        self.declare_parameter('mqtt.broker', 'localhost')
        self.declare_parameter('mqtt.port', 1883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)

        topic = self.get_parameter('oceanography.ctd.publish_topic').value
        rate_hz = float(self.get_parameter('oceanography.ctd.publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)
        self.vehicle_code = self.get_parameter('vehicle.id').value
        self.sensor_code = self.get_parameter('oceanography.ctd.sensor_code').value
        transport_mode = str(self.get_parameter('transport.mode').value).strip().lower()
        if transport_mode not in ('mqtt', 'api', 'both'):
            transport_mode = 'mqtt'
        self._enable_mqtt = transport_mode in ('mqtt', 'both')
        self.timezone_name = self.get_parameter('oceanography.ctd.timezone').value
        self.gps_topic = self.get_parameter('oceanography.ctd.gps_topic').value
        self.latitude = float(self.get_parameter('oceanography.ctd.default_latitude').value)
        self.longitude = float(self.get_parameter('oceanography.ctd.default_longitude').value)
        self.altitude = float(self.get_parameter('oceanography.ctd.default_altitude').value)
        self.gps_ok = False
        self.max_depth_m = max(float(self.get_parameter('oceanography.ctd.max_depth_m').value), 5.0)
        self.cycle_seconds = max(float(self.get_parameter('oceanography.ctd.cycle_seconds').value), 20.0)
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.qos = int(self.get_parameter('mqtt.qos').value)
        self.mqtt_topic = f'{self.base_topic}/{self.vehicle_code}/{self.sensor_code}/data'
        self.sample_index = 0

        # CSV Logger
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        _log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._ctd_csv = _DailyCsvWriter(
            os.path.join(_log_dir, 'oceanography'),
            'ctd_log',
            _CTD_FIELDS,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            sensor_qos,
        )
        self.timer = self.create_timer(period, self.publish_measurement)
        self.mqtt_client = self._create_mqtt_client()

        try:
            self.local_tz = ZoneInfo(self.timezone_name)
        except Exception:
            self.get_logger().warn(f'Invalid timezone: {self.timezone_name}, fallback to UTC')
            self.local_tz = timezone.utc

        self.get_logger().info(f'CTD sensor node started on topic: {topic}')
        self.get_logger().info(f'CTD MQTT topic: {self.mqtt_topic}')
        self.get_logger().info(f'CTD GPS source: {self.gps_topic}')

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        self.gps_ok = msg.status.status >= 0

    def _create_mqtt_client(self):
        if not self._enable_mqtt:
            self.get_logger().info('CTD MQTT disabled (transport.mode=api)')
            return None
        client = mqtt.Client()
        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)
        client.tls_set(cert_reqs=ssl.CERT_NONE)
        client.tls_insecure_set(True)

        try:
            client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            client.loop_start()
            self.get_logger().info(f'Connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}')
            return client
        except Exception as exc:
            self.get_logger().error(f'MQTT connection failed: {exc}')
            return None

    def publish_measurement(self):
        elapsed = self.sample_index * self.timer.timer_period_ns / 1e9
        cycle_phase = (elapsed % self.cycle_seconds) / self.cycle_seconds

        # Auto-generate lat/lon if MAVROS GPS not active (for heatmap demo)
        if not self.gps_ok:
            default_lat = float(self.get_parameter('oceanography.ctd.default_latitude').value)
            default_lon = float(self.get_parameter('oceanography.ctd.default_longitude').value)
            # Generate survey-like path: oscillating drift
            lat_offset = (
                0.015 * math.sin(elapsed * 0.08 + 0.0) +
                0.008 * math.sin(elapsed * 0.15 + 1.2)
            )
            lon_offset = (
                0.015 * math.cos(elapsed * 0.08 + 0.3) +
                0.008 * math.cos(elapsed * 0.15 + 2.1)
            )
            self.latitude = default_lat + lat_offset
            self.longitude = default_lon + lon_offset

        # Slow profiling movement (dive/climb) + fast wave components for visible motion
        depth_base = self.max_depth_m * 0.5 * (1.0 - math.cos(2.0 * math.pi * cycle_phase))
        depth_wave_fast = (
            8.0 * math.sin(elapsed * 0.95 + 0.6) +
            4.2 * math.sin(elapsed * 1.75 + 1.2) +
            1.8 * math.cos(elapsed * 2.6 + 0.3)
        )
        depth_m = max(0.5, depth_base + depth_wave_fast)

        # Thermocline baseline with stronger, quicker oscillations
        if depth_m <= 20.0:
            temp_profile = -0.05 * depth_m
        elif depth_m <= 50.0:
            temp_profile = -1.0 - 0.14 * (depth_m - 20.0)
        else:
            temp_profile = -5.2 - 0.055 * (depth_m - 50.0)
        temp_wave = (
            1.8 * math.sin(elapsed * 0.75 + 0.5) +
            1.2 * math.sin(elapsed * 1.35 + 1.8) +
            0.8 * math.cos(elapsed * 2.1 + 0.3)
        )
        temperature_c = 29.5 + temp_profile + temp_wave

        # Halocline + quick tidal-like oscillations
        salt_profile = 0.025 * depth_m + 0.6 * math.log1p(depth_m * 0.08)
        salt_wave = (
            0.65 * math.sin(elapsed * 0.68 + 1.0) +
            0.45 * math.sin(elapsed * 1.28 + 2.4) +
            0.30 * math.cos(elapsed * 2.3 + 0.7)
        )
        salinity_psu = 31.5 + salt_profile + salt_wave

        pressure = depth_m * 1.02
        conductivity_ms_cm = (
            38.0 +
            0.28 * salinity_psu +
            0.18 * temperature_c +
            0.55 * math.sin(elapsed * 1.9 + 0.2)
        )
        density_kg_m3 = (
            1025.0 +
            0.78 * (salinity_psu - 35.0) -
            0.2 * temperature_c +
            0.004 * depth_m +
            0.9 * math.sin(elapsed * 1.15 + 0.9)
        )

        # Mackenzie-like approximation for seawater sound speed (m/s)
        sound_velocity_ms = (
            1449.2
            + 4.6 * temperature_c
            - 0.055 * temperature_c ** 2
            + 0.00029 * temperature_c ** 3
            + (1.34 - 0.01 * temperature_c) * (salinity_psu - 35.0)
            + 0.016 * depth_m
            + 1.8 * math.sin(elapsed * 1.6 + 0.4)
        )

        now_local = datetime.now(self.local_tz).isoformat()
        payload = {
            'date_time': now_local,
            'vehicle_code': self.vehicle_code,
            'sensor_code': self.sensor_code,
            'sensor': 'CTD',
            'latitude': round(self.latitude, 7),
            'longitude': round(self.longitude, 7),
            'altitude': round(self.altitude, 2),
            'gps_ok': self.gps_ok,
            'depth_m': round(depth_m, 2),
            'pressure_m': round(pressure, 2),
            'temperature_c': round(temperature_c, 2),
            'conductivity_ms_cm': round(conductivity_ms_cm, 2),
            'salinity_psu': round(salinity_psu, 2),
            'density_kg_m3': round(density_kg_m3, 2),
            'sound_velocity_ms': round(sound_velocity_ms, 2),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)

        if self.mqtt_client is not None:
            mqtt_publish_ts = datetime.now(self.local_tz).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]
            self.mqtt_client.publish(self.mqtt_topic, msg.data, qos=self.qos)
            row = dict(payload)
            row['mqtt_publish_timestamp'] = mqtt_publish_ts
            self._ctd_csv.write(row)

        self.sample_index += 1

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
