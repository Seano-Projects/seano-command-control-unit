#!/usr/bin/env python3
import json
import math
import ssl
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import RCIn, State, VfrHud
from mavros_msgs.srv import SetMode
from std_msgs.msg import String

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


class AntiTheftNode(Node):
    def __init__(self):
        super().__init__('anti_theft_node')

        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('anti_theft.loop_rate_hz', 1.0)
        self.declare_parameter('anti_theft.alert_topic', 'anti_theft/alert')
        self.declare_parameter('anti_theft.target_speed_mps', 1.0)

        self.declare_parameter('anti_theft.mavros.gps_topic', '/mavros/global_position/global')
        self.declare_parameter('anti_theft.mavros.imu_topic', '/mavros/imu/data')
        self.declare_parameter('anti_theft.mavros.vfr_topic', '/mavros/vfr_hud')
        self.declare_parameter('anti_theft.mavros.state_topic', '/mavros/state')
        self.declare_parameter('anti_theft.mavros.rc_in_topic', '/mavros/rc/in')
        self.declare_parameter('anti_theft.mavros.set_mode_service', '/mavros/set_mode')

        self.declare_parameter('anti_theft.mqtt_enabled', True)
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', False)

        self.declare_parameter('anti_theft.geofence_limit', 10.0)
        self.declare_parameter('anti_theft.crit_tilt_deg', 10.0)
        self.declare_parameter('anti_theft.tilt_confirm_time', 7.0)
        self.declare_parameter('anti_theft.rc_failsafe_pwm', 975)
        self.declare_parameter('anti_theft.mission_speed_margin', 1.5)

        alert_topic = str(self.get_parameter('anti_theft.alert_topic').value)
        self.alert_pub = self.create_publisher(String, alert_topic, 10)

        self.vehicle_id = str(self.get_parameter('vehicle.id').value)
        loop_rate_hz = float(self.get_parameter('anti_theft.loop_rate_hz').value)
        self.loop_period = 1.0 / max(loop_rate_hz, 0.1)
        self.target_speed = float(self.get_parameter('anti_theft.target_speed_mps').value)

        self.gps_topic = str(self.get_parameter('anti_theft.mavros.gps_topic').value)
        self.imu_topic = str(self.get_parameter('anti_theft.mavros.imu_topic').value)
        self.vfr_topic = str(self.get_parameter('anti_theft.mavros.vfr_topic').value)
        self.state_topic = str(self.get_parameter('anti_theft.mavros.state_topic').value)
        self.rc_in_topic = str(self.get_parameter('anti_theft.mavros.rc_in_topic').value)
        self.set_mode_service = str(self.get_parameter('anti_theft.mavros.set_mode_service').value)

        self.mqtt_enabled = bool(self.get_parameter('anti_theft.mqtt_enabled').value)
        self.mqtt_broker = str(self.get_parameter('mqtt.broker').value)
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = str(self.get_parameter('mqtt.username').value)
        self.mqtt_password = str(self.get_parameter('mqtt.password').value)
        self.mqtt_base_topic = str(self.get_parameter('mqtt.base_topic').value)
        self.mqtt_qos = int(self.get_parameter('mqtt.qos').value)
        self.mqtt_keepalive = int(self.get_parameter('mqtt.keepalive').value)
        self.mqtt_use_tls = bool(self.get_parameter('mqtt.use_tls').value)
        self.mqtt_tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)
        self.mqtt_alert_topic = f'{self.mqtt_base_topic}/{self.vehicle_id}/anti_theft/alert'

        self.geofence_limit = float(self.get_parameter('anti_theft.geofence_limit').value)
        self.crit_tilt_deg = float(self.get_parameter('anti_theft.crit_tilt_deg').value)
        self.tilt_confirm_time = float(self.get_parameter('anti_theft.tilt_confirm_time').value)
        self.rc_failsafe_pwm = int(self.get_parameter('anti_theft.rc_failsafe_pwm').value)
        self.mission_speed_margin = float(self.get_parameter('anti_theft.mission_speed_margin').value)

        self.security_active = False
        self.home_location = None
        self.tilt_start_time = None
        self.last_alarm = None

        self.lat = None
        self.lon = None
        self.alt = None
        self.heading = None
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.tilt_deg = 0.0
        self.groundspeed = 0.0
        self.mode_name = 'UNKNOWN'
        self.rc3_pwm = None
        self.gps_fix_type = -1

        self.set_mode_client = self.create_client(SetMode, self.set_mode_service)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, sensor_qos)
        self.create_subscription(VfrHud, self.vfr_topic, self.vfr_callback, sensor_qos)
        self.create_subscription(State, self.state_topic, self.state_callback, sensor_qos)
        self.create_subscription(RCIn, self.rc_in_topic, self.rc_in_callback, sensor_qos)

        if self.mqtt_enabled:
            self._setup_mqtt()

        self.timer = self.create_timer(self.loop_period, self._main_loop)

        self.get_logger().info('ANTI-THEFT SEANO READY (MAVROS topic mode)')
        self.get_logger().info(f'GPS topic: {self.gps_topic}')
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'VFR topic: {self.vfr_topic}')
        self.get_logger().info(f'RC input topic: {self.rc_in_topic}')

    def _setup_mqtt(self):
        if mqtt is None:
            self.get_logger().error('paho-mqtt belum terpasang, MQTT dinonaktifkan')
            self.mqtt_client = None
            return

        try:
            self.mqtt_client = mqtt.Client(client_id=f'anti_theft_{self.vehicle_id}')
            if self.mqtt_username:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)

            if self.mqtt_use_tls:
                self.mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
                self.mqtt_client.tls_insecure_set(self.mqtt_tls_insecure)

            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'MQTT connected: {self.mqtt_broker}:{self.mqtt_port} topic={self.mqtt_alert_topic}'
            )
        except Exception as exc:
            self.get_logger().error(f'MQTT connection failed: {exc}')
            self.mqtt_client = None

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        self.gps_fix_type = int(msg.status.status)

    def imu_callback(self, msg):
        q = msg.orientation

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll_deg = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.pitch_deg = math.degrees(math.copysign(math.pi / 2.0, sinp))
        else:
            self.pitch_deg = math.degrees(math.asin(sinp))

        self.tilt_deg = math.sqrt(self.roll_deg ** 2 + self.pitch_deg ** 2)

    def vfr_callback(self, msg):
        self.groundspeed = float(msg.groundspeed)
        self.heading = float(msg.heading)

    def state_callback(self, msg):
        self.mode_name = str(msg.mode)

    def rc_in_callback(self, msg):
        if len(msg.channels) >= 3:
            self.rc3_pwm = int(msg.channels[2])

    @staticmethod
    def _distance_meters(loc1, loc2):
        if loc1 is None or loc2 is None:
            return 0.0

        lat1, lon1 = loc1
        lat2, lon2 = loc2

        if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
            return 0.0

        earth_radius = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (
            math.sin(dlat / 2.0) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2.0) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return earth_radius * c

    def _set_rtl_mode(self):
        try:
            if not self.set_mode_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().error('SetMode service not available')
                return False

            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = 'RTL'
            self.set_mode_client.call_async(req)
            self.get_logger().warn('ACTION: RTL command sent to flight controller')
            return True
        except Exception as exc:
            self.get_logger().error(f'Failed to set RTL mode: {exc}')
            return False

    def _publish_outputs(self, payload, alarm):
        if alarm:
            alert_msg = String()
            alert_msg.data = alarm
            self.alert_pub.publish(alert_msg)

        if alarm and self.mqtt_client is not None:
            try:
                alert_payload = json.dumps({'vehicle_id': self.vehicle_id, 'alert': alarm})
                self.mqtt_client.publish(self.mqtt_alert_topic, alert_payload, qos=self.mqtt_qos)
            except Exception as exc:
                self.get_logger().error(f'MQTT publish failed: {exc}')

    def _main_loop(self):
        lat = self.lat
        lon = self.lon
        alt = self.alt
        heading = self.heading
        roll = self.roll_deg
        pitch = self.pitch_deg
        tilt = self.tilt_deg
        groundspeed = self.groundspeed
        current_mode = self.mode_name
        rc3 = self.rc3_pwm
        target_speed = self.target_speed
        gps_fix_type = self.gps_fix_type

        if lat is None or lon is None:
            self.get_logger().info('Menunggu GPS fix / posisi...')
            return

        current_location = (lat, lon)

        if self.home_location is None and gps_fix_type >= 0:
            self.home_location = current_location
            self.get_logger().info(f'Home location locked at {lat:.7f}, {lon:.7f}')

        drift = 0.0
        alarm = None

        rc3_value = 0 if rc3 is None else rc3

        if rc3_value < self.rc_failsafe_pwm:
            if not self.security_active:
                self.home_location = current_location
                self.security_active = True
                self.get_logger().warn('RC LOST: SECURITY ACTIVE')
        else:
            if self.security_active:
                self.security_active = False
                self.tilt_start_time = None
                self.get_logger().info('RC DETECTED: SECURITY STANDBY')

        if current_mode in ['AUTO', 'GUIDED']:
            self.home_location = current_location
            max_allowed_speed = max(float(target_speed), 0.0) + self.mission_speed_margin
            if groundspeed > max_allowed_speed:
                alarm = 'TOWING DETECTED'
        else:
            if self.security_active and self.home_location is not None:
                drift = self._distance_meters(self.home_location, current_location)

                if drift > self.geofence_limit:
                    alarm = 'GEOFENCE BREACH'
                    if current_mode != 'RTL':
                        self._set_rtl_mode()

                if tilt > self.crit_tilt_deg:
                    if self.tilt_start_time is None:
                        self.tilt_start_time = time.time()
                    elif (time.time() - self.tilt_start_time) > self.tilt_confirm_time:
                        alarm = 'BOAT FLIPPED'
                else:
                    self.tilt_start_time = None

        payload = {
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'heading': heading,
            'roll': roll,
            'pitch': pitch,
            'tilt': tilt,
            'drift': drift,
            'groundspeed': groundspeed,
            'mode': current_mode,
            'security_active': self.security_active,
            'rc3_pwm': rc3,
            'alarm': alarm,
            'home_lat': None if self.home_location is None else self.home_location[0],
            'home_lon': None if self.home_location is None else self.home_location[1],
            'gps_fix_type': gps_fix_type,
            'gps_ok': gps_fix_type >= 0,
            'timestamp': int(time.time()),
        }

        self.get_logger().info(
            f'[{current_mode}] GPS:{lat:.7f},{lon:.7f} | Tilt:{tilt:.1f} | Drift:{drift:.1f}m | GS:{groundspeed:.2f} | RC3:{rc3}'
        )

        if alarm and alarm != self.last_alarm:
            self.get_logger().warn(f'ALERT: {alarm}')

        self.last_alarm = alarm
        self._publish_outputs(payload, alarm)

    def destroy_node(self):
        try:
            if self.mqtt_client is not None:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AntiTheftNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()