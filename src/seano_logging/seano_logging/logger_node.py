import os
import re
import subprocess
import time
from datetime import datetime
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import String


class SeanoLogger(Node):
    def __init__(self):
        super().__init__('logger_node')

        self.declare_parameter('logging.mount_point', '/media/seano/SEANO_SSD')
        self.declare_parameter('logging.flush_interval', 3.0)
        self.declare_parameter('logging.auto_mount', True)
        self.declare_parameter('logging.device_uuid', '')
        self.declare_parameter('logging.device_label', 'SEANO_SSD')
        self.declare_parameter('logging.topics.gps', '/mavros/global_position/global')
        self.declare_parameter('logging.topics.imu', '/mavros/imu/data')
        self.declare_parameter('logging.topics.battery', '/mavros/battery')
        self.declare_parameter('logging.topics.ctd', 'oceanography/ctd')
        self.declare_parameter('logging.topics.adcp', 'oceanography/adcp')
        self.declare_parameter('logging.topics.sbes', 'oceanography/sbes')
        self.declare_parameter('logging.topics.telemetry', 'telemetry')

        self.mount_point = str(self.get_parameter('logging.mount_point').value)
        self.flush_interval = float(self.get_parameter('logging.flush_interval').value)
        self.auto_mount = bool(self.get_parameter('logging.auto_mount').value)
        self.device_uuid = str(self.get_parameter('logging.device_uuid').value).strip()
        self.device_label = str(self.get_parameter('logging.device_label').value).strip()
        self.gps_topic = str(self.get_parameter('logging.topics.gps').value)
        self.imu_topic = str(self.get_parameter('logging.topics.imu').value)
        self.battery_topic = str(self.get_parameter('logging.topics.battery').value)
        self.ctd_topic = str(self.get_parameter('logging.topics.ctd').value)
        self.adcp_topic = str(self.get_parameter('logging.topics.adcp').value)
        self.sbes_topic = str(self.get_parameter('logging.topics.sbes').value)
        self.telemetry_topic = str(self.get_parameter('logging.topics.telemetry').value)

        self.ensure_mount_ready()

        self.start_time_obj = datetime.now()
        self.local_timezone = time.tzname[0]

        year = self.start_time_obj.strftime('%Y')
        month = self.start_time_obj.strftime('%m')
        day = self.start_time_obj.strftime('%d')

        self.mission_id = self.start_time_obj.strftime(
            f'MISSION_START_%H-%M-%S_{self.local_timezone}'
        )

        self.base_path = os.path.join(
            self.mount_point,
            'SEANO_MISSIONS',
            year,
            month,
            day,
            self.mission_id,
        )
        os.makedirs(self.base_path, exist_ok=True)

        with open(os.path.join(self.base_path, 'mission_info.txt'), 'w') as f:
            f.write(f'Start Time: {self.start_time_obj}\n')
            f.write(f'Timezone: {self.local_timezone}\n')
            f.write('Platform: SEANO USV\n')

        self.files = {}
        self.csv_files = {}
        self.buffers = {}
        self.sample_count = {}
        self.detected_sensors = set()
        self.subscriptions_map = {}

        self.create_timer(2.0, self.detect_and_initialize_sensors)
        self.create_timer(self.flush_interval, self.flush_buffers)

        self.get_logger().info(f'Mission folder: {self.base_path}')
        self.get_logger().info(f'Timezone: {self.local_timezone}')
        self.get_logger().info('SEANO Logger Started')

    def ensure_mount_ready(self):
        if os.path.ismount(self.mount_point):
            self.get_logger().info(f'SSD mounted at {self.mount_point}')
            return

        if not self.auto_mount:
            raise RuntimeError(
                f'SSD mount point is not mounted: {self.mount_point}. '
                'Enable logging.auto_mount or mount manually first.'
            )

        os.makedirs(self.mount_point, exist_ok=True)
        device_path = self.resolve_device_path()
        mounted_path = self.mount_with_udiskctl(device_path)

        if mounted_path:
            if mounted_path != self.mount_point:
                self.get_logger().warn(
                    f'SSD auto-mounted at {mounted_path}, overriding configured mount point '
                    f'{self.mount_point}'
                )
                self.mount_point = mounted_path
            return

        raise RuntimeError(
            f'Failed to auto-mount SSD device {device_path}. '
            f'Please mount it manually at {self.mount_point}.'
        )

    def resolve_device_path(self):
        if self.device_uuid:
            by_uuid = f'/dev/disk/by-uuid/{self.device_uuid}'
            if os.path.exists(by_uuid):
                return os.path.realpath(by_uuid)

        if self.device_label:
            by_label = f'/dev/disk/by-label/{self.device_label}'
            if os.path.exists(by_label):
                return os.path.realpath(by_label)

        raise RuntimeError(
            'SSD device not found. Set logging.device_uuid or logging.device_label '
            'to match your external SSD.'
        )

    def mount_with_udiskctl(self, device_path):
        try:
            result = subprocess.run(
                ['udisksctl', 'mount', '-b', device_path],
                check=True,
                capture_output=True,
                text=True,
            )
            combined_output = f"{result.stdout}\n{result.stderr}".strip()
            match = re.search(r'at\s+(.+?)\.', combined_output)
            if match:
                mounted_path = match.group(1).strip()
                self.get_logger().info(f'SSD auto-mounted at {mounted_path}')
                return mounted_path
        except FileNotFoundError as exc:
            self.get_logger().error('udisksctl not found. Install udisks2 for auto-mount support.')
            raise RuntimeError('Auto-mount tool unavailable') from exc
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(
                f'udisksctl mount failed: {(exc.stderr or exc.stdout or "").strip()}'
            )

        if os.path.ismount(self.mount_point):
            self.get_logger().info(f'SSD mounted at {self.mount_point}')
            return self.mount_point

        return ''

    def get_local_timestamp(self):
        now = datetime.now()
        return now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    def detect_and_initialize_sensors(self):
        topics = dict(self.get_topic_names_and_types())

        if (
            self.topic_has_type(topics, self.gps_topic, 'sensor_msgs/msg/NavSatFix')
            and 'gps' not in self.detected_sensors
        ):
            self.init_sensor(
                'gps',
                'GPS',
                f'Timestamp({self.local_timezone})\tLatitude\tLongitude\tAltitude',
                'timestamp,latitude,longitude,altitude',
            )
            self.subscriptions_map['gps'] = self.create_subscription(
                NavSatFix,
                self.gps_topic,
                self.gps_callback,
                qos_profile_sensor_data,
            )
            self.detected_sensors.add('gps')

        if (
            self.topic_has_type(topics, self.imu_topic, 'sensor_msgs/msg/Imu')
            and 'imu' not in self.detected_sensors
        ):
            self.init_sensor(
                'imu',
                'IMU',
                f'Timestamp({self.local_timezone})\tAccX\tAccY\tAccZ',
                'timestamp,acc_x,acc_y,acc_z',
            )
            self.subscriptions_map['imu'] = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                qos_profile_sensor_data,
            )
            self.detected_sensors.add('imu')

        if (
            self.topic_has_type(topics, self.ctd_topic, 'std_msgs/msg/String')
            and 'ctd' not in self.detected_sensors
        ):
            self.init_sensor(
                'ctd',
                'CTD',
                (
                    f'Timestamp({self.local_timezone})\tDepth\tTemp\tCond\tSalinity'
                    '\tDensity\tSoundVel'
                ),
                'timestamp,depth,temp,cond,salinity,density,soundvel',
            )
            self.subscriptions_map['ctd'] = self.create_subscription(
                String,
                self.ctd_topic,
                self.ctd_callback,
                50,
            )
            self.detected_sensors.add('ctd')

        if (
            self.topic_has_type(topics, self.adcp_topic, 'std_msgs/msg/String')
            and 'adcp' not in self.detected_sensors
        ):
            self.init_sensor(
                'adcp',
                'ADCP',
                f'Timestamp({self.local_timezone})\tCurrentSpeed\tCurrentDirection\tWaterDepth',
                'timestamp,current_speed_ms,current_direction_deg,water_depth_m',
            )
            self.subscriptions_map['adcp'] = self.create_subscription(
                String,
                self.adcp_topic,
                self.adcp_callback,
                10,
            )
            self.detected_sensors.add('adcp')

        if (
            self.topic_has_type(topics, self.sbes_topic, 'std_msgs/msg/String')
            and 'sbes' not in self.detected_sensors
        ):
            self.init_sensor(
                'sbes',
                'SBES',
                f'Timestamp({self.local_timezone})\tDepth\tSeafloorConfidence\tSoundVelocity',
                'timestamp,depth_m,seafloor_confidence_percent,sound_velocity_ms',
            )
            self.subscriptions_map['sbes'] = self.create_subscription(
                String,
                self.sbes_topic,
                self.sbes_callback,
                10,
            )
            self.detected_sensors.add('sbes')

        if (
            self.topic_has_type(topics, self.battery_topic, 'sensor_msgs/msg/BatteryState')
            and 'battery' not in self.detected_sensors
        ):
            self.init_sensor(
                'battery',
                'Battery',
                f'Timestamp({self.local_timezone})\tVoltage\tCurrent\tPercentage',
                'timestamp,voltage,current,percentage',
            )
            self.subscriptions_map['battery'] = self.create_subscription(
                BatteryState,
                self.battery_topic,
                self.battery_callback,
                10,
            )
            self.detected_sensors.add('battery')

        if (
            self.topic_has_type(topics, self.telemetry_topic, 'std_msgs/msg/String')
            and 'telemetry' not in self.detected_sensors
        ):
            self.init_sensor(
                'telemetry',
                'Telemetry',
                f'Timestamp({self.local_timezone})\tRawJSON',
                'timestamp,payload_json',
            )
            self.subscriptions_map['telemetry'] = self.create_subscription(
                String,
                self.telemetry_topic,
                self.telemetry_callback,
                10,
            )
            self.detected_sensors.add('telemetry')

    def topic_has_type(self, topics, topic_name, ros_type):
        return topic_name in topics and ros_type in topics[topic_name]

    def init_sensor(self, key, name, log_columns, csv_columns):
        log_path = os.path.join(self.base_path, f'{key}.log')
        csv_path = os.path.join(self.base_path, f'{key}.csv')

        log_file = open(log_path, 'w')
        csv_file = open(csv_path, 'w')

        log_file.write(
            '[System]\n'
            'Platform=SEANO USV\n'
            f"Start_Time={self.start_time_obj.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f'Timezone={self.local_timezone}\n\n'
            '[Sensor]\n'
            f'Name={name}\n\n'
            '[Columns]\n'
            f'{log_columns}\n\n'
            '[Data]\n'
        )

        csv_file.write(csv_columns + '\n')

        self.files[key] = log_file
        self.csv_files[key] = csv_file
        self.buffers[key] = []
        self.sample_count[key] = 0

        self.get_logger().info(f'{name} detected')

    def gps_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = f'{t}\t{msg.latitude}\t{msg.longitude}\t{msg.altitude}\n'
        csv_line = f'{t},{msg.latitude},{msg.longitude},{msg.altitude}\n'
        self.buffers['gps'].append((log_line, csv_line))
        self.sample_count['gps'] += 1

    def imu_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = (
            f'{t}\t'
            f'{msg.linear_acceleration.x}\t'
            f'{msg.linear_acceleration.y}\t'
            f'{msg.linear_acceleration.z}\n'
        )
        csv_line = (
            f'{t},'
            f'{msg.linear_acceleration.x},'
            f'{msg.linear_acceleration.y},'
            f'{msg.linear_acceleration.z}\n'
        )
        self.buffers['imu'].append((log_line, csv_line))
        self.sample_count['imu'] += 1

    def ctd_callback(self, msg):
        t = self.get_local_timestamp()
        payload = self.parse_json_payload(msg.data)
        depth = payload.get('depth_m', '')
        temp = payload.get('temperature_c', '')
        cond = payload.get('conductivity_ms_cm', '')
        salinity = payload.get('salinity_psu', '')
        density = payload.get('density_kg_m3', '')
        sound_vel = payload.get('sound_velocity_ms', '')
        log_line = f'{t}\t{depth}\t{temp}\t{cond}\t{salinity}\t{density}\t{sound_vel}\n'
        csv_line = f'{t},{depth},{temp},{cond},{salinity},{density},{sound_vel}\n'
        self.buffers['ctd'].append((log_line, csv_line))
        self.sample_count['ctd'] += 1

    def adcp_callback(self, msg):
        t = self.get_local_timestamp()
        payload = self.parse_json_payload(msg.data)
        speed = payload.get('current_speed_ms', '')
        direction = payload.get('current_direction_deg', '')
        depth = payload.get('water_depth_m', '')
        log_line = f'{t}\t{speed}\t{direction}\t{depth}\n'
        csv_line = f'{t},{speed},{direction},{depth}\n'
        self.buffers['adcp'].append((log_line, csv_line))
        self.sample_count['adcp'] += 1

    def sbes_callback(self, msg):
        t = self.get_local_timestamp()
        payload = self.parse_json_payload(msg.data)
        depth = payload.get('depth_m', '')
        confidence = payload.get('seafloor_confidence_percent', '')
        sound_vel = payload.get('sound_velocity_ms', '')
        log_line = f'{t}\t{depth}\t{confidence}\t{sound_vel}\n'
        csv_line = f'{t},{depth},{confidence},{sound_vel}\n'
        self.buffers['sbes'].append((log_line, csv_line))
        self.sample_count['sbes'] += 1

    def battery_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = f'{t}\t{msg.voltage}\t{msg.current}\t{msg.percentage}\n'
        csv_line = f'{t},{msg.voltage},{msg.current},{msg.percentage}\n'
        self.buffers['battery'].append((log_line, csv_line))
        self.sample_count['battery'] += 1

    def telemetry_callback(self, msg):
        t = self.get_local_timestamp()
        safe_payload = msg.data.replace('\n', ' ')
        log_line = f'{t}\t{safe_payload}\n'
        csv_line = f'{t},{json.dumps(safe_payload)}\n'
        self.buffers['telemetry'].append((log_line, csv_line))
        self.sample_count['telemetry'] += 1

    def parse_json_payload(self, payload_text):
        try:
            parsed = json.loads(payload_text)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass
        return {}

    def flush_buffers(self):
        for key in self.buffers:
            if self.buffers[key]:
                for log_line, csv_line in self.buffers[key]:
                    self.files[key].write(log_line)
                    self.csv_files[key].write(csv_line)

                self.files[key].flush()
                self.csv_files[key].flush()
                self.buffers[key].clear()

        self.get_logger().info('Buffers flushed')

    def destroy_node(self):
        self.flush_buffers()

        for key in self.files:
            self.files[key].close()
            self.csv_files[key].close()

        os.sync()
        super().destroy_node()


def main():
    rclpy.init()
    node = SeanoLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
