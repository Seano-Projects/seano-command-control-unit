import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, VfrHud, RadioStatus
from sensor_msgs.msg import NavSatFix, Imu
import paho.mqtt.client as mqtt
import ssl
import json
import glob
from datetime import datetime, timezone


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Declare parameters
        self.declare_parameter('system.mode', 'unknown')
        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.battery_topic', '')

        self.system_mode = self.get_parameter('system.mode').value
        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.mqtt_battery_topic = self.get_parameter('mqtt.battery_topic').value
        if not self.mqtt_battery_topic:
            self.mqtt_battery_topic = f"seano/{self.vehicle_id}/Battery"

        # Data from MAVROS
        self.armed = False
        self.mode = "UNKNOWN"
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Additional telemetry data
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_percentage = 0
        self.speed = 0.0
        self.rssi = 0
        self.gps_ok = False
        self.system_status = "UNKNOWN"

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

        # Subscribe to radio status for RSSI
        self.radio_sub = self.create_subscription(
            RadioStatus,
            '/mavros/radio_status',
            self.radio_callback,
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
        if self.mqtt_username:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.mqtt_client.tls_insecure_set(True)
        self.mqtt_client.on_connect = self._mqtt_on_connect
        self.mqtt_client.on_message = self._mqtt_on_message
        self.mqtt_client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=60)
        self.mqtt_client.loop_start()

        self.get_logger().info(f'Telemetry node started')
        self.get_logger().info(f'Vehicle ID   : {self.vehicle_id}')
        self.get_logger().info(f'System Mode : {self.system_mode}')
        self.get_logger().info(f'Battery MQTT Topic : {self.mqtt_battery_topic}')

    def state_callback(self, msg):
        """Callback for MAVROS state (armed, mode)"""
        self.armed = msg.armed
        self.mode = msg.mode
        
        # Determine system status based on connection and mode
        if msg.connected:
            self.system_status = "OK"
        else:
            self.system_status = "DISCONNECTED"

    def gps_callback(self, msg):
        """Callback for GPS position"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        
        # Check GPS fix status
        # NavSatFix status: 0=no fix, 1=fix, 2=SBAS fix, 3=GBAS fix
        self.gps_ok = (msg.status.status >= 0)

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.mqtt_battery_topic, qos=1)
            self.get_logger().info(f'Subscribed battery MQTT: {self.mqtt_battery_topic}')
        else:
            self.get_logger().warn(f'Battery MQTT connect failed rc={rc}')

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

        # Current can be missing; force default to 0.0 when unavailable.
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

    def vfr_callback(self, msg):
        """Callback for VFR HUD (speed)"""
        self.speed = msg.groundspeed

    def radio_callback(self, msg):
        """Callback for radio status (RSSI)"""
        self.rssi = msg.rssi

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
        telemetry_data = {
            "vehicle_code": self.vehicle_id,
            "usv_timestamp": datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z',
            "battery_voltage": round(self.battery_voltage, 1),
            "battery_current": round(self.battery_current, 1),
            "battery_percentage": self.battery_percentage,
            "rssi": self.rssi,
            "latitude": round(self.latitude, 6),
            "longitude": round(self.longitude, 6),
            "altitude": round(self.altitude, 1),
            "heading": round(self.heading, 1),
            "armed": self.armed,
            "gps_ok": self.gps_ok,
            "system_status": self.system_status,
            "mode": self.mode,
            "speed": round(self.speed, 1),
            "roll": round(self.roll, 1),
            "pitch": round(self.pitch, 1),
            "yaw": round(self.yaw, 1),
            "temperature_system": f"{round(self._read_jetson_temperature(), 1)}"
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
