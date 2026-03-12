import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, VfrHud, RadioStatus
from sensor_msgs.msg import NavSatFix, Imu, Temperature, BatteryState
import json


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Declare parameters
        self.declare_parameter('system.mode', 'unknown')
        self.declare_parameter('vehicle.id', 'unknown')

        self.system_mode = self.get_parameter('system.mode').value
        self.vehicle_id = self.get_parameter('vehicle.id').value

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
        self.temperature_system = 0.0
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
            10
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

        # Subscribe to battery status
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            10
        )

        # Subscribe to VFR HUD for speed
        self.vfr_sub = self.create_subscription(
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_callback,
            10
        )

        # Subscribe to radio status for RSSI
        self.radio_sub = self.create_subscription(
            RadioStatus,
            '/mavros/radio_status',
            self.radio_callback,
            10
        )

        # Subscribe to temperature (if available)
        self.temp_sub = self.create_subscription(
            Temperature,
            '/mavros/temperature',
            self.temperature_callback,
            10
        )

        # Publisher for telemetry (JSON format)
        self.publisher_ = self.create_publisher(
            String,
            'telemetry',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_telemetry)

        self.get_logger().info(f'Telemetry node started')
        self.get_logger().info(f'Vehicle ID   : {self.vehicle_id}')
        self.get_logger().info(f'System Mode : {self.system_mode}')

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

    def battery_callback(self, msg):
        """Callback for battery status (sensor_msgs/BatteryState)"""
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_percentage = int(msg.percentage) if msg.percentage >= 0 else 0

    def vfr_callback(self, msg):
        """Callback for VFR HUD (speed)"""
        self.speed = msg.groundspeed

    def radio_callback(self, msg):
        """Callback for radio status (RSSI)"""
        self.rssi = msg.rssi

    def temperature_callback(self, msg):
        """Callback for temperature"""
        self.temperature_system = msg.temperature

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
            "temperature_system": round(self.temperature_system, 1)
        }
        
        msg = String()
        msg.data = json.dumps(telemetry_data)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
