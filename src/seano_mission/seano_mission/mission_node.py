import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from mavros_msgs.msg import State, WaypointReached, WaypointList, Waypoint, HomePosition
from mavros_msgs.srv import WaypointPull
from geographic_msgs.msg import GeoPoint
import json
import math


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('vehicle.id', 'USV-001')
        self.vehicle_id = self.get_parameter('vehicle.id').value

        # Mission state
        self.current_waypoint_seq = 0
        self.total_waypoints = 0
        self.waypoints = []
        self.mission_active = False
        self.mavros_connected = False
        self.armed = False
        self.mode = 'UNKNOWN'

        # Home position
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0

        # Last reached waypoint
        self.last_reached_seq = -1

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe MAVROS state
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Subscribe waypoint reached event
        self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            sensor_qos
        )

        # Subscribe waypoint list (current mission loaded)
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoint_list_callback,
            sensor_qos
        )

        # Subscribe home position
        self.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self.home_callback,
            sensor_qos
        )

        # Publisher — mission status JSON
        self.pub_status = self.create_publisher(String, 'mission/status', 10)

        # Publisher — waypoint reached event JSON
        self.pub_reached = self.create_publisher(String, 'mission/waypoint_reached', 10)

        # Timer publish status tiap 2 detik
        self.create_timer(2.0, self.publish_status)

        self.get_logger().info('Mission Node started')
        self.get_logger().info(f'Vehicle ID: {self.vehicle_id}')

    def state_callback(self, msg: State):
        self.mavros_connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode
        self.mission_active = msg.armed and msg.mode in ('AUTO', 'GUIDED')

    def waypoint_reached_callback(self, msg: WaypointReached):
        self.last_reached_seq = msg.wp_seq
        self.get_logger().info(f'Waypoint reached: #{msg.wp_seq}')

        # Publish event
        data = {
            'vehicle_id': self.vehicle_id,
            'event': 'waypoint_reached',
            'wp_seq': msg.wp_seq,
            'total': self.total_waypoints,
            'remaining': max(0, self.total_waypoints - msg.wp_seq - 1),
        }
        msg_out = String()
        msg_out.data = json.dumps(data)
        self.pub_reached.publish(msg_out)

    def waypoint_list_callback(self, msg: WaypointList):
        self.current_waypoint_seq = msg.current_seq
        self.total_waypoints = len(msg.waypoints)
        self.waypoints = []

        for i, wp in enumerate(msg.waypoints):
            self.waypoints.append({
                'seq': i,
                'frame': wp.frame,
                'command': wp.command,
                'is_current': wp.is_current,
                'autocontinue': wp.autocontinue,
                'lat': round(wp.x_lat, 6),
                'lon': round(wp.y_long, 6),
                'alt': round(wp.z_alt, 2),
                'param1': wp.param1,  # e.g. acceptance radius for NAV_WAYPOINT
            })

    def home_callback(self, msg: HomePosition):
        self.home_lat = round(msg.geo.latitude, 6)
        self.home_lon = round(msg.geo.longitude, 6)
        self.home_alt = round(msg.geo.altitude, 2)

    def _distance_to_wp(self, wp_lat, wp_lon, cur_lat, cur_lon):
        """Hitung jarak haversine dari posisi sekarang ke waypoint (meter)"""
        if cur_lat == 0.0 and cur_lon == 0.0:
            return None
        R = 6371000
        phi1 = math.radians(cur_lat)
        phi2 = math.radians(wp_lat)
        dphi = math.radians(wp_lat - cur_lat)
        dlam = math.radians(wp_lon - cur_lon)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)), 1)

    def publish_status(self):
        current_wp = None
        for wp in self.waypoints:
            if wp['seq'] == self.current_waypoint_seq:
                current_wp = wp
                break

        data = {
            'vehicle_id': self.vehicle_id,
            'connected': self.mavros_connected,
            'armed': self.armed,
            'mode': self.mode,
            'mission_active': self.mission_active,
            'current_wp_seq': self.current_waypoint_seq,
            'total_waypoints': self.total_waypoints,
            'last_reached_seq': self.last_reached_seq,
            'remaining_waypoints': max(0, self.total_waypoints - self.current_waypoint_seq - 1),
            'home': {
                'lat': self.home_lat,
                'lon': self.home_lon,
                'alt': self.home_alt,
            },
            'current_waypoint': current_wp,
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
