from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Path ke config file
    config_file = os.path.join(
        get_package_share_directory('seano_mqtt_bridge'),
        'config',
        'mqtt_status.yaml'
    )

    # Declare launch arguments
    vehicle_code_arg = DeclareLaunchArgument(
        'vehicle_code',
        default_value='USV-001',
        description='Vehicle identification code'
    )

    # MQTT Status Node
    mqtt_status_node = Node(
        package='seano_mqtt_bridge',
        executable='mqtt_status_node',
        name='mqtt_status',
        output='screen',
        parameters=[config_file, {
            'vehicle_code': LaunchConfiguration('vehicle_code')
        }]
    )

    return LaunchDescription([
        vehicle_code_arg,
        mqtt_status_node
    ])
