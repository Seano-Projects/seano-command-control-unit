from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_param_file = os.path.join(
        os.getenv('HOME'),
        'Seano_ws/src/seano_startup/config/system.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_param_file,
            description='Path ke file parameter YAML'
        ),
        Node(
            package='seano_network_monitor',
            executable='network_monitor_node',
            name='network_monitor',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
    ])
