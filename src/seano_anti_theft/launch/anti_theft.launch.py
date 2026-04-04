from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    param_file = os.path.join(
        os.getenv('HOME'),
        'Seano_ws/src/seano_startup/config/system.yaml'
    )

    return LaunchDescription([
        Node(
            package='seano_anti_theft',
            executable='anti_theft_node',
            name='anti_theft_node',
            output='screen',
            parameters=[param_file]
        )
    ])