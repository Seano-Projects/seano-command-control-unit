from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


DEFAULT_PARAMS = os.path.join(
    os.path.expanduser('~'),
    'Seano_ws', 'src', 'seano_startup', 'config', 'system.yaml'
)


def launch_nodes(context, *args, **kwargs):
    params_file = os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)
    )
    return [
        Node(
            package='seano_cam',
            executable='camera_node',
            name='camera_node',
            parameters=[params_file, {'camera.enable_display': False}],
            output='screen',
        ),
        Node(
            package='seano_cam',
            executable='rtmp_streamer',
            name='rtmp_streamer',
            parameters=[params_file],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=DEFAULT_PARAMS,
            description='Path ke file parameter YAML'
        ),
        OpaqueFunction(function=launch_nodes),
    ])

