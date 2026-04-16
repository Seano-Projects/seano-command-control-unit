from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _launch_setup(context, *args, **kwargs):
    params_file = os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)
    )

    if not os.path.isfile(params_file):
        raise FileNotFoundError(f"Params file tidak ditemukan: {params_file}")

    return [
        Node(
            package='seano_command',
            executable='command_node',
            name='command',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='seano_command',
            executable='waypoint_node',
            name='waypoint',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='seano_command',
            executable='thruster_node',
            name='thruster',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='seano_command',
            executable='param_node',
            name='param',
            parameters=[params_file],
            output='screen'
        ),
    ]


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
        OpaqueFunction(function=_launch_setup),
    ])
