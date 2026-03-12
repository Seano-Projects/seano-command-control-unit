from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os

def generate_launch_description():

    param_file = os.path.join(
        os.getenv('HOME'),
        'Seano_ws/src/seano_startup/config/system.yaml'
    )

    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/mavros/launch',
                'apm.launch'
            )
        ),
        launch_arguments={
            'fcu_url': '/dev/ttyACM0:115200',
            'gcs_url': 'udp://@0.0.0.0:14550'
        }.items()
    )

    return LaunchDescription([
        mavros_launch,
        
        GroupAction([
            PushRosNamespace('usv'),

            Node(
                package='seano_telemetry',
                executable='telemetry_node',
                name='telemetry',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_logging',
                executable='telemetry_logger_node',
                name='telemetry_logger',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_mqtt_bridge',
                executable='mqtt_bridge_node',
                name='mqtt_bridge',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_mqtt_bridge',
                executable='mqtt_status_node',
                name='mqtt_status',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_command',
                executable='command_node',
                name='command',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_communication',
                executable='communication_node',
                name='communication',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_failsafe',
                executable='seano_battery',
                name='battery',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_failsafe',
                executable='seano_communication_monitor',
                name='communication_monitor',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_failsafe',
                executable='seano_failsafe',
                name='failsafe',
                parameters=[param_file],
                output='screen'
            ),
        ])
    ])
