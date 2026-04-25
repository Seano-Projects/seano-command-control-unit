from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os

import yaml


def _load_system_mode(param_file: str) -> str:
    try:
        with open(param_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        params = data.get('/**', {}).get('ros__parameters', {})
        mode = params.get('system', {}).get('mode', 'field_test')
        return str(mode)
    except Exception:
        return 'field_test'


def _vision_profile_defaults(system_mode: str) -> dict:
    mode = system_mode.strip().lower()

    # Field testing on edge hardware defaults to lighter vision settings.
    if mode == 'field_test':
        return {
            'enable_vision_stack': 'false',
            'enable_vision_actuation': 'false',
            'enable_rtmp_stream': 'false',
            'vision_det_imgsz': '320',
            'vision_det_max_fps': '6.0',
            'vision_det_conf': '0.30',
            'vision_camera_launch': 'phase2_camera_usb_test.launch.py',
        }

    return {
        'enable_vision_stack': 'false',
        'enable_vision_actuation': 'false',
        'enable_rtmp_stream': 'false',
        'vision_det_imgsz': '416',
        'vision_det_max_fps': '10.0',
        'vision_det_conf': '0.25',
        'vision_camera_launch': 'phase2_camera_usb_test.launch.py',
    }

def generate_launch_description():
    param_file = os.path.join(
        os.getenv('HOME'),
        'Seano_ws/src/seano_startup/config/system.yaml'
    )

    system_mode = _load_system_mode(param_file)
    profile_defaults = _vision_profile_defaults(system_mode)

    enable_vision_stack = LaunchConfiguration('enable_vision_stack')
    enable_vision_actuation = LaunchConfiguration('enable_vision_actuation')
    enable_rtmp_stream = LaunchConfiguration('enable_rtmp_stream')
    vision_det_imgsz = LaunchConfiguration('vision_det_imgsz')
    vision_det_max_fps = LaunchConfiguration('vision_det_max_fps')
    vision_det_conf = LaunchConfiguration('vision_det_conf')
    vision_camera_launch = LaunchConfiguration('vision_camera_launch')
    enable_failsafe = LaunchConfiguration('enable_failsafe')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    vision_full_ca_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('HOME'),
                'Seano_ws/src/seano_vision/launch/demo_full_ca.launch.py'
            )
        ),
        condition=IfCondition(enable_vision_stack),
        launch_arguments={
            'camera_launch': vision_camera_launch,
            'use_camera': 'true',
            'use_detector': 'false',
            'use_waterline': 'false',
            'use_fp_guard': 'false',
            'use_fusion': 'false',
            'use_vq': 'false',
            'use_freeze': 'false',
            'use_risk': 'false',
            'use_watchdog': 'false',
            'image_topic': '/seano/camera/image_raw_reliable',
            'annotated_topic': '/camera/image_annotated',
            'detections_raw_topic': '/camera/detections',
            'det_imgsz': vision_det_imgsz,
            'det_max_fps': vision_det_max_fps,
            'det_conf': vision_det_conf,
            'use_ca_viewer': 'false',
            'use_wl_viewer': 'false',
        }.items(),
    )

    vision_actuation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('HOME'),
                'Seano_ws/src/seano_vision/launch/run_auto_stack.launch.py'
            )
        ),
        condition=IfCondition(enable_vision_actuation),
    )

    # ArduPilot broadcasts all params on connect; MAVROS logs each unsolicited
    # value at INFO level. Suppress mavros.param to WARN via env var so it
    # takes effect before mavros_node even starts.
    suppress_mavros_param_log = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY_MAP',
        'mavros.param:WARN,mavros.time:WARN'
    )

    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/mavros/launch',
                'apm.launch'
            )
        ),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_vision_stack', default_value=profile_defaults['enable_vision_stack']),
        DeclareLaunchArgument(
            'enable_vision_actuation',
            default_value=profile_defaults['enable_vision_actuation'],
        ),
        DeclareLaunchArgument('enable_rtmp_stream', default_value=profile_defaults['enable_rtmp_stream']),
        DeclareLaunchArgument('vision_det_imgsz', default_value=profile_defaults['vision_det_imgsz']),
        DeclareLaunchArgument('vision_det_max_fps', default_value=profile_defaults['vision_det_max_fps']),
        DeclareLaunchArgument('vision_det_conf', default_value=profile_defaults['vision_det_conf']),
        DeclareLaunchArgument('vision_camera_launch', default_value=profile_defaults['vision_camera_launch']),
        DeclareLaunchArgument('enable_failsafe', default_value=os.getenv('SEANO_ENABLE_FAILSAFE', 'false')),
        DeclareLaunchArgument('fcu_url', default_value=os.getenv('SEANO_FCU_URL', '/dev/ttyACM0:115200')),
        DeclareLaunchArgument('gcs_url', default_value=os.getenv('SEANO_GCS_URL', 'udp://@100.124.223.119:14550')),
        suppress_mavros_param_log,
        mavros_launch,
        vision_full_ca_launch,
        vision_actuation_launch,
        
        GroupAction([
            PushRosNamespace('usv'),

            Node(
                package='seano_startup',
                executable='mqtt_status_node',
                name='mqtt_status',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_telemetry',
                executable='telemetry_node',
                name='telemetry',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_mission',
                executable='mission_node',
                name='mission',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_startup',
                executable='mqtt_bridge_node',
                name='mqtt_bridge',
                parameters=[param_file],
                respawn=True,
                respawn_delay=5.0,
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
                package='seano_command',
                executable='waypoint_node',
                name='waypoint',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_command',
                executable='thruster_node',
                name='thruster',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_anti_theft',
                executable='anti_theft_node',
                name='anti_theft',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_oceanography',
                executable='ctd_sensor_node',
                name='ctd_sensor',
                parameters=[param_file],
                output='screen'
            ),

            Node(
                package='seano_failsafe',
                executable='seano_battery',
                name='battery',
                parameters=[param_file],
                condition=IfCondition(enable_failsafe),
                output='screen'
            ),

            Node(
                package='seano_failsafe',
                executable='seano_communication_monitor',
                name='communication_monitor',
                parameters=[param_file],
                condition=IfCondition(enable_failsafe),
                output='screen'
            ),

            Node(
                package='seano_failsafe',
                executable='seano_failsafe',
                name='failsafe',
                parameters=[param_file],
                condition=IfCondition(enable_failsafe),
                output='screen'
            ),

            Node(
                package='seano_vision',
                executable='rtmp_streamer',
                name='rtmp_streamer',
                parameters=[param_file],
                condition=IfCondition(enable_rtmp_stream),
                output='screen'
            ),
        ])
    ])
