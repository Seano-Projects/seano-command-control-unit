#!/usr/bin/env python3
"""
Launch file: seano_vision (camera + YOLO detector) + rtmp_streamer
- Kamera dari seano_vision camera_node (camera_hp)
- Deteksi YOLO dari seano_vision detector_node
- Stream RTMP dari seano_cam rtmp_streamer
- Output stream: /camera/image_annotated (frame + bounding box)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_PARAMS = os.path.join(
    os.path.expanduser('~'),
    'Seano_ws', 'src', 'seano_startup', 'config', 'system.yaml'
)


def launch_nodes(context, *args, **kwargs):
    params_file = os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)
    )

    pkg_vision = get_package_share_directory('seano_vision')
    camera_cfg = os.path.join(pkg_vision, 'config', 'camera_usb.yaml')
    model_path = os.path.join(pkg_vision, 'models', 'yolov8n.pt')

    camera_node = Node(
        package='seano_vision',
        executable='camera_node',
        name='camera_hp',
        output='screen',
        emulate_tty=True,
        parameters=[camera_cfg, {
            'source': 'device',
            'backend': 'opencv',
            'device_index': 0,
            'device_path': '/dev/video0',
            'device_fourcc': 'MJPG',
            'device_width': 320,
            'device_height': 240,
            'device_fps': 30,
            'max_fps': 30.0,
            'max_age_ms': 80,
            'grab_skip': 0,
            'publish_in_reader': False,
            'output_encoding': 'bgr8',
            'swap_rb': False,
            'publish_best_effort': True,
            'publish_reliable': False,
            'topic_best_effort': '/camera/image_raw',
            'topic_reliable': '/camera/image_raw_reliable',
            'reconnect_sec': 0.5,
            'log_stats_sec': 5.0,
        }],
    )

    detector_node = Node(
        package='seano_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'sub_image': '/camera/image_raw',
            'sub_reliability': 'best_effort',
            'pub_det': '/camera/detections',
            'pub_det_reliability': 'best_effort',
            'publish_annotated': True,        # aktifkan frame + bounding box
            'pub_annotated': '/camera/image_annotated',
            'publish_detections': True,
            'publish_empty_detections': True,
            'model_path': model_path,
            'device': 'cuda',
            'imgsz': 320,
            'conf': 0.25,
            'iou': 0.45,
            'class_ids': 'ALL',
            'max_det': 50,
            'max_fps': 15.0,
            'qos_depth': 1,
            'warmup': False,
            'stats_period': 5.0,
        }],
    )

    rtmp_streamer = Node(
        package='seano_cam',
        executable='rtmp_streamer',
        name='rtmp_streamer',
        output='screen',
        parameters=[params_file, {
            'camera.topic': '/camera/image_annotated',
            'rtmp.fps': 15,
        }],
    )

    # Delay detector 2s biar kamera publish dulu sebelum detector subscribe
    delayed_detector = TimerAction(period=2.0, actions=[detector_node])
    # Delay rtmp 4s biar detector sudah siap dan annotated topic sudah ada
    delayed_rtmp = TimerAction(period=4.0, actions=[rtmp_streamer])

    return [camera_node, delayed_detector, delayed_rtmp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=DEFAULT_PARAMS,
            description='Path ke file parameter YAML (untuk rtmp config)'
        ),
        OpaqueFunction(function=launch_nodes),
    ])
