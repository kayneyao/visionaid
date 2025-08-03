#!/usr/bin/env python3
"""
Taiwan YOLOv8 Detection - FIXED Launch File
Corrected topic routing and launch arguments
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Model path argument
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class/weights/balanced.onnx',
        description='Path to Balanced Augmented 11-class ONNX model')
    
    # RealSense camera launch with optimized settings
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true', 
            'enable_depth': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480',
            'color_fps': '30',
            'depth_fps': '30',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'clip_distance': '2.0'
        }.items()
    )
    
    # YOLOv8 Taiwan detection node with FIXED topic routing
    yolov8_node = Node(
        package='yolov8_detection',
        executable='yolov8_camera_node',
        name='traffic_detector_11class',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': 0.3,  # Lower global threshold since we use class-specific thresholds
            'nms_threshold': 0.4,
            'max_detections': 50,
            'inference_rate': 30.0,
            'publish_detection_images': True,
        }],
        remappings=[
            # FIXED: Direct topic mapping without remapping conflicts
            ('image_raw', '/camera/camera/color/image_raw'),
            ('detections', '/camera/detections'),
            ('detections/visualization', '/camera/detections/visualization')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        model_path_arg,
        realsense_launch,
        yolov8_node
    ])
