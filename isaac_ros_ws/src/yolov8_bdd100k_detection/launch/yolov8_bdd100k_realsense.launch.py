#!/usr/bin/env python3
"""
Taiwan Traffic Safety System Launch File - FIXED
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_yolov8 = get_package_share_directory('yolov8_bdd100k_detection')
    
    # Configuration files with FIXED paths
    camera_config = os.path.join(pkg_yolov8, 'config', 'camera_params.yaml')
    detection_config = os.path.join(pkg_yolov8, 'config', 'detection_params.yaml')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization')
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/models/yolov8/17class/taiwan.onnx',
        description='Path to Taiwan 17-class ONNX model')
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5',
        description='Detection confidence threshold')

    # RealSense camera launch
    try:
        pkg_realsense = get_package_share_directory('realsense2_camera')
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
            ]),
            launch_arguments=[
                ('camera_name', 'camera'),
                ('camera_namespace', ''),
                ('enable_color', 'true'),
                ('enable_depth', 'true'),
                ('color_width', '640'),
                ('color_height', '480'),
                ('color_fps', '30'),
                ('depth_width', '640'),
                ('depth_height', '480'),
                ('depth_fps', '30'),
                ('enable_infra1', 'false'),
                ('enable_infra2', 'false'),
            ].items()
        )
    except:
        # Fallback if realsense package not found
        realsense_launch = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_color_optical_frame'
            }],
            remappings=[('/image_raw', '/camera/color/image_raw')]
        )
    
    # FIXED: Taiwan YOLOv8 Detection Node
    yolov8_node = Node(
        package='yolov8_bdd100k_detection',
        executable='yolov8_camera_node',  # FIXED: Use entry point name
        name='yolov8_taiwan_detector',
        parameters=[
            camera_config,
            {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }
        ],
        remappings=[
            ('~/image_raw', '/camera/color/image_raw'),
            ('~/detections', '/camera/detections')
        ],
        output='screen'
    )
    
    # Detection Processor Node
    detection_processor_node = Node(
        package='yolov8_bdd100k_detection',
        executable='detection_processor',  # FIXED: Use entry point name
        name='taiwan_detection_processor',
        parameters=[detection_config],
        remappings=[
            ('~/detections', '/camera/detections'),
            ('~/filtered', '/detections/filtered'),
            ('~/poses', '/obstacles/poses'),
            ('~/grid', '/obstacles/grid')
        ],
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        model_path_arg,
        confidence_arg,
        realsense_launch,
        yolov8_node,
        detection_processor_node,
        rviz_node
    ])
