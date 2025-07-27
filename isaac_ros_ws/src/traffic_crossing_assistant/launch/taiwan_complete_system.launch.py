#!/usr/bin/env python3
"""
Taiwan Traffic Safety System - Complete Integration Launch
Launches all components for 3-priority hierarchy system
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
    pkg_crossing = get_package_share_directory('traffic_crossing_assistant')
    
    # Configuration files
    taiwan_config = os.path.join(pkg_crossing, 'config', 'taiwan_safety_config.yaml')
    audio_config = os.path.join(pkg_crossing, 'config', 'audio_config.yaml')
    
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/models/yolov8/17class/taiwan.onnx',
        description='Path to Taiwan 17-class ONNX model')
    
    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio', default_value='true',
        description='Enable audio feedback system')
    
    # Launch YOLOv8 Detection System (separate package)
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yolov8_detection'), 
                        'launch', 'yolov8_realsense.launch.py')
        ]),
        launch_arguments=[
            ('model_path', LaunchConfiguration('model_path')),
        ].items()
    )
    
    # Priority 1: Vehicle Movement Analyzer
    vehicle_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='vehicle_movement_analyzer',
        name='vehicle_movement_analyzer',
        parameters=[taiwan_config],
        output='screen'
    )
    
    # Priority 2: Taiwan Crossing Analyzer (Your 85.5% mAP50 Innovation)
    crossing_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='taiwan_crossing_analyzer',
        name='taiwan_crossing_analyzer',
        parameters=[taiwan_config],
        output='screen'
    )
    
    # Priority 3: Traffic Light Analyzer
    traffic_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='traffic_light_analyzer',
        name='traffic_light_analyzer',
        parameters=[taiwan_config],
        output='screen'
    )
    
    # Main Decision Engine
    decision_engine = Node(
        package='traffic_crossing_assistant',
        executable='decision_engine',
        name='decision_engine',
        parameters=[taiwan_config],
        output='screen'
    )
    
    # Audio Feedback System
    audio_system = Node(
        package='traffic_crossing_assistant',
        executable='audio_feedback_system',
        name='audio_feedback_system',
        parameters=[audio_config],
        condition=IfCondition(LaunchConfiguration('enable_audio')),
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        enable_audio_arg,
        
        # Detection system (separate package)
        yolov8_launch,
        
        # Priority hierarchy components
        vehicle_analyzer,      # Priority 1
        crossing_analyzer,     # Priority 2 (Your 85.5% mAP50 innovation)
        traffic_analyzer,      # Priority 3
        
        # Decision coordination
        decision_engine,
        
        # User interface
        audio_system,
        rviz_node
    ])
