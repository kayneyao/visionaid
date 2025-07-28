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
        'enable_audio', 
        default_value='true',
        description='Enable audio feedback system')
    
    # FIXED: Launch YOLOv8 Detection System with correct launch_arguments syntax
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yolov8_detection'), 
                        'launch', 'yolov8_realsense.launch.py')
        ]),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
        }.items()  # FIXED: Use dictionary with .items() instead of list
    )
    
    # Priority 1: Vehicle Movement Analyzer
    vehicle_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='vehicle_movement_analyzer',
        name='vehicle_movement_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('detections', '/camera/detections'),
            ('vehicle_trajectories', '/traffic_safety/vehicle_trajectories'),
            ('collision_risk', '/traffic_safety/collision_risk')
        ],
        output='screen'
    )
    
    # Priority 2: Taiwan Crossing Analyzer (Your 96.35% pedestrian detection innovation)
    crossing_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='taiwan_crossing_analyzer',
        name='taiwan_crossing_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('detections', '/camera/detections'),
            ('taiwan_analysis', '/traffic_safety/taiwan_analysis'),
            ('crossing_recommendation', '/traffic_safety/crossing_recommendation')
        ],
        output='screen'
    )
    
    # Priority 3: Traffic Light Analyzer
    traffic_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='traffic_light_analyzer',
        name='traffic_light_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('detections', '/camera/detections'),
            ('traffic_light_status', '/traffic_safety/traffic_light_status')
        ],
        output='screen'
    )
    
    # Main Decision Engine
    decision_engine = Node(
        package='traffic_crossing_assistant',
        executable='decision_engine',
        name='decision_engine',
        parameters=[taiwan_config],
        remappings=[
            ('vehicle_trajectories', '/traffic_safety/vehicle_trajectories'),
            ('taiwan_analysis', '/traffic_safety/taiwan_analysis'),
            ('traffic_light_status', '/traffic_safety/traffic_light_status'),
            ('final_recommendation', '/traffic_safety/final_recommendation'),
            ('crossing_safe', '/traffic_safety/crossing_safe')
        ],
        output='screen'
    )
    
    # Enhanced Audio Feedback System
    audio_system = Node(
        package='traffic_crossing_assistant',
        executable='enhanced_audio_system',
        name='enhanced_audio_system',
        parameters=[audio_config],
        condition=IfCondition(LaunchConfiguration('enable_audio')),
        remappings=[
            ('final_recommendation', '/traffic_safety/final_recommendation'),
            ('audio_message', '/traffic_safety/audio_message')
        ],
        output='screen'
    )
    
    # Ego Motion Compensator for improved tracking
    ego_motion = Node(
        package='traffic_crossing_assistant',
        executable='ego_motion_compensator',
        name='ego_motion_compensator',
        parameters=[taiwan_config],
        remappings=[
            ('camera_info', '/camera/camera/color/camera_info'),
            ('depth_image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('ego_motion', '/traffic_safety/ego_motion')
        ],
        output='screen'
    )
    
    # RViz for visualization with Taiwan traffic overlay
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_crossing, 'config', 'taiwan_traffic_viz.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        enable_audio_arg,
        
        # Detection system (your excellent 96.35% pedestrian, 85.91% vehicle performance)
        yolov8_launch,
        
        # Priority hierarchy components
        vehicle_analyzer,      # Priority 1: Vehicle safety (85.91% car detection)
        crossing_analyzer,     # Priority 2: Taiwan crossing innovation  
        traffic_analyzer,      # Priority 3: Traffic light analysis
        
        # Motion compensation for enhanced tracking
        ego_motion,
        
        # Decision coordination
        decision_engine,
        
        # User interface
        audio_system,
        rviz_node
    ])
