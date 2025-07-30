#!/usr/bin/env python3
"""
Taiwan Complete Traffic Safety System - Enhanced 3D Detection
Full integration with RealSense depth, ego-motion compensation, and TTC analysis
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_crossing = get_package_share_directory('traffic_crossing_assistant')
    pkg_yolov8 = get_package_share_directory('yolov8_detection')
    
    # Configuration files
    taiwan_config = os.path.join(pkg_crossing, 'config', 'taiwan_safety_config.yaml')
    audio_config = os.path.join(pkg_crossing, 'config', 'audio_config.yaml')
    
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/models/yolov8/17class/taiwan.onnx',
        description='Path to Taiwan 17-class ONNX model'
    )
    
    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio', 
        default_value='true',
        description='Enable enhanced audio feedback system'
    )
    
    enable_ttc_arg = DeclareLaunchArgument(
        'enable_ttc',
        default_value='true',
        description='Enable time-to-collision analysis'
    )
    
    # YOLOv8 Detection with RealSense integration
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_yolov8, 'launch', 'yolov8_realsense.launch.py')
        ]),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
        }.items()
    )
    
    # Enhanced Vehicle Movement Analyzer (Priority 1: 3D Depth-Aware)
    vehicle_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='vehicle_movement_analyzer',
        name='enhanced_vehicle_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('/camera/detections', '/camera/detections'),
            ('/camera/aligned_depth_to_color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/rtabmap/odom', '/rtabmap/odom'),
            ('/immediate_crossing_danger', '/traffic_safety/immediate_crossing_danger'),
            ('/vehicle_threat_status', '/traffic_safety/vehicle_threat_status'),
            ('/time_to_collision', '/traffic_safety/time_to_collision'),
            ('/vehicle_relative_velocity', '/traffic_safety/vehicle_relative_velocity')
        ],
        output='screen'
    )
    
    # Enhanced Ego Motion Compensator
    ego_motion = Node(
        package='traffic_crossing_assistant',
        executable='ego_motion_compensator',
        name='enhanced_ego_motion_compensator',
        parameters=[taiwan_config],
        remappings=[
            ('/rtabmap/odom', '/rtabmap/odom'),
            ('/camera/detections', '/camera/detections'),
            ('/detections/motion_compensated', '/traffic_safety/motion_compensated_detections'),
            ('/motion_compensation_quality', '/traffic_safety/motion_compensation_quality'),
            ('/excessive_motion_detected', '/traffic_safety/excessive_motion_detected'),
            ('/motion_compensation_stats', '/traffic_safety/motion_compensation_stats')
        ],
        output='screen'
    )
    
    # Taiwan Crossing Analyzer (Priority 2: Your 85.5% mAP50 Innovation)
    crossing_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='taiwan_crossing_analyzer',
        name='taiwan_crossing_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('/camera/detections', '/camera/detections'),
            ('/crossing_path_confirmed', '/traffic_safety/crossing_path_confirmed'),
            ('/spatial_classification_confidence', '/traffic_safety/spatial_classification_confidence'),
            ('/taiwan_crossing_status', '/traffic_safety/taiwan_crossing_status')
        ],
        output='screen'
    )
    
    # Traffic Light Analyzer (Priority 3: Conservative Signal Analysis)
    traffic_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='traffic_light_analyzer',
        name='traffic_light_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('/camera/detections', '/camera/detections'),
            ('/traffic_light_state', '/traffic_safety/traffic_light_state'),
            ('/traffic_light_confidence', '/traffic_safety/traffic_light_confidence')
        ],
        output='screen'
    )
    
    # Main Decision Engine (3-Priority Hierarchy)
    decision_engine = Node(
        package='traffic_crossing_assistant',
        executable='decision_engine',
        name='decision_engine',
        parameters=[taiwan_config],
        remappings=[
            ('/immediate_crossing_danger', '/traffic_safety/immediate_crossing_danger'),
            ('/crossing_path_confirmed', '/traffic_safety/crossing_path_confirmed'),
            ('/spatial_classification_confidence', '/traffic_safety/spatial_classification_confidence'),
            ('/traffic_light_state', '/traffic_safety/traffic_light_state'),
            ('/traffic_light_confidence', '/traffic_safety/traffic_light_confidence'),
            ('/crossing_decision', '/traffic_safety/crossing_decision'),
            ('/decision_reasoning', '/traffic_safety/decision_reasoning')
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
            ('/crossing_decision', '/traffic_safety/crossing_decision'),
            ('/time_to_collision', '/traffic_safety/time_to_collision'),
            ('/vehicle_relative_velocity', '/traffic_safety/vehicle_relative_velocity'),
            ('/audio_message', '/traffic_safety/audio_message')
        ],
        output='screen'
    )
    
    # Multimodal Safety Coordinator (VLM Integration)
    multimodal_coordinator = Node(
        package='traffic_crossing_assistant',
        executable='multimodal_safety_coordinator',
        name='multimodal_safety_coordinator',
        parameters=[taiwan_config],
        remappings=[
            ('/camera/color/image_raw', '/camera/camera/color/image_raw'),
            ('/camera/detections', '/camera/detections'),
            ('/crossing_decision', '/traffic_safety/crossing_decision'),
            ('/user_voice_query', '/traffic_safety/user_voice_query'),
            ('/multimodal_guidance', '/traffic_safety/multimodal_guidance'),
            ('/vlm_scene_description', '/traffic_safety/vlm_scene_description'),
            ('/safety_reasoning_explanation', '/traffic_safety/safety_reasoning_explanation')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        enable_audio_arg,
        enable_ttc_arg,
        
        # Detection system (your excellent 96.35% pedestrian, 85.91% vehicle performance)
        yolov8_launch,
        
        # Enhanced 3D depth-aware components
        ego_motion,                    # Enhanced ego-motion compensation
        vehicle_analyzer,              # Priority 1: 3D depth-aware vehicle analysis
        
        # Priority hierarchy components
        crossing_analyzer,             # Priority 2: Taiwan crossing innovation  
        traffic_analyzer,              # Priority 3: Traffic light analysis
        
        # Decision coordination
        decision_engine,
        
        # User interface
        multimodal_coordinator,        # VLM integration
        audio_system,                  # Enhanced audio feedback
    ])
