#!/usr/bin/env python3
"""
Taiwan Complete Traffic Safety System - 11-Class 2-Priority System
Full integration with RealSense depth, ego-motion compensation, and TTC analysis
Updated for: bicycle, bus, car, crosswalk, greenlight, motorcycle, pedestrian, redlight, sidewalk, truck, yellowlight
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
        default_value='/home/sophie/visionaid-1/models/yolov8/balanced.onnx',
        description='Path to Balanced Augmented 11-class ONNX model'
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

    enable_vlm_arg = DeclareLaunchArgument(
        'enable_vlm',
        default_value='false',
        description='Enable VLM multimodal coordinator'
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
            ('/camera/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw'),
            ('/rtabmap/odom', '/rtabmap/odom'),
            ('/immediate_crossing_danger', '/traffic_safety/immediate_crossing_danger'),
            ('/vehicle_threat_status', '/traffic_safety/vehicle_threat_status'),
            ('/time_to_collision', '/traffic_safety/time_to_collision'),
            ('/vehicle_relative_velocity', '/traffic_safety/vehicle_relative_velocity'),
            ('/traffic_safety/tracking_timing', '/traffic_safety/tracking_timing')
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
    
    # Crosswalk Analyzer (Context Information)
    crosswalk_analyzer = Node(
        package='traffic_crossing_assistant',
        executable='crosswalk_analyzer',
        name='crosswalk_analyzer',
        parameters=[taiwan_config],
        remappings=[
            ('/camera/detections', '/camera/detections'),
            ('/crosswalk_detected', '/traffic_safety/crosswalk_detected'),
            ('/crosswalk_confidence', '/traffic_safety/crosswalk_confidence'),
            ('/crosswalk_status', '/traffic_safety/crosswalk_status')
        ],
        output='screen'
    )
    
    # Traffic Light Analyzer (Priority 2: Signal Analysis)
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
    
    # Main Decision Engine (2-Priority Hierarchy)
    decision_engine = Node(
        package='traffic_crossing_assistant',
        executable='decision_engine',
        name='decision_engine',
        parameters=[taiwan_config],
        remappings=[
            ('/immediate_crossing_danger', '/traffic_safety/immediate_crossing_danger'),
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
        enable_vlm_arg,
        
        # Detection system (11-class model)
        yolov8_launch,
        
        # Enhanced 3D depth-aware components
        ego_motion,                    # Enhanced ego-motion compensation
        vehicle_analyzer,              # Priority 1: 3D depth-aware vehicle analysis
        
        # 2-Priority hierarchy components
        crosswalk_analyzer,            # Context: Crosswalk detection
        traffic_analyzer,              # Priority 2: Traffic light analysis
        
        # Decision coordination
        decision_engine,               # 2-priority decision system
        
        # User interface (VLM optional)
        audio_system,                  # Enhanced audio feedback
    ] + (
        [multimodal_coordinator] if LaunchConfiguration('enable_vlm') == 'true' else []
    ))
