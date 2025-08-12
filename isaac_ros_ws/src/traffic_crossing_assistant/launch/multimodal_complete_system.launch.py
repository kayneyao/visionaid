#!/usr/bin/env python3
"""
Complete Multimodal Taiwan Traffic Safety System - Enhanced Audio Only
Updated for 11-class, 2-priority system
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('traffic_crossing_assistant'),
        'config', 'taiwan_safety_config.yaml')
    
    return LaunchDescription([
        # YOLOv8 detection system - UPDATED FOR 11-CLASS
        Node(
            package='yolov8_detection',
            executable='yolov8_camera_node',
            name='taiwan_traffic_detector',
            parameters=[{
                'model_path': '/home/sophie/visionaid-1/models/yolov8/balanced.onnx',
                'confidence_threshold': 0.5,
            }],
            output='screen'
        ),
        
        # 2-Priority hierarchy system (Priority 1: Vehicles, Priority 2: Traffic Lights)
        Node(package='traffic_crossing_assistant', executable='vehicle_movement_analyzer', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='crosswalk_analyzer', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='traffic_light_analyzer', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='decision_engine', parameters=[config_file]),
        
        # VLM integration (Layer 2: Natural Language)
        Node(package='traffic_crossing_assistant', executable='multimodal_safety_coordinator', parameters=[config_file]),
        
        # Enhanced audio system
        Node(
            package='traffic_crossing_assistant',
            executable='enhanced_audio_system',
            name='multimodal_audio',
            parameters=[config_file],
            output='screen'
        ),
    ])
