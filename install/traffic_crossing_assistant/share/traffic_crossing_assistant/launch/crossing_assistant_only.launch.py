#!/usr/bin/env python3
"""
Traffic Crossing Assistant - Standalone Launch
Updated for 11-class, 2-priority system
For testing the safety logic independently
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('traffic_crossing_assistant'),
        'config', 'taiwan_safety_config.yaml'
    )
    
    return LaunchDescription([
        # 2-Priority system components
        Node(package='traffic_crossing_assistant', executable='vehicle_movement_analyzer', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='crosswalk_analyzer', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='traffic_light_analyzer', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='decision_engine', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='audio_feedback_system', parameters=[config_file]),
        Node(package='traffic_crossing_assistant', executable='ego_motion_compensator', parameters=[config_file]),
    ])
