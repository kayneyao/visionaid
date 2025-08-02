#!/usr/bin/env python3
"""
Launch file for RViz with YOLOv8 detection visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the path to the RViz configuration file
    rviz_config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'yolov8_detection.rviz'
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Path to RViz configuration file'
        ),
        
        # Launch RViz with the custom configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ]) 