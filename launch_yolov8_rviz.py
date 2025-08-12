#!/usr/bin/env python3
"""
Complete YOLOv8 Detection System with RViz Visualization
Launches detection system, marker converter, and RViz
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
        
        # Launch the YOLOv8 detection system
        Node(
            package='yolov8_detection',
            executable='yolov8_camera_node',
            name='yolov8_camera_node',
            output='screen',
            parameters=[{
                'model_path': '/home/sophie/visionaid-1/models/yolov8/balanced.onnx',
                'confidence_threshold': 0.5,
                'inference_rate': 30.0,
                'publish_detection_images': True
            }]
        ),
        
        # Launch the detection to markers converter
        Node(
            package='python3',
            executable='python3',
            name='detection_to_markers',
            arguments=[os.path.join(os.path.dirname(os.path.abspath(__file__)), 'detection_to_markers.py')],
            output='screen'
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