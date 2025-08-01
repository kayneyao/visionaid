#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class/weights/balanced.onnx',
        description='Path to YOLOv8 11-class balanced augmented model'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold'
    )
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable Isaac ROS Visual SLAM'
    )
    
    # YOLOv8 Camera Node
    yolov8_camera_node = Node(
        package='yolov8_detection',
        executable='yolov8_camera_node',
        name='yolov8_camera_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
        output='screen'
    )
    
    # Static transform publishers
    camera_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        confidence_threshold_arg,
        enable_slam_arg,
        
        # Transform publishers
        camera_base_tf,
        
        # Main nodes
        yolov8_camera_node,
    ])
