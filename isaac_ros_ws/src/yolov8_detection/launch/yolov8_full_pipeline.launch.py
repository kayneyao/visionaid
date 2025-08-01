#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class/weights/balanced.onnx',
        description='Path to YOLOv8 11-class balanced augmented model'
    )
    
    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable Nav2 navigation stack'
    )
    
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='true',
        description='Enable SLAM mapping'
    )
    
    # Package paths
    yolov8_package = FindPackageShare('yolov8_detection')
    nav2_bringup_package = FindPackageShare('nav2_bringup')
    
    # Include YOLOv8 + SLAM launch
    yolov8_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            yolov8_package,
            '/launch/yolov8_slam_integration.launch.py'
        ]),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'enable_slam': LaunchConfiguration('enable_mapping'),
            'enable_rviz': 'false',  # We'll launch our own RViz
        }.items()
    )
    
    # Navigation stack (Nav2)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_package,
            '/launch/navigation_launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('enable_navigation')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                yolov8_package,
                'config',
                'nav2_params.yaml'
            ]),
            'map': PathJoinSubstitution([
                yolov8_package,
                'maps',
                'default_map.yaml'
            ])
        }.items()
    )
    
    # Obstacle to costmap converter
    obstacle_converter_node = Node(
        package='yolov8_detection',
        executable='obstacle_publisher.py',
        name='obstacle_converter',
        parameters=[{
            'obstacle_lifetime': 2.0,
            'inflation_radius': 0.5,
            'cost_scaling_factor': 10.0
        }],
        remappings=[
            ('/obstacles/points', '/obstacles/points'),
            ('/obstacles/costmap', '/local_costmap/obstacles')
        ]
    )
    
    # Enhanced RViz with navigation
    rviz_config_path = PathJoinSubstitution([
        yolov8_package,
        'config',
        'yolov8_navigation_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Robot state publisher (for simple robot model)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': '''
            <?xml version="1.0"?>
            <robot name="vision_robot">
              <link name="base_link">
                <visual>
                  <geometry>
                    <box size="0.3 0.3 0.1"/>
                  </geometry>
                  <material name="blue">
                    <color rgba="0 0 1 1"/>
                  </material>
                </visual>
              </link>
              <link name="camera_link">
                <visual>
                  <geometry>
                    <box size="0.05 0.1 0.05"/>
                  </geometry>
                  <material name="red">
                    <color rgba="1 0 0 1"/>
                  </material>
                </visual>
              </link>
              <joint name="camera_joint" type="fixed">
                <parent link="base_link"/>
                <child link="camera_link"/>
                <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
              </joint>
            </robot>
            '''
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        enable_navigation_arg,
        enable_mapping_arg,
        
        # Robot description
        robot_state_publisher,
        
        # Main detection and SLAM pipeline
        yolov8_slam_launch,
        
        # Obstacle processing
        obstacle_converter_node,
        
        # Navigation (optional)
        nav2_launch,
        
        # Visualization
        rviz_node,
    ])
