#!/usr/bin/env python3
"""
Launch: Taiwan Crossing Assistant (no VLM) + Experiment Recorder

Usage:
  ros2 launch traffic_crossing_assistant experiment_record_and_system.launch.py \
    model_path:=/home/sophie/visionaid-1/models/yolov8/balanced.onnx \
    output_dir:=/home/sophie/visionaid-1/experiment_data \
    segment_seconds:=10 \
    max_segments:=6 \
    record_viz:=false \
    enable_tegrastats:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_crossing = get_package_share_directory('traffic_crossing_assistant')

    # Args
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sophie/visionaid-1/models/yolov8/balanced.onnx',
        description='Path to YOLOv8 model'
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sophie/visionaid-1/experiment_data',
        description='Directory to store experiment sessions'
    )
    segment_seconds_arg = DeclareLaunchArgument(
        'segment_seconds', default_value='10', description='Clip segment duration (seconds)'
    )
    max_segments_arg = DeclareLaunchArgument(
        'max_segments', default_value='0', description='Max segments to record (0=unlimited)'
    )
    record_viz_arg = DeclareLaunchArgument(
        'record_viz', default_value='false', description='Save detection viz snapshots'
    )
    enable_tegrastats_arg = DeclareLaunchArgument(
        'enable_tegrastats', default_value='true', description='Run tegrastats while recording'
    )

    # Include VLM-free assistant + rely on yolov8_realsense from separate launch (user should start it or add include if needed)
    assistant_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_crossing, 'launch', 'crossing_assistant_only.launch.py')
        ]),
    )

    # Execute the recorder Python script as a process
    recorder_py = '/home/sophie/visionaid-1/isaac_ros_ws/src/traffic_crossing_assistant/scripts/experiment_recorder.py'

    recorder_proc = ExecuteProcess(
        cmd=[
            'python3', recorder_py,
            '--output', LaunchConfiguration('output_dir'),
            '--segment-seconds', LaunchConfiguration('segment_seconds'),
            '--max-segments', LaunchConfiguration('max_segments'),
            '--camera-topic', '/camera/camera/color/image_raw',
            '--detection-topic', '/camera/detections',
            '--detection-viz-topic', '/camera/detections/visualization',
            '--decision-topic', '/traffic_safety/crossing_decision',
            '--reasoning-topic', '/traffic_safety/decision_reasoning',
            '--traffic-light-state-topic', '/traffic_safety/traffic_light_state',
            '--traffic-light-conf-topic', '/traffic_safety/traffic_light_confidence',
            '--ttc-topic', '/traffic_safety/time_to_collision',
            '--vehicle-threat-topic', '/traffic_safety/vehicle_threat_status',
            '--immediate-danger-topic', '/traffic_safety/immediate_crossing_danger',
            '--motion-quality-topic', '/traffic_safety/motion_compensation_quality',
            '--tracking-timing-topic', '/traffic_safety/tracking_timing',
            '--record-viz', LaunchConfiguration('record_viz'),
            '--enable-tegrastats', LaunchConfiguration('enable_tegrastats'),
        ],
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        output_dir_arg,
        segment_seconds_arg,
        max_segments_arg,
        record_viz_arg,
        enable_tegrastats_arg,
        assistant_launch,
        recorder_proc,
    ]) 