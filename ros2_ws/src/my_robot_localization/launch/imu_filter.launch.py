from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'gain': 0.0000866,
                'magnetic_declination': -0.0873,
                'use_magnetometer': True,
                'frequency': 100.0,
            }],
            remappings=[
                ('imu/data_raw', '/imu'),
                ('imu/mag', '/mag'),
                ('imu/data_fused', 'imu/data_fused'),
            ],
        ),
    ])
