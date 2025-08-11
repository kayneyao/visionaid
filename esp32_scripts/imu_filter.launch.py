from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_serial',
            arguments=[
                'serial',
                '--dev', '/dev/ttyACM0',
                '-b', '115200',
                '-v5'
            ],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'gain': 0.4,
                'zeta': 0.0,
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
