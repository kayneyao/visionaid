import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_localization')
    params_file = os.path.join(pkg_share, 'config', 'robot_localization.yaml')

    return LaunchDescription([
        # Static transform from base_link to imu_link for TF\        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_broadcaster',
            arguments=['0','0','0','0','0','0','base_link','imu_link']
        ),
        # EKF fusing IMU + GPS → odom\        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/imu/data', '/sensors/imu'),
                ('/odometry/gps', '/odometry/gps')
            ]
        ),
        # NavSat transform: GPS fix → odometry/gps\        
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/gps/fix', '/sensors/gps'),
                ('/odometry/filtered', '/odometry/gps')
            ]
        ),
    ])