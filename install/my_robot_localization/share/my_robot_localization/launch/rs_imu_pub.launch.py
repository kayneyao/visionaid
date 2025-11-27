"""Launch realsense2_camera node."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name',                  'default': 'camera',      'description': 'camera unique name'},
                    {'name': 'camera_namespace',             'default': 'camera',      'description': 'camera namespace'},
                    {'name': 'enable_color',                 'default': 'true',        'description': 'enable color stream'},
                    {'name': 'enable_depth',                 'default': 'true',        'description': 'enable depth stream'},
                    {'name': 'enable_infra1',                 'default': 'false',        'description': 'enable depth stream'},
                    {'name': 'enable_infra2',                 'default': 'false',        'description': 'enable depth stream'},
                    {'name': 'rgb_camera.color_profile',    'default': '1280x720x30', 'description': 'set color resolution'},
                    {'name': 'depth_module.depth_profile',  'default': '1280x720x30', 'description': 'set depth resolution'},
                    # {'name': 'depth_module.infra_profile',  'default': '1280x720x30', 'description': 'set depth resolution'},
                    {'name': 'depth_module.emitter_always_on',  'default': 'false', 'description': 'set depth resolution'},
                    {'name': 'depth_module.emitter_enabled',  'default': '0', 'description': 'set depth resolution'},
                    {'name': 'rgb_camera.enable_auto_exposure',  'default': 'true', 'description': 'enable depth auto exposure'},
                    {'name': 'rgb_camera.auto_exposure_priority',  'default': 'false', 'description': 'enable depth auto exposure'},
                    {'name': 'depth_module.enable_auto_exposure',  'default': 'true', 'description': 'enable depth auto exposure'},
                    {'name': 'depth_module.auto_exposure_priority',  'default': 'false', 'description': 'enable depth auto exposure'},
                    {'name': 'align_depth.enable',           'default': 'true',        'description': 'enable align depth filter'},
                    {'name': 'depth_module.laser_power',           'default': '0',        'description': 'enable align depth filter'},
                    # {'name': 'rgb_camera.exposure',           'default': '120',        'description': ''},
                    # {'name': 'rgb_camera.gain',           'default': '64',        'description': ''},
                    # {'name': 'rgb_camera.brightness',           'default': '5.0',        'description': ''},
                    # {'name': 'rgb_camera.contrast',           'default': '54.0',        'description': ''},
                    # {'name': 'rgb_camera.saturation',           'default': '58.0',        'description': ''},
                    # {'name': 'pointcloud.enable',             'default': 'true',        'description': 'enable pointcloud'},
                    {'name': 'enable_sync',                  'default': 'true',        'description': 'enable sync mode'},
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])

def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) + 
        [
            OpaqueFunction(function=rs_launch.launch_setup,
                           kwargs={'params': set_configurable_parameters(params)}),

            # ------- SINGLE STATIC TF: camera_link -> imu_link -------
            # (computed from camera_link->IR1_optical and Kalibr cam1 T_ic)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_imu_to_camera',
                arguments=[
                    # translation (m)
                    '--x','0.01801478','--y','0.00884002','--z','-0.08406489',
                    # quaternion (x y z w)
                    '--qx','-0.00293130','--qy','0.00095323','--qz','-0.71006219','--qw','0.70413222',
                    # parent and child
                    '--frame-id','imu_link','--child-frame-id','camera_link'
                ],
                output='screen'
            ),

            Node(
                package='my_robot_localization',
                executable='imu_pub.py',
            ),

            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                parameters=[{
                    'gain': 0.8,
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
        ]
    )
