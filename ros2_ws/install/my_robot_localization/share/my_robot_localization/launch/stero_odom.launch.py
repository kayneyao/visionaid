from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Frame IDs (override if yours differ)
    imu_frame   = LaunchConfiguration('imu_frame')
    rgb_frame   = LaunchConfiguration('rgb_optical_frame')
    ir1_frame   = LaunchConfiguration('ir1_optical_frame')
    ir2_frame   = LaunchConfiguration('ir2_optical_frame')
    cam_frame   = LaunchConfiguration('camera_frame')

    # Topic roots (override if your namespace differs)
    left_rect   = LaunchConfiguration('left_rect')    # IR1 rectified image
    right_rect  = LaunchConfiguration('right_rect')   # IR2 rectified image
    left_info   = LaunchConfiguration('left_info')
    right_info  = LaunchConfiguration('right_info')
    imu_topic   = LaunchConfiguration('imu_topic')

    return LaunchDescription([
        # ---------- Args ----------
        DeclareLaunchArgument('imu_frame',          default_value='imu_link'),
        DeclareLaunchArgument('rgb_optical_frame',  default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('ir1_optical_frame',  default_value='camera_infra1_optical_frame'),
        DeclareLaunchArgument('ir2_optical_frame',  default_value='camera_infra2_optical_frame'),
        DeclareLaunchArgument('camera_frame',       default_value='camera_link'),

        DeclareLaunchArgument('left_rect',  default_value='/camera/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_rect', default_value='/camera/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_info',  default_value='/camera/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_info', default_value='/camera/camera/infra2/camera_info'),
        DeclareLaunchArgument('imu_topic',  default_value='/imu/data'),

        # ---------- Stereo Visual Odometry (IR1/IR2 + IMU) ----------
        Node(
            package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': cam_frame,       # base camera frame (usually camera_link)
                'odom_frame_id': 'odom',
                'publish_tf': False,         # let your state estimator own TF if you use one
                'approx_sync': True,
                'queue_size': 10,
                'subscribe_imu': True,
                'wait_imu_to_init': True,
                'stereo': True, 

                'rtabmap_args': (
                    '--delete_db_on_start'
                )
            }],
            remappings=[
                ('left/image_rect',   left_rect),
                ('right/image_rect',  right_rect),
                ('left/camera_info',  left_info),
                ('right/camera_info', right_info),
                ('imu',               imu_topic),
            ],
        ),
    ])
