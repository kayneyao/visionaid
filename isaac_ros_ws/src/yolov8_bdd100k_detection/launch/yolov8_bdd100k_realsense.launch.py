from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
        }]
    )

    # Isaac ROS YOLOv8 pipeline (DNN encoder, TensorRT, YOLOv8 decoder)
    yolov8_container = ComposableNodeContainer(
        name='yolov8_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_dnn_image_encoder',
                plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                name='dnn_image_encoder',
                parameters=[{
                    'input_image_width': 640,
                    'input_image_height': 640,
                    'network_image_width': 640,
                    'network_image_height': 640,
                    'image_mean': [0.0, 0.0, 0.0],
                    'image_stddev': [255.0, 255.0, 255.0],
                    'encoding_desired': 'rgb8',
                    'network_image_encoding': 'rgb8'
                }],
                remappings=[
                    ('image', '/camera/color/image_raw'),
                    ('encoded_tensor', 'tensor_pub')
                ]
            ),
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt',
                parameters=[{
                    'model_file_path': '/home/sophie/visionaid/isaac_ros_ws/models/yolov8_bdd100k/best.onnx',
                    'engine_file_path': '/home/sophie/visionaid/isaac_ros_ws/models/yolov8_bdd100k/best.trt',
                    'input_binding_names': ['images'],
                    'output_binding_names': ['output0'],
                    'input_tensor_names': ['image'],
                    'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                    'output_tensor_names': ['detections'],
                    'output_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                    'force_engine_update': False,
                    'verbose': False,
                    'enable_fp16': True
                }],
                remappings=[
                    ('tensor_pub', 'tensor_pub'),
                    ('tensor_sub', 'tensor_sub')
                ]
            ),
            ComposableNode(
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                name='yolov8_decoder',
                parameters=[{
                    'confidence_threshold': 0.25,
                    'nms_threshold': 0.45,
                }],
                remappings=[
                    ('tensor_sub', 'tensor_sub'),
                    ('detections_output', '/detections')
                ]
            )
        ],
        output='screen'
    )

    # Detection processor node
    detection_processor_node = Node(
        package='yolov8_bdd100k_detection',
        executable='detection_processor',
        name='detection_processor',
        output='screen',
        parameters=[{
            'confidence_threshold': 0.25
        }]
    )

    return LaunchDescription([
        realsense_node,
        yolov8_container,
        detection_processor_node
    ])
