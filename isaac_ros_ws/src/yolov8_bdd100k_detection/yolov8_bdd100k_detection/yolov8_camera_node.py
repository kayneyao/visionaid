#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import time
import os

class YOLOv8CameraNode(Node):
    def __init__(self):
        # Initialize ROS 2 node first
        super().__init__('yolov8_camera_node')
        
        # Declare parameters with defaults
        self.declare_parameter('model_path', '/home/sophie/visionaid/models/yolov8/best.onnx')
        self.declare_parameter('confidence_threshold', 0.20)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('inference_rate', 30.0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('publish_detection_images', True)
        self.declare_parameter('enable_depth', True)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.publish_detection_images = self.get_parameter('publish_detection_images').value
        self.enable_depth = self.get_parameter('enable_depth').value
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.raw_image_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile)
        self.depth_image_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', qos_profile)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile)
        self.detections_pub = self.create_publisher(Detection2DArray, '/camera/detections', qos_profile)
        self.obstacle_pub = self.create_publisher(PointStamped, '/obstacles/points', qos_profile)
        
        if self.publish_detection_images:
            self.detection_image_pub = self.create_publisher(Image, '/camera/detection_image', qos_profile)
        
        # Initialize components
        self.bridge = CvBridge()
        
        # CORRECTED: Updated class names for your specific 12-class model
        self.class_names = {
            0: 'car',
            1: 'bus', 
            2: 'bicycle',
            3: 'pedestrian',
            4: 'pole',
            5: 'tree',
            6: 'trash_bin',
            7: 'crosswalk',
            8: 'road_sign',
            9: 'red_light',
            10: 'green_light',
            11: 'motorcycle'
        }
        
        # Initialize camera and model
        self.init_camera()
        self.init_model()
        self.setup_io_binding()
        
        # Create timer for inference
        self.timer = self.create_timer(
            1.0 / self.inference_rate,
            self.inference_callback
        )
        
        self.get_logger().info(f'TensorRT-optimized YOLOv8 Camera Node initialized with 12-class model: {self.model_path}')
        
    def init_camera(self):
        """Initialize RealSense camera"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Configure streams
            config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, self.camera_fps)
            if self.enable_depth:
                config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, self.camera_fps)
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Get camera intrinsics
            color_stream = profile.get_stream(rs.stream.color)
            self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            # Create align object for depth alignment
            if self.enable_depth:
                align_to = rs.stream.color
                self.align = rs.align(align_to)
            
            self.get_logger().info('RealSense camera initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {str(e)}')
            raise
   
    def init_model(self):
        """Initialize YOLOv8 model with TensorRT optimization"""
        try:
            import onnxruntime as ort
            
            # Create TensorRT cache directory
            cache_dir = '/tmp/onnxruntime_trt_cache'
            os.makedirs(cache_dir, exist_ok=True)
            
            # Enhanced TensorRT options
            trt_options = {
                'trt_fp16_enable': True,
                'trt_engine_cache_enable': True,
                'trt_timing_cache_enable': True,
                'trt_max_workspace_size': 4294967296,  # 4GB for better optimization
                'trt_engine_cache_path': cache_dir,
                'trt_timing_cache_path': cache_dir,
                'trt_dump_ep_context_model': True,
                'trt_ep_context_file_path': f'{cache_dir}/trt_context.json'
            }
            
            # Enhanced session options
            session_options = ort.SessionOptions()
            session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            session_options.log_severity_level = 1
            session_options.enable_mem_pattern = True
            session_options.enable_cpu_mem_arena = True
            session_options.enable_mem_reuse = True
            session_options.add_session_config_entry('session.disable_prepacking', '1')
            
            # Force TensorRT provider order
            providers = [
                ('TensorrtExecutionProvider', trt_options),
                'CUDAExecutionProvider',
                'CPUExecutionProvider'
            ]
            
            # Create optimized session
            self.ort_session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=providers
            )
            
            # Get input and output details
            self.input_name = self.ort_session.get_inputs()[0].name
            self.output_names = [output.name for output in self.ort_session.get_outputs()]
            
            # Log model output shape for verification
            for output in self.ort_session.get_outputs():
                self.get_logger().info(f'Model output: {output.name}, Shape: {output.shape}')
            
            # Verify TensorRT activation
            used_providers = self.ort_session.get_providers()
            self.get_logger().info(f'Active providers: {used_providers}')
            
            if 'TensorrtExecutionProvider' in used_providers:
                self.get_logger().info('SUCCESS: TensorRT provider is active!')
            else:
                self.get_logger().error('FAILED: TensorRT provider not active')
                raise RuntimeError("TensorRT provider failed to activate")
            
            self.get_logger().info(f'TensorRT-optimized 12-class model loaded: {self.model_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise

    def setup_io_binding(self):
        """Setup IOBinding for zero-copy GPU operations"""
        try:
            # Create IOBinding object
            self.io_binding = self.ort_session.io_binding()
            
            # Pre-allocate GPU memory
            input_shape = (1, 3, 640, 640)
            self.gpu_input = torch.zeros(input_shape, dtype=torch.float32, device='cuda')
            
            # Bind input to GPU memory
            self.io_binding.bind_input(
                name=self.input_name,
                device_type='cuda',
                device_id=0,
                element_type=np.float32,
                shape=input_shape,
                buffer_ptr=self.gpu_input.data_ptr()
            )
            
            # Bind outputs to GPU memory
            for output in self.ort_session.get_outputs():
                self.io_binding.bind_output(output.name, 'cuda')
            
            self.use_io_binding = True
            self.get_logger().info('IOBinding setup successful - zero-copy operations enabled')
            
        except Exception as e:
            self.get_logger().warn(f'IOBinding setup failed: {e}, using standard inference')
            self.use_io_binding = False

    def preprocess_image(self, image):
        """Preprocess image for ONNX Runtime inference"""
        # Resize to model input size (640x640 for YOLOv8)
        input_image = cv2.resize(image, (640, 640))
        
        # Convert BGR to RGB
        input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
        
        # Normalize to 0-1 range
        input_image = input_image.astype(np.float32) / 255.0
        
        # Convert HWC to CHW format
        input_image = np.transpose(input_image, (2, 0, 1))
        
        # Add batch dimension
        input_image = np.expand_dims(input_image, axis=0)
        
        return input_image

    def run_tensorrt_inference(self, preprocessed_image):
        """Run inference using TensorRT-optimized session"""
        if self.use_io_binding:
            # Copy data to GPU memory
            self.gpu_input.copy_(torch.from_numpy(preprocessed_image).cuda())
            
            # Run inference with IOBinding
            self.io_binding.synchronize_inputs()
            self.ort_session.run_with_iobinding(self.io_binding)
            self.io_binding.synchronize_outputs()
            
            # Get outputs
            outputs = self.io_binding.copy_outputs_to_cpu()
        else:
            # Standard inference
            outputs = self.ort_session.run(
                self.output_names,
                {self.input_name: preprocessed_image}
            )
        
        return outputs

    def apply_nms(self, boxes, scores, iou_threshold):
        """Apply Non-Maximum Suppression"""
        # Convert to x1, y1, x2, y2 format
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2
        
        areas = (x2 - x1) * (y2 - y1)
        indices = np.argsort(scores)[::-1]
        
        keep = []
        while len(indices) > 0:
            current = indices[0]
            keep.append(current)
            
            if len(indices) == 1:
                break
                
            # Calculate IoU with remaining boxes
            xx1 = np.maximum(x1[current], x1[indices[1:]])
            yy1 = np.maximum(y1[current], y1[indices[1:]])
            xx2 = np.minimum(x2[current], x2[indices[1:]])
            yy2 = np.minimum(y2[current], y2[indices[1:]])
            
            intersection = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            union = areas[current] + areas[indices[1:]] - intersection
            iou = intersection / union
            
            # Keep boxes with IoU below threshold
            indices = indices[1:][iou <= iou_threshold]
        
        return keep

    def process_tensorrt_outputs(self, outputs, original_shape):
        """CORRECTED: Process TensorRT outputs for 12-class model"""
        output = outputs[0]  # Shape: [1, 16, 8400]
        
        # Log actual shape for debugging
        self.get_logger().debug(f"Model output shape: {output.shape}")
        
        # Transpose to [batch, 8400, 16] for easier processing
        if len(output.shape) == 3 and output.shape[1] == 16:
            output = np.transpose(output, (0, 2, 1))  # [1, 8400, 16]
        
        # Extract bounding boxes (first 4 channels)
        boxes = output[0, :, :4]  # [8400, 4]
        
        # CORRECTED: Extract class scores (channels 4-15 for 12 classes)
        scores = output[0, :, 4:16]  # [8400, 12]
        
        # Apply confidence threshold
        max_scores = np.max(scores, axis=1)
        valid_indices = max_scores > self.confidence_threshold
        
        if not np.any(valid_indices):
            return []
        
        # Filter valid detections
        valid_boxes = boxes[valid_indices]
        valid_scores = scores[valid_indices]
        
        # CORRECTED: Apply softmax to normalize scores
        exp_scores = np.exp(valid_scores - np.max(valid_scores, axis=1, keepdims=True))
        normalized_scores = exp_scores / np.sum(exp_scores, axis=1, keepdims=True)
        
        valid_classes = np.argmax(normalized_scores, axis=1)
        valid_confidences = np.max(normalized_scores, axis=1)
        
        # CORRECTED: Ensure class IDs are within valid range (0-11 for 12 classes)
        valid_mask = valid_classes < 12
        valid_boxes = valid_boxes[valid_mask]
        valid_classes = valid_classes[valid_mask]
        valid_confidences = valid_confidences[valid_mask]
        
        if len(valid_boxes) == 0:
            return []
        
        # Scale boxes to original image size
        height, width = original_shape[:2]
        valid_boxes[:, [0, 2]] *= width / 640   # Scale x coordinates
        valid_boxes[:, [1, 3]] *= height / 640  # Scale y coordinates
        
        # Apply NMS
        indices = self.apply_nms(valid_boxes, valid_confidences, self.nms_threshold)
        
        # Format results
        results = []
        for idx in indices:
            results.append({
                'bbox': valid_boxes[idx],
                'confidence': valid_confidences[idx],
                'class_id': valid_classes[idx]
            })
        
        return results
    
    def create_camera_info_msg(self, header):
        """Create camera info message"""
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.width = self.intrinsics.width
        camera_info.height = self.intrinsics.height
        camera_info.distortion_model = "plumb_bob"
        
        # Camera matrix
        camera_info.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # Distortion coefficients
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix (identity for monocular)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix
        camera_info.p = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx, 0.0,
            0.0, self.intrinsics.fy, self.intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return camera_info
    
    def inference_callback(self):
        """Main inference callback with TensorRT optimization"""
        callback_start = time.time()
        try:
            # Get frames from camera
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            if not frames:
                self.get_logger().warn("No frames received within timeout")
                return
            
            if self.enable_depth:
                # Align depth frame to color frame
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
            else:
                color_frame = frames.get_color_frame()
                depth_frame = None
            
            if not color_frame:
                return
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None
            
            # Create header with current timestamp
            current_time = self.get_clock().now()
            header = Header()
            header.stamp = current_time.to_msg()
            header.frame_id = 'camera_color_optical_frame'
            
            # Publish raw images
            self.publish_images(color_image, depth_image, header)
            
            # Preprocess image for TensorRT inference
            preprocessed_image = self.preprocess_image(color_image)
            
            # Run TensorRT inference
            inference_start = time.time()
            outputs = self.run_tensorrt_inference(preprocessed_image)
            inference_time = (time.time() - inference_start) * 1000
            
            if inference_time > 10:  # Adjusted threshold for TensorRT
                self.get_logger().warn(f"Slow inference: {inference_time:.2f}ms")
            else:
                self.get_logger().info(f"Fast TensorRT inference: {inference_time:.2f}ms")
            
            # Process TensorRT outputs
            results = self.process_tensorrt_outputs(outputs, color_image.shape)
            
            # Publish detections
            self.publish_tensorrt_detections(results, header, color_image, depth_frame)
            
            # Log total callback time if slow
            total_time = (time.time() - callback_start) * 1000
            if total_time > 100:
                self.get_logger().warn(f"Slow callback: {total_time:.2f}ms")
            
        except Exception as e:
            self.get_logger().error(f'Inference callback error: {str(e)}')
    
    def publish_images(self, color_image, depth_image, header):
        """Publish raw camera images"""
        try:
            # Publish color image
            color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            color_msg.header = header
            self.raw_image_pub.publish(color_msg)
            
            # Publish depth image if available
            if depth_image is not None:
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')
                depth_msg.header = header
                self.depth_image_pub.publish(depth_msg)
            
            # Publish camera info
            camera_info = self.create_camera_info_msg(header)
            self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing images: {str(e)}')
    
    def publish_tensorrt_detections(self, results, header, color_image, depth_frame):
        """Process and publish TensorRT detection results"""
        try:
            detection_array = Detection2DArray()
            detection_array.header = header
            
            annotated_image = color_image.copy()
            
            for result in results:
                # Create detection message
                detection = Detection2D()
                detection.header = header
                
                # Extract box coordinates (center_x, center_y, width, height)
                bbox = result['bbox']
                center_x, center_y, width, height = bbox
                
                # Set bounding box
                detection.bbox.center.position.x = float(center_x)
                detection.bbox.center.position.y = float(center_y)
                detection.bbox.size_x = float(width)
                detection.bbox.size_y = float(height)
                
                # Set classification
                hypothesis = ObjectHypothesisWithPose()
                class_id = int(result['class_id'])
                hypothesis.hypothesis.class_id = str(class_id)
                hypothesis.hypothesis.score = float(result['confidence'])
                detection.results.append(hypothesis)
                
                detection_array.detections.append(detection)
                
                # Annotate image
                if self.publish_detection_images:
                    class_name = self.class_names.get(class_id, f'class_{class_id}')
                    confidence = float(result['confidence'])
                    
                    # Calculate corner coordinates for drawing
                    x1 = int(center_x - width / 2)
                    y1 = int(center_y - height / 2)
                    x2 = int(center_x + width / 2)
                    y2 = int(center_y + height / 2)
                    
                    # Draw bounding box
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw label
                    label = f'{class_name}: {confidence:.2f}'
                    cv2.putText(annotated_image, label, (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish obstacle points for navigation
                self.publish_obstacle_point(center_x, center_y, depth_frame, header, class_id)
            
            # Publish detections
            self.detections_pub.publish(detection_array)
            
            # Publish annotated image
            if self.publish_detection_images:
                detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
                detection_img_msg.header = header
                self.detection_image_pub.publish(detection_img_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing detections: {str(e)}')
    
    def publish_obstacle_point(self, center_x, center_y, depth_frame, header, class_id):
        """CORRECTED: Publish obstacle points for navigation - updated for 12 classes"""
        if depth_frame is None:
            return
        
        # Updated obstacle classes based on your actual model
        # Include classes that represent physical obstacles for navigation
        obstacle_classes = [
            0,   # car
            1,   # bus
            2,   # bicycle
            3,   # pedestrian
            4,   # pole
            5,   # tree
            6,   # trash_bin
            11   # motorcycle
        ]
        # Exclude: crosswalk (7), road_sign (8), red_light (9), green_light (10)
        # These are important for navigation but not physical obstacles
        
        if class_id not in obstacle_classes:
            return
        
        try:
            # Get depth value at detection center
            depth_value = depth_frame.get_distance(int(center_x), int(center_y))
            
            if depth_value > 0:  # Valid depth
                # Convert to 3D point using camera intrinsics
                point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [center_x, center_y], depth_value)
                
                # Create obstacle point message
                obstacle_point = PointStamped()
                obstacle_point.header = header
                obstacle_point.point.x = point_3d[2]  # Forward (Z in camera frame)
                obstacle_point.point.y = -point_3d[0]  # Left (X in camera frame, negated)
                obstacle_point.point.z = -point_3d[1]  # Up (Y in camera frame, negated)
                
                self.obstacle_pub.publish(obstacle_point)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing obstacle point: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources"""
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLOv8CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
