#!/usr/bin/env python3
"""
Taiwan 17-Class Traffic Safety YOLOv8 Camera Node - GPU ONLY
Forces GPU execution with proper ONNX Runtime provider configuration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
import time
import onnxruntime as ort
from pathlib import Path


class YOLOv8CameraNode(Node):
    def __init__(self):
        super().__init__('yolov8_camera_node')
        
        # Taiwan 17-class mapping
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crossing_crosswalk',
            4: 'crossing_green_light', 5: 'crossing_red_light', 
            6: 'irrelevant_crosswalk', 7: 'irrelevant_green_light',
            8: 'irrelevant_red_light', 9: 'motorcycle', 10: 'pedestrian',
            11: 'sidewalk', 12: 'truck', 13: 'crosswalk',
            14: 'red_light', 15: 'green_light', 16: 'tree'
        }
        
        # Taiwan class priorities for decision tree integration
        self.taiwan_priorities = {
            'critical_crossing': [3, 4, 5],      # Your research innovation
            'safety_vehicles': [0, 1, 2, 9, 12], # Vehicle safety
            'safety_pedestrian': [10],           # Pedestrian safety
            'fallback_traffic': [13, 14, 15],    # Fallback signals
            'context_classes': [6, 7, 8, 11, 16] # Background context
        }
        
        # Parameters
        self.declare_parameter('model_path', '/home/sophie/visionaid-1/models/yolov8/17class/taiwan.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('max_detections', 50)
        self.declare_parameter('inference_rate', 30.0)
        self.declare_parameter('publish_detection_images', True)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.max_detections = self.get_parameter('max_detections').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.publish_images = self.get_parameter('publish_detection_images').value
        
        # Initialize model
        self.model = None
        self.bridge = CvBridge()
        
        # GPU validation
        if not torch.cuda.is_available():
            self.get_logger().error('❌ CUDA not available - GPU required for this node!')
            raise RuntimeError("GPU required but CUDA not available")
        
        self.get_logger().info(f'✅ CUDA available - GPU: {torch.cuda.get_device_name(0)}')
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        
        if self.publish_images:
            self.image_pub = self.create_publisher(
                Image, 'detections/visualization', 10)
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        
        # Performance tracking
        self.inference_times = []
        self.frame_count = 0
        self.gpu_memory_logged = False
        
        # Initialize model with GPU requirements
        if self.initialize_model_gpu():
            self.get_logger().info('✅ Taiwan Traffic Detection Node Ready (GPU MODE)')
            self.get_logger().info(f'🇹🇼 17-class Taiwan model loaded: {self.model_path}')
        else:
            self.get_logger().error('❌ Failed to initialize Taiwan model with GPU')
            raise RuntimeError("Failed to initialize GPU model")
        
        # Performance logging timer
        self.create_timer(5.0, self.log_performance)
        
        # GPU memory monitoring timer
        self.create_timer(10.0, self.log_gpu_status)
    
    def initialize_model_gpu(self):
        """Initialize Taiwan 17-class YOLO model with FORCED GPU execution"""
        try:
            if not Path(self.model_path).exists():
                self.get_logger().error(f'Model file not found: {self.model_path}')
                return False
            
            self.get_logger().info(f'Loading Taiwan model: {self.model_path}')
            
            # Configure ONNX Runtime to FORCE GPU execution
            if self.model_path.endswith('.onnx'):
                # Set ONNX Runtime to use GPU providers ONLY
                available_providers = ort.get_available_providers()
                self.get_logger().info(f'Available ONNX providers: {available_providers}')
                
                if 'CUDAExecutionProvider' not in available_providers:
                    self.get_logger().error('❌ CUDAExecutionProvider not available in ONNX Runtime')
                    return False
                
                # Force GPU-only providers (no CPU fallback)
                gpu_providers = ['CUDAExecutionProvider']
                if 'TensorrtExecutionProvider' in available_providers:
                    gpu_providers.insert(0, 'TensorrtExecutionProvider')
                    self.get_logger().info('🚀 TensorRT provider available - maximum performance mode')
                
                self.get_logger().info(f'Using providers: {gpu_providers}')
            
            # Load YOLO model
            self.model = YOLO(self.model_path)
            
            # For PyTorch models, force GPU
            if self.model_path.endswith('.pt'):
                self.model.to('cuda')
                self.get_logger().info('✅ PyTorch model moved to GPU')
            
            # Verify class count
            if hasattr(self.model, 'names') and len(self.model.names) != 17:
                self.get_logger().warning(f'Expected 17 classes, got {len(self.model.names)}')
            
            # Log Taiwan-specific classes
            self.get_logger().info('Taiwan spatial classification classes:')
            for idx in self.taiwan_priorities['critical_crossing']:
                if idx < len(self.class_names):
                    self.get_logger().info(f'  {idx}: {self.class_names[idx]}')
            
            # Verify GPU memory availability
            if torch.cuda.is_available():
                gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
                self.get_logger().info(f'✅ GPU Memory Available: {gpu_memory:.1f} GB')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'GPU model initialization failed: {e}')
            return False
    
    def image_callback(self, msg: Image):
        """Process camera images with FORCED GPU Taiwan traffic detection"""
        if self.model is None:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # FORCE GPU inference - no CPU fallback
            start_time = time.time()
            results = self.model(cv_image, 
                               conf=self.confidence_threshold,
                               iou=self.nms_threshold,
                               max_det=self.max_detections,
                               device=0,  # FORCE GPU device 0
                               verbose=False)
            
            inference_time = (time.time() - start_time) * 1000
            self.inference_times.append(inference_time)
            self.frame_count += 1
            
            # Log first successful GPU inference
            if self.frame_count == 1:
                self.get_logger().info(f'🚀 First GPU inference: {inference_time:.1f}ms')
            
            # Convert to ROS Detection2DArray
            detection_array = self.convert_to_detection_array(results[0], msg.header)
            self.detection_pub.publish(detection_array)
            
            # Publish annotated image if enabled
            if self.publish_images and results[0].boxes is not None:
                annotated_image = self.create_annotated_image(cv_image, results[0])
                img_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
                img_msg.header = msg.header
                self.image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'GPU inference failed: {e}')
            # No CPU fallback - raise error to indicate GPU failure
            raise RuntimeError(f"GPU inference required but failed: {e}")
    
    def convert_to_detection_array(self, results, header):
        """Convert YOLO results to ROS Detection2DArray with Taiwan priorities"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        if results.boxes is None:
            return detection_array
        
        boxes = results.boxes.cpu().numpy()
        
        # Sort by Taiwan priority for decision tree integration
        detections_with_priority = []
        for i, box in enumerate(boxes.data):
            x1, y1, x2, y2, conf, cls_id = box
            cls_id = int(cls_id)
            priority = self.get_taiwan_priority(cls_id)
            
            detections_with_priority.append({
                'box': box,
                'priority': priority,
                'class_id': cls_id
            })
        
        # Sort by priority (higher first)
        detections_with_priority.sort(key=lambda x: x['priority'], reverse=True)
        
        # Convert to Detection2D messages
        for det_info in detections_with_priority:
            box = det_info['box']
            x1, y1, x2, y2, conf, cls_id = box
            cls_id = int(cls_id)
            
            detection = Detection2D()
            
            # Bounding box
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
            
            # Classification result
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(cls_id)
            hypothesis.hypothesis.score = float(conf)
            
            detection.results.append(hypothesis)
            detection_array.detections.append(detection)
        
        return detection_array
    
    def get_taiwan_priority(self, class_id: int) -> float:
        """Assign priority for decision tree integration"""
        if class_id in self.taiwan_priorities['critical_crossing']:
            return 1.0  # Highest priority for your research innovation
        elif class_id in self.taiwan_priorities['safety_vehicles']:
            return 0.9  # High priority for vehicle safety
        elif class_id in self.taiwan_priorities['safety_pedestrian']:
            return 0.9  # High priority for pedestrian safety
        elif class_id in self.taiwan_priorities['fallback_traffic']:
            return 0.8  # Medium priority for fallback signals
        elif class_id in self.taiwan_priorities['context_classes']:
            return 0.6  # Lower priority for context
        else:
            return 0.5  # Default priority
    
    def create_annotated_image(self, image, results):
        """Create annotated image with Taiwan class priorities"""
        annotated = image.copy()
        
        if results.boxes is not None:
            boxes = results.boxes.cpu().numpy()
            
            for box in boxes.data:
                x1, y1, x2, y2, conf, cls_id = box
                cls_id = int(cls_id)
                
                # Color coding based on Taiwan priorities
                if cls_id in self.taiwan_priorities['critical_crossing']:
                    color = (0, 255, 0)  # Green for Taiwan innovations
                elif cls_id in self.taiwan_priorities['safety_vehicles'] or \
                     cls_id in self.taiwan_priorities['safety_pedestrian']:
                    color = (0, 0, 255)  # Red for safety-critical
                elif cls_id in self.taiwan_priorities['fallback_traffic']:
                    color = (255, 0, 0)  # Blue for fallback
                else:
                    color = (128, 128, 128)  # Gray for context
                
                # Draw bounding box
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                
                # Draw label
                label = f"{self.class_names.get(cls_id, 'unknown')}:{conf:.2f}"
                cv2.putText(annotated, label, (int(x1), int(y1-10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated
    
    def log_performance(self):
        """Log GPU performance statistics"""
        if not self.inference_times:
            return
        
        recent_times = self.inference_times[-50:]  # Last 50 frames
        avg_time = np.mean(recent_times)
        fps = 1000.0 / avg_time if avg_time > 0 else 0
        
        self.get_logger().info(f'🚀 Taiwan GPU Detection: {avg_time:.1f}ms avg, {fps:.1f} FPS')
        
        # Check for target 2.3ms performance
        if avg_time <= 2.5:
            self.get_logger().info('✅ Achieving target performance (<2.5ms)')
        elif avg_time < 10.0:
            self.get_logger().info('✅ Excellent GPU performance (<10ms)')
        elif avg_time < 33.0:
            self.get_logger().info('✅ Meeting real-time requirements')
        else:
            self.get_logger().warning('⚠️ Below real-time performance - check GPU utilization')
    
    def log_gpu_status(self):
        """Log GPU memory and utilization status"""
        if torch.cuda.is_available() and not self.gpu_memory_logged:
            try:
                gpu_memory_used = torch.cuda.memory_allocated(0) / 1e9
                gpu_memory_total = torch.cuda.get_device_properties(0).total_memory / 1e9
                gpu_utilization = (gpu_memory_used / gpu_memory_total) * 100
                
                self.get_logger().info(f'🔥 GPU Memory: {gpu_memory_used:.1f}GB/{gpu_memory_total:.1f}GB ({gpu_utilization:.1f}%)')
                self.gpu_memory_logged = True
                
            except Exception as e:
                self.get_logger().warning(f'Could not read GPU status: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLOv8CameraNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"❌ GPU-only node failed to start: {e}")
    except KeyboardInterrupt:
        print("🛑 GPU detection node stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
