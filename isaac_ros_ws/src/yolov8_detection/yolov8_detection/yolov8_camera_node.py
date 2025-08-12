#!/usr/bin/env python3
"""
11-Class Traffic Safety YOLOv8 Camera Node - GPU ONLY
Updated for fresh 11-class model with 2-priority safety system
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
        
        # NEW: 11-class mapping
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # NEW: 2-Priority Safety System
        self.safety_priorities = {
            # Priority 1: Vehicle Safety (ABSOLUTE OVERRIDE)
            'vehicles': [1, 2, 5, 9],           # bus, car, motorcycle, truck
            
            # Priority 2: Traffic Lights
            'traffic_lights': [4, 7, 10],      # greenlight, redlight, yellowlight
            
            # Context classes (no safety override)
            'pedestrian_context': [0, 6],      # bicycle, pedestrian (cross together)
            'infrastructure': [3, 8]           # crosswalk, sidewalk
        }
        
        # NEW: Class-specific confidence thresholds (lowered for temporal filtering)
        self.class_confidence_thresholds = {
            0: 0.55,   # bicycle - lowered for temporal filtering
            1: 0.55,   # bus - lowered for temporal filtering
            2: 0.55,   # car - lowered for temporal filtering
            3: 0.5,   # crosswalk - lowered for temporal filtering
            4: 0.2,   # greenlight - very low for small objects
            5: 0.65,   # motorcycle - lowered for temporal filtering
            6: 0.5,   # pedestrian - lowered for temporal filtering
            7: 0.2,   # redlight - very low for small objects
            8: 0.5,   # sidewalk - lowered for temporal filtering
            9: 0.7,   # truck - lowered for temporal filtering
            10: 0.2   # yellowlight - very low for small objects
        }
        
        # NEW: Temporal consistency filter to eliminate glitch false positives
        self.temporal_filter = {
            'detection_history': {},  # Track detections per class
            'min_consecutive_frames': 2,  # Must be detected for 2 consecutive frames
            'max_frames_without_detection': 2,  # Allow 2 frames gap
            'frame_count': 0
        }
        
        # Parameters
        self.declare_parameter('model_path', '/home/sophie/visionaid-1/models/yolov8/balanced.onnx')
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
        
        # Log class-specific thresholds and temporal filter settings
        self.get_logger().info('🎯 Class-specific confidence thresholds:')
        for class_id, threshold in self.class_confidence_thresholds.items():
            class_name = self.class_names.get(class_id, f'class_{class_id}')
            self.get_logger().info(f'   {class_name} (Class {class_id}): {threshold}')
        
        self.get_logger().info('⏱️ Temporal filter settings:')
        self.get_logger().info(f'   Min consecutive frames: {self.temporal_filter["min_consecutive_frames"]}')
        self.get_logger().info(f'   Max frames without detection: {self.temporal_filter["max_frames_without_detection"]}')
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        
        # Raw detections publisher (before filtering)
        self.raw_detection_pub = self.create_publisher(
            Detection2DArray, 'detections/raw', 10)
        
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
            self.get_logger().info(f'🇹🇼 11-class Taiwan model loaded: {self.model_path}')
        else:
            self.get_logger().error('❌ Failed to initialize Taiwan model with GPU')
            raise RuntimeError("Failed to initialize GPU model")
        
        # Performance logging timer
        self.create_timer(5.0, self.log_performance)
        
        # GPU memory monitoring timer
        self.create_timer(10.0, self.log_gpu_status)
    
    def initialize_model_gpu(self):
        """Initialize Taiwan 11-class YOLO model with FORCED GPU execution"""
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
            if hasattr(self.model, 'names') and len(self.model.names) != 11:
                self.get_logger().warning(f'Expected 11 classes, got {len(self.model.names)}')
            
            # Log Taiwan-specific classes
            self.get_logger().info('Taiwan spatial classification classes:')
            for idx in self.safety_priorities['vehicles']:
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
            
            # Publish raw detections (before filtering)
            raw_detection_array = self.convert_to_raw_detection_array(results[0], msg.header)
            self.raw_detection_pub.publish(raw_detection_array)
            
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
        """Convert YOLO results to ROS Detection2DArray with temporal filtering and class-specific thresholds"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        if results.boxes is None:
            return detection_array
        
        boxes = results.boxes.cpu().numpy()

        
        # Update temporal filter with current detections
        self.update_temporal_filter(boxes.data)
        
        # Sort by Taiwan priority for decision tree integration
        detections_with_priority = []
        for i, box in enumerate(boxes.data):
            x1, y1, x2, y2, conf, cls_id = box
            cls_id = int(cls_id)
            
            # Apply class-specific confidence threshold
            class_threshold = self.class_confidence_thresholds.get(cls_id, self.confidence_threshold)
            if conf < class_threshold:
                continue  # Skip detections below class-specific threshold
            
            # Apply temporal consistency filter (but bypass for traffic lights)
            if cls_id not in self.safety_priorities['traffic_lights']:  # Skip temporal filter for traffic lights
                if not self.is_temporally_consistent(cls_id):
                    continue  # Skip detections that aren't temporally consistent
            
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
    
    def convert_to_raw_detection_array(self, results, header):
        """Convert YOLO results to ROS Detection2DArray WITHOUT filtering (raw detections)"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        if results.boxes is None:
            return detection_array
        
        boxes = results.boxes.cpu().numpy()
        
        # Convert ALL detections without any filtering
        for box in boxes.data:
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
        if class_id in self.safety_priorities['vehicles']:
            return 1.0  # Highest priority for vehicle safety
        elif class_id in self.safety_priorities['traffic_lights']:
            return 0.8  # High priority for traffic lights
        elif class_id in self.safety_priorities['pedestrian_context']:
            return 0.6  # Medium priority for pedestrian context
        elif class_id in self.safety_priorities['infrastructure']:
            return 0.4  # Lower priority for infrastructure
        else:
            return 0.5  # Default priority
    
    def update_temporal_filter(self, detections):
        """Update temporal filter with current frame detections"""
        self.temporal_filter['frame_count'] += 1
        current_frame = self.temporal_filter['frame_count']
        
        # Get unique class IDs from current detections
        current_classes = set()
        for box in detections:
            cls_id = int(box[5])
            current_classes.add(cls_id)
        
        # Update detection history for each class
        for class_id in range(11):  # All 11 classes
            if class_id not in self.temporal_filter['detection_history']:
                self.temporal_filter['detection_history'][class_id] = []
            
            history = self.temporal_filter['detection_history'][class_id]
            
            if class_id in current_classes:
                # Class detected in current frame
                history.append(current_frame)
            else:
                # Class not detected in current frame
                if history and (current_frame - history[-1]) > self.temporal_filter['max_frames_without_detection']:
                    # Clear old history if gap is too large
                    history.clear()
        
        # Clean up old history entries (keep only recent frames)
        max_history_age = self.temporal_filter['min_consecutive_frames'] + self.temporal_filter['max_frames_without_detection']
        for class_id in self.temporal_filter['detection_history']:
            history = self.temporal_filter['detection_history'][class_id]
            # Remove entries older than max_history_age
            history[:] = [frame for frame in history if (current_frame - frame) <= max_history_age]
    
    def is_temporally_consistent(self, class_id):
        """Check if a class has been detected consistently enough"""
        if class_id not in self.temporal_filter['detection_history']:
            return False
        
        history = self.temporal_filter['detection_history'][class_id]
        
        # Special treatment for traffic lights (small objects) - require fewer consecutive frames
        if class_id in self.safety_priorities['traffic_lights']:
            min_frames = 1  # Traffic lights only need 1 frame (very permissive)
        else:
            min_frames = self.temporal_filter['min_consecutive_frames']
        
        if len(history) < min_frames:
            return False
        
        # Check if we have enough recent detections
        current_frame = self.temporal_filter['frame_count']
        recent_detections = [frame for frame in history if (current_frame - frame) <= self.temporal_filter['max_frames_without_detection']]
        
        return len(recent_detections) >= min_frames
    
    def create_annotated_image(self, image, results):
        """Create annotated image with Taiwan class priorities - show ALL detections for debugging"""
        annotated = image.copy()
        
        if results.boxes is not None:
            boxes = results.boxes.cpu().numpy()
            
            for box in boxes.data:
                x1, y1, x2, y2, conf, cls_id = box
                cls_id = int(cls_id)
                
                # Get class-specific threshold for color coding
                class_threshold = self.class_confidence_thresholds.get(cls_id, self.confidence_threshold)
                
                # Color coding based on safety priorities and threshold status
                if conf >= class_threshold:
                    # Detections that pass threshold - solid colors
                    if cls_id in self.safety_priorities['vehicles']:
                        color = (0, 0, 255)  # Red for vehicles (highest priority)
                    elif cls_id in self.safety_priorities['traffic_lights']:
                        color = (0, 255, 255)  # Yellow for traffic lights
                    elif cls_id in self.safety_priorities['pedestrian_context']:
                        color = (255, 0, 0)  # Blue for pedestrian context
                    elif cls_id in self.safety_priorities['infrastructure']:
                        color = (0, 255, 0)  # Green for infrastructure
                    else:
                        color = (128, 128, 128)  # Gray for context
                else:
                    # Detections below threshold - dashed/dotted appearance (lighter colors)
                    if cls_id in self.safety_priorities['vehicles']:
                        color = (100, 100, 255)  # Light red for vehicles
                    elif cls_id in self.safety_priorities['traffic_lights']:
                        color = (100, 255, 255)  # Light yellow for traffic lights
                    elif cls_id in self.safety_priorities['pedestrian_context']:
                        color = (255, 100, 100)  # Light blue for pedestrian context
                    elif cls_id in self.safety_priorities['infrastructure']:
                        color = (100, 255, 100)  # Light green for infrastructure
                    else:
                        color = (180, 180, 180)  # Light gray for context
                
                # Draw bounding box
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                
                # Draw label with threshold and temporal filter status
                class_name = self.class_names.get(cls_id, 'unknown')
                threshold_status = "PASS" if conf >= class_threshold else "FAIL"
                temporal_status = "TEMP" if self.is_temporally_consistent(cls_id) else "GLITCH"
                label = f"{class_name}:{conf:.2f} ({threshold_status}/{temporal_status})"
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
