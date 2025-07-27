#!/usr/bin/env python3
"""
Taiwan 17-Class Traffic Safety YOLOv8 Camera Node
Integrates with RealSense D435 for real-time traffic detection
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
import time
from pathlib import Path

class YOLOv8CameraNode(Node):
    def __init__(self):
        super().__init__('yolov8_camera_node')
        
        # Taiwan 17-class mapping (CORRECTED)
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
        
        # Parameters (using consistent naming)
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
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/camera/detections', 10)
        
        if self.publish_images:
            self.image_pub = self.create_publisher(
                Image, '/camera/detections/image', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        
        # Performance tracking
        self.inference_times = []
        self.frame_count = 0
        
        # Initialize model
        if self.initialize_model():
            self.get_logger().info('✅ Taiwan Traffic Detection Node Ready')
            self.get_logger().info(f'🇹🇼 17-class Taiwan model loaded: {self.model_path}')
        else:
            self.get_logger().error('❌ Failed to initialize Taiwan model')
        
        # Performance logging timer
        self.create_timer(5.0, self.log_performance)
    
    def initialize_model(self):
        """Initialize Taiwan 17-class YOLO model"""
        try:
            if not Path(self.model_path).exists():
                self.get_logger().error(f'Model file not found: {self.model_path}')
                return False
            
            self.get_logger().info(f'Loading Taiwan model: {self.model_path}')
            self.model = YOLO(self.model_path)
            
            # Verify class count
            if len(self.model.names) != 17:
                self.get_logger().warning(f'Expected 17 classes, got {len(self.model.names)}')
            
            # Log Taiwan-specific classes for verification
            self.get_logger().info('Taiwan spatial classification classes:')
            for idx in self.taiwan_priorities['critical_crossing']:
                if idx < len(self.class_names):
                    self.get_logger().info(f'  {idx}: {self.class_names[idx]}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Model initialization failed: {e}')
            return False
    
    def image_callback(self, msg: Image):
        """Process camera images with Taiwan traffic detection"""
        if self.model is None:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run Taiwan traffic detection
            start_time = time.time()
            results = self.model(cv_image, 
                               conf=self.confidence_threshold,
                               iou=self.nms_threshold,
                               max_det=self.max_detections,
                               verbose=False)
            
            inference_time = (time.time() - start_time) * 1000
            self.inference_times.append(inference_time)
            self.frame_count += 1
            
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
            self.get_logger().error(f'Image processing failed: {e}')
    
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
            
            # Classification result - FIXED: Convert class_id to string
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(cls_id)  # ROS expects string
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
        """Log performance statistics"""
        if not self.inference_times:
            return
        
        recent_times = self.inference_times[-50:]  # Last 50 frames
        avg_time = np.mean(recent_times)
        fps = 1000.0 / avg_time if avg_time > 0 else 0
        
        self.get_logger().info(f'Taiwan Detection Performance: {avg_time:.1f}ms avg, {fps:.1f} FPS')
        
        # Check real-time performance
        if avg_time < 33.0:  # 30 FPS requirement
            self.get_logger().info('✅ Meeting real-time requirements')
        else:
            self.get_logger().warning('⚠️ Below real-time performance')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
