#!/usr/bin/env python3
"""
Vehicle Movement Analyzer - Priority 1 Safety Component
Detects horizontal vehicle movement for immediate crossing danger
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import defaultdict
import time

class VehicleMovementAnalyzer(Node):
    def __init__(self):
        super().__init__('vehicle_movement_analyzer')
        
        # Taiwan vehicle classes from your 17-class model
        self.vehicle_classes = {
            0: 'bicycle',    # Class 0
            1: 'bus',        # Class 1  
            2: 'car',        # Class 2 - Your 83.1% mAP50 reliable detection
            9: 'motorcycle', # Class 9 - Very common in Taiwan
            12: 'truck'      # Class 12
        }
        
        # Safety parameters
        self.declare_parameter('horizontal_threat_distance', 15.0)  # meters
        self.declare_parameter('vehicle_confidence_threshold', 0.6)
        self.declare_parameter('motion_velocity_threshold', 2.0)    # m/s
        
        # State tracking
        self.current_depth_image = None
        self.vehicle_tracks = defaultdict(list)
        self.bridge = CvBridge()
        
        # Subscriptions
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10)
        
        # Publishers  
        self.immediate_danger_pub = self.create_publisher(
            Bool, '/immediate_crossing_danger', 10)
        
        self.vehicle_threat_pub = self.create_publisher(
            String, '/vehicle_threat_status', 10)
        
        self.get_logger().info('🚗 Vehicle Movement Analyzer initialized (Priority 1)')
    
    def depth_callback(self, msg):
        """Store depth image for 3D position calculation"""
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
    
    def detection_callback(self, msg: Detection2DArray):
        """Analyze vehicle detections for horizontal movement threats"""
        if self.current_depth_image is None:
            return
        
        # Extract vehicle detections
        vehicle_detections = []
        for detection in msg.detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # Check if vehicle class with sufficient confidence
            if (class_id in self.vehicle_classes and 
                confidence >= self.get_parameter('vehicle_confidence_threshold').value):
                
                vehicle_detections.append({
                    'class_id': class_id,
                    'class_name': self.vehicle_classes[class_id],
                    'confidence': confidence,
                    'bbox': detection.bbox
                })
        
        # Analyze for immediate horizontal movement danger
        immediate_danger = self.analyze_horizontal_movement_danger(vehicle_detections)
        
        # Publish results
        danger_msg = Bool()
        danger_msg.data = immediate_danger
        self.immediate_danger_pub.publish(danger_msg)
        
        # Publish threat status
        threat_msg = String()
        if immediate_danger:
            threat_msg.data = "IMMEDIATE_DANGER: Horizontal vehicle movement detected"
        else:
            threat_msg.data = f"SAFE: {len(vehicle_detections)} vehicles monitored"
        self.vehicle_threat_pub.publish(threat_msg)
        
        if immediate_danger:
            self.get_logger().warn('🚨 IMMEDIATE DANGER: Vehicle crossing path!')
    
    def analyze_horizontal_movement_danger(self, vehicles):
        """Check for vehicles moving horizontally across crossing path"""
        for vehicle in vehicles:
            # Get 3D position using depth
            bbox = vehicle['bbox']
            center_x = int(bbox.center.position.x)
            center_y = int(bbox.center.position.y)
            
            # Check if position is valid
            if (0 <= center_y < self.current_depth_image.shape[0] and 
                0 <= center_x < self.current_depth_image.shape[1]):
                
                depth_value = self.current_depth_image[center_y, center_x] / 1000.0  # Convert to meters
                
                # If vehicle within threat distance - immediate danger
                if 0 < depth_value <= self.get_parameter('horizontal_threat_distance').value:
                    # High-threat vehicles trigger immediate stop
                    if vehicle['class_name'] in ['car', 'bus', 'motorcycle', 'truck']:
                        return True
        
        return False

def main():
    rclpy.init()
    node = VehicleMovementAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
