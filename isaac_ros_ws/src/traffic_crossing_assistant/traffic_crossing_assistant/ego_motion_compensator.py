#!/usr/bin/env python3
"""
Ego Motion Compensator - RTAB-Map Integration
Removes camera movement effects from vehicle tracking for accurate threat assessment
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Point, Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Bool
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
import time
from collections import defaultdict

class EgoMotionCompensator(Node):
    def __init__(self):
        super().__init__('ego_motion_compensator')
        
        # Motion compensation parameters
        self.declare_parameter('motion_threshold', 0.1)      # m/s - significant motion
        self.declare_parameter('confidence_penalty', 0.2)    # Penalty during motion
        self.declare_parameter('tracking_window', 2.0)       # seconds
        self.declare_parameter('vehicle_classes', [0, 1, 2, 9, 12])  # Your Taiwan vehicle classes
        
        # State tracking
        self.current_velocity = Vector3()
        self.angular_velocity = Vector3()
        self.motion_magnitude = 0.0
        self.previous_detections = defaultdict(list)
        
        # Camera parameters (RealSense D435)
        self.camera_intrinsics = {
            'fx': 617.0, 'fy': 617.0,  # Focal lengths
            'cx': 320.0, 'cy': 240.0   # Principal point
        }
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom',
            self.odometry_callback, 10)
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        # Publishers
        self.compensated_detections_pub = self.create_publisher(
            Detection2DArray, '/detections/motion_compensated', 10)
        
        self.motion_quality_pub = self.create_publisher(
            Float32, '/motion_compensation_quality', 10)
        
        self.motion_alert_pub = self.create_publisher(
            Bool, '/excessive_motion_detected', 10)
        
        self.get_logger().info('📱 Ego Motion Compensator initialized')
        self.get_logger().info('Using RTAB-Map visual odometry for motion compensation')
    
    def odometry_callback(self, msg: Odometry):
        """Process RTAB-Map visual odometry for ego motion tracking"""
        
        # Extract linear and angular velocities
        self.current_velocity = msg.twist.twist.linear
        self.angular_velocity = msg.twist.twist.angular
        
        # Calculate motion magnitude
        self.motion_magnitude = math.sqrt(
            self.current_velocity.x**2 + 
            self.current_velocity.y**2 + 
            self.current_velocity.z**2
        )
        
        # Check for excessive motion
        excessive_motion = self.motion_magnitude > self.get_parameter('motion_threshold').value
        
        # Publish motion quality metrics
        self.publish_motion_quality(excessive_motion)
        
        # Log significant motion events
        if excessive_motion:
            self.get_logger().debug(f'📱 Camera motion: {self.motion_magnitude:.2f} m/s')
    
    def detection_callback(self, msg: Detection2DArray):
        """Apply ego motion compensation to vehicle detections"""
        
        # Create compensated detection array
        compensated_msg = Detection2DArray()
        compensated_msg.header = msg.header
        
        for detection in msg.detections:
            compensated_detection = self.compensate_detection_motion(detection)
            if compensated_detection:
                compensated_msg.detections.append(compensated_detection)
        
        # Publish motion-compensated results
        self.compensated_detections_pub.publish(compensated_msg)
    
    def compensate_detection_motion(self, detection):
        """Apply ego motion compensation to individual detection"""
        
        if not detection.results:
            return detection
        
        class_id = int(detection.results[0].hypothesis.class_id)
        
        # Only compensate vehicle classes (Priority 1 for your system)
        vehicle_classes = self.get_parameter('vehicle_classes').value
        if class_id not in vehicle_classes:
            return detection
        
        # Calculate motion compensation
        compensation_factor = self.calculate_motion_compensation_factor()
        
        # Apply confidence adjustment based on motion
        original_confidence = detection.results[0].hypothesis.score
        compensated_confidence = self.apply_confidence_compensation(
            original_confidence, compensation_factor)
        
        # Update detection confidence
        detection.results[0].hypothesis.score = compensated_confidence
        
        # Add motion compensation metadata (for debugging)
        detection.bbox.size_x *= (1.0 + compensation_factor * 0.1)  # Slight size adjustment
        
        return detection
    
    def calculate_motion_compensation_factor(self):
        """Calculate motion compensation factor based on current motion"""
        
        motion_threshold = self.get_parameter('motion_threshold').value
        
        if self.motion_magnitude < motion_threshold:
            return 0.0  # No compensation needed
        
        # Linear scaling: higher motion = higher compensation factor
        max_compensation = 0.5  # Maximum 50% compensation
        compensation_factor = min(max_compensation, 
                                self.motion_magnitude / (motion_threshold * 4))
        
        return compensation_factor
    
    def apply_confidence_compensation(self, original_confidence, compensation_factor):
        """Apply motion-based confidence adjustment"""
        
        confidence_penalty = self.get_parameter('confidence_penalty').value
        
        # Reduce confidence during motion (conservative approach)
        motion_penalty = compensation_factor * confidence_penalty
        compensated_confidence = max(0.1, original_confidence - motion_penalty)
        
        return compensated_confidence
    
    def publish_motion_quality(self, excessive_motion):
        """Publish motion quality metrics"""
        
        # Publish motion quality score (0.0 = poor, 1.0 = excellent)
        quality_score = max(0.0, 1.0 - (self.motion_magnitude / 2.0))
        quality_msg = Float32()
        quality_msg.data = quality_score
        self.motion_quality_pub.publish(quality_msg)
        
        # Publish excessive motion alert
        alert_msg = Bool()
        alert_msg.data = excessive_motion
        self.motion_alert_pub.publish(alert_msg)
    
    def calculate_vehicle_true_velocity(self, detection_history):
        """Calculate true vehicle velocity removing ego motion effects"""
        
        if len(detection_history) < 2:
            return 0.0
        
        # Get recent detection positions
        recent = detection_history[-1]
        previous = detection_history[-2]
        
        # Calculate apparent movement
        dx = recent['position'].x - previous['position'].x
        dy = recent['position'].y - previous['position'].y
        dt = recent['timestamp'] - previous['timestamp']
        
        if dt <= 0:
            return 0.0
        
        # Calculate apparent velocity
        apparent_velocity = math.sqrt(dx**2 + dy**2) / dt
        
        # Compensate for ego motion
        ego_velocity_2d = math.sqrt(
            self.current_velocity.x**2 + self.current_velocity.y**2)
        
        # True vehicle velocity (simplified)
        true_velocity = max(0.0, apparent_velocity - ego_velocity_2d)
        
        return true_velocity

def main():
    rclpy.init()
    node = EgoMotionCompensator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
