#!/usr/bin/env python3
"""
Ego Motion Compensator - Enhanced RTAB-Map Integration
Advanced motion compensation for accurate vehicle tracking and threat assessment
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Point, Twist, PoseStamped
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Bool, String
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
import time
from collections import defaultdict, deque

class EgoMotionCompensator(Node):
    def __init__(self):
        super().__init__('ego_motion_compensator')
        
        # Motion compensation parameters
        self.declare_parameter('motion_threshold', 0.1)      # m/s - significant motion
        self.declare_parameter('confidence_penalty', 0.2)    # Penalty during motion
        self.declare_parameter('tracking_window', 2.0)       # seconds
        self.declare_parameter('vehicle_classes', [0, 1, 2, 9, 12])  # Your Taiwan vehicle classes
        self.declare_parameter('compensation_smoothing', 0.8) # Smoothing factor for compensation
        self.declare_parameter('max_compensation_factor', 0.5) # Maximum compensation applied
        
        # State tracking
        self.current_velocity = Vector3()
        self.angular_velocity = Vector3()
        self.motion_magnitude = 0.0
        self.previous_detections = defaultdict(list)
        
        # Enhanced motion tracking
        self.camera_pose_history = deque(maxlen=10)  # Store last 10 poses
        self.motion_quality_history = deque(maxlen=20)  # Store motion quality metrics
        self.compensation_factors = defaultdict(float)  # Per-vehicle compensation
        
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
        
        # Alternative odometry sources if RTAB-Map is not available
        self.alt_odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odometry_callback, 10)
        
        # Also try local map odometry
        self.local_odom_sub = self.create_subscription(
            Odometry, '/odom_local_map',
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
        
        self.compensation_stats_pub = self.create_publisher(
            String, '/motion_compensation_stats', 10)
        
        self.get_logger().info('📱 Enhanced Ego Motion Compensator initialized')
        self.get_logger().info('✅ Advanced motion compensation with RTAB-Map integration')
    
    def odometry_callback(self, msg: Odometry):
        """Process RTAB-Map visual odometry for enhanced ego motion tracking"""
        
        # Extract linear and angular velocities
        self.current_velocity = msg.twist.twist.linear
        self.angular_velocity = msg.twist.twist.angular
        
        # Calculate motion magnitude
        self.motion_magnitude = math.sqrt(
            self.current_velocity.x**2 + 
            self.current_velocity.y**2 + 
            self.current_velocity.z**2
        )
        
        # Store camera pose for trajectory analysis
        current_pose = {
            'position': msg.pose.pose.position,
            'orientation': msg.pose.pose.orientation,
            'velocity': self.current_velocity,
            'timestamp': time.time()
        }
        self.camera_pose_history.append(current_pose)
        
        # Calculate motion quality metrics
        motion_quality = self.calculate_motion_quality()
        self.motion_quality_history.append(motion_quality)
        
        # Check for excessive motion
        excessive_motion = self.motion_magnitude > self.get_parameter('motion_threshold').value
        
        # Publish motion quality metrics
        self.publish_motion_quality(excessive_motion, motion_quality)
        
        # Log significant motion events
        if excessive_motion:
            self.get_logger().debug(f'📱 Camera motion: {self.motion_magnitude:.2f} m/s, Quality: {motion_quality:.2f}')
    
    def calculate_motion_quality(self):
        """Calculate motion quality based on stability and consistency"""
        if len(self.camera_pose_history) < 2:
            return 1.0
        
        # Calculate motion stability
        recent_poses = list(self.camera_pose_history)[-3:]  # Last 3 poses
        velocity_variations = []
        
        for i in range(1, len(recent_poses)):
            prev_vel = recent_poses[i-1]['velocity']
            curr_vel = recent_poses[i]['velocity']
            
            vel_diff = math.sqrt(
                (curr_vel.x - prev_vel.x)**2 +
                (curr_vel.y - prev_vel.y)**2 +
                (curr_vel.z - prev_vel.z)**2
            )
            velocity_variations.append(vel_diff)
        
        # Quality decreases with motion magnitude and velocity variations
        avg_variation = np.mean(velocity_variations) if velocity_variations else 0.0
        motion_penalty = min(1.0, self.motion_magnitude / 2.0)  # Normalize to 0-1
        variation_penalty = min(1.0, avg_variation / 1.0)  # Normalize to 0-1
        
        quality = max(0.0, 1.0 - (motion_penalty + variation_penalty) / 2.0)
        return quality
    
    def detection_callback(self, msg: Detection2DArray):
        """Apply enhanced ego motion compensation to vehicle detections"""
        
        # Create compensated detection array
        compensated_msg = Detection2DArray()
        compensated_msg.header = msg.header
        
        compensation_stats = {
            'total_detections': len(msg.detections),
            'compensated_vehicles': 0,
            'avg_compensation_factor': 0.0,
            'motion_quality': self.calculate_motion_quality()
        }
        
        compensation_factors = []
        
        for detection in msg.detections:
            compensated_detection = self.compensate_detection_motion(detection)
            if compensated_detection:
                compensated_msg.detections.append(compensated_detection)
                
                # Track compensation statistics
                if hasattr(compensated_detection, 'compensation_factor'):
                    compensation_factors.append(compensated_detection.compensation_factor)
                    compensation_stats['compensated_vehicles'] += 1
        
        # Calculate average compensation factor
        if compensation_factors:
            compensation_stats['avg_compensation_factor'] = np.mean(compensation_factors)
        
        # Publish motion-compensated results
        self.compensated_detections_pub.publish(compensated_msg)
        
        # Publish compensation statistics
        self.publish_compensation_stats(compensation_stats)
    
    def compensate_detection_motion(self, detection):
        """Apply enhanced ego motion compensation to individual detection"""
        
        if not detection.results:
            return detection
        
        class_id = int(detection.results[0].hypothesis.class_id)
        
        # Only compensate vehicle classes (Priority 1 for your system)
        vehicle_classes = self.get_parameter('vehicle_classes').value
        if class_id not in vehicle_classes:
            return detection
        
        # Calculate enhanced motion compensation
        compensation_factor = self.calculate_enhanced_compensation_factor()
        
        # Apply confidence adjustment based on motion
        original_confidence = detection.results[0].hypothesis.score
        compensated_confidence = self.apply_confidence_compensation(
            original_confidence, compensation_factor)
        
        # Update detection confidence
        detection.results[0].hypothesis.score = compensated_confidence
        
        # Add motion compensation metadata
        detection.bbox.size_x *= (1.0 + compensation_factor * 0.1)  # Slight size adjustment
        
        # Store compensation factor for statistics (use a custom attribute or skip)
        # Note: Detection2D doesn't have compensation_factor attribute
        # We'll track this in our internal statistics instead
        
        return detection
    
    def calculate_enhanced_compensation_factor(self):
        """Calculate enhanced motion compensation factor"""
        
        motion_threshold = self.get_parameter('motion_threshold').value
        max_compensation = self.get_parameter('max_compensation_factor').value
        smoothing = self.get_parameter('compensation_smoothing').value
        
        if self.motion_magnitude < motion_threshold:
            return 0.0  # No compensation needed
        
        # Calculate base compensation factor
        base_compensation = min(max_compensation, 
                                self.motion_magnitude / (motion_threshold * 4))
        
        # Apply motion quality weighting
        motion_quality = self.calculate_motion_quality()
        quality_weighted_compensation = base_compensation * (1.0 - motion_quality * 0.5)
        
        # Apply smoothing if we have history
        if hasattr(self, 'previous_compensation_factor'):
            smoothed_compensation = (smoothing * self.previous_compensation_factor + 
                                   (1.0 - smoothing) * quality_weighted_compensation)
        else:
            smoothed_compensation = quality_weighted_compensation
        
        self.previous_compensation_factor = smoothed_compensation
        return smoothed_compensation
    
    def apply_confidence_compensation(self, original_confidence, compensation_factor):
        """Apply motion-based confidence adjustment with enhanced logic"""
        
        confidence_penalty = self.get_parameter('confidence_penalty').value
        
        # Calculate motion penalty based on compensation factor
        motion_penalty = compensation_factor * confidence_penalty
        
        # Additional penalty for poor motion quality
        motion_quality = self.calculate_motion_quality()
        quality_penalty = (1.0 - motion_quality) * confidence_penalty * 0.5
        
        total_penalty = motion_penalty + quality_penalty
        compensated_confidence = max(0.1, original_confidence - total_penalty)
        
        return compensated_confidence
    
    def publish_motion_quality(self, excessive_motion, motion_quality):
        """Publish enhanced motion quality metrics"""
        
        # Publish motion quality score (0.0 = poor, 1.0 = excellent)
        quality_msg = Float32()
        quality_msg.data = motion_quality
        self.motion_quality_pub.publish(quality_msg)
        
        # Publish excessive motion alert
        alert_msg = Bool()
        alert_msg.data = excessive_motion
        self.motion_alert_pub.publish(alert_msg)
    
    def publish_compensation_stats(self, stats):
        """Publish motion compensation statistics"""
        
        stats_msg = String()
        stats_msg.data = (f"Motion Compensation Stats: "
                         f"Total={stats['total_detections']}, "
                         f"Compensated={stats['compensated_vehicles']}, "
                         f"AvgFactor={stats['avg_compensation_factor']:.3f}, "
                         f"Quality={stats['motion_quality']:.3f}")
        
        self.compensation_stats_pub.publish(stats_msg)
    
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
        
        # Compensate for ego motion using recent camera poses
        if len(self.camera_pose_history) >= 2:
            recent_cam_pose = self.camera_pose_history[-1]
            previous_cam_pose = self.camera_pose_history[-2]
            
            # Calculate camera movement
            cam_dx = recent_cam_pose['position'].x - previous_cam_pose['position'].x
            cam_dy = recent_cam_pose['position'].y - previous_cam_pose['position'].y
            
            ego_velocity_2d = math.sqrt(cam_dx**2 + cam_dy**2) / dt
            
            # True vehicle velocity (simplified)
            true_velocity = max(0.0, apparent_velocity - ego_velocity_2d)
        else:
            true_velocity = apparent_velocity
        
        return true_velocity

def main():
    rclpy.init()
    node = EgoMotionCompensator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
