#!/usr/bin/env python3
"""
Vehicle Movement Analyzer - Priority 1 Safety Component
Enhanced with full 3D depth-aware detection and ego-motion compensation
Updated for 11-class model: Only motorized vehicles are safety threats
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import defaultdict
import time
import math
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class VehicleMovementAnalyzer(Node):
    def __init__(self):
        super().__init__('vehicle_movement_analyzer')
        
        # NEW: 11-class motorized vehicle classes (Priority 1 Safety)
        self.vehicle_classes = {
            1: 'bus',        # Class 1 - Large vehicles
            2: 'car',        # Class 2 - Standard vehicles  
            5: 'motorcycle', # Class 5 - Fast vehicles
            9: 'truck'       # Class 9 - Heavy vehicles
        }
        
        # NOTE: Bicycles (Class 0) and pedestrians (Class 6) are now context classes
        # They cross together and don't trigger vehicle safety overrides
        
        # Safety parameters
        self.declare_parameter('horizontal_threat_distance', 15.0)  # meters
        self.declare_parameter('vehicle_confidence_threshold', 0.6)
        self.declare_parameter('car_specific_threshold', 0.4)       # Lower threshold for cars
        self.declare_parameter('motion_velocity_threshold', 2.0)    # m/s
        self.declare_parameter('ttc_safety_threshold', 4.0)        # seconds - time to collision
        self.declare_parameter('tracking_window', 2.0)             # seconds for velocity calculation
        
        # RealSense D435 camera intrinsics (from your config)
        self.camera_intrinsics = {
            'fx': 617.0, 'fy': 617.0,  # Focal lengths
            'cx': 320.0, 'cy': 240.0   # Principal point
        }
        
        # State tracking
        self.current_depth_image = None
        self.vehicle_tracks = defaultdict(list)  # Track 3D positions over time
        self.bridge = CvBridge()
        
        # Ego-motion tracking
        self.current_camera_pose = None
        self.previous_camera_pose = None
        self.camera_pose_timestamp = None
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10)
        
        # Ego-motion from RTAB-Map
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom',
            self.odometry_callback, 10)
        
        # Publishers  
        self.immediate_danger_pub = self.create_publisher(
            Bool, '/immediate_crossing_danger', 10)
        
        self.vehicle_threat_pub = self.create_publisher(
            String, '/vehicle_threat_status', 10)
        
        self.ttc_pub = self.create_publisher(
            Float32, '/time_to_collision', 10)
        
        self.vehicle_velocity_pub = self.create_publisher(
            Float32, '/vehicle_relative_velocity', 10)
        
        self.get_logger().info('🚗 Enhanced Vehicle Movement Analyzer initialized (Priority 1)')
        self.get_logger().info('✅ Full 3D depth-aware detection with ego-motion compensation')
    
    def depth_callback(self, msg):
        """Store depth image for 3D position calculation"""
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
    
    def odometry_callback(self, msg: Odometry):
        """Track camera ego-motion for compensation"""
        self.previous_camera_pose = self.current_camera_pose
        self.current_camera_pose = {
            'position': msg.pose.pose.position,
            'orientation': msg.pose.pose.orientation,
            'timestamp': time.time()
        }
        self.camera_pose_timestamp = time.time()
    
    def detection_callback(self, msg: Detection2DArray):
        """Enhanced detection callback with 3D projection and ego-motion compensation"""
        if self.current_depth_image is None:
            return
        
        current_time = time.time()
        
        # Extract vehicle detections with 3D projection
        vehicle_detections = []
        for detection in msg.detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # Check if vehicle class with sufficient confidence
            if (class_id in self.vehicle_classes and 
                confidence >= self.get_parameter('vehicle_confidence_threshold').value):
                
                # Project 2D detection to 3D using depth and camera intrinsics
                position_3d = self.project_to_3d(detection.bbox)
                
                if position_3d is not None:
                    vehicle_data = {
                    'class_id': class_id,
                    'class_name': self.vehicle_classes[class_id],
                    'confidence': confidence,
                        'bbox': detection.bbox,
                        'position_3d': position_3d,
                        'timestamp': current_time
                    }
                    vehicle_detections.append(vehicle_data)
        
        # Update vehicle tracking with ego-motion compensation
        self.update_vehicle_tracking(vehicle_detections, current_time)
        
        # Analyze for threats using TTC and relative motion
        immediate_danger, ttc_info = self.analyze_enhanced_threats(vehicle_detections)
        
        # Publish results
        danger_msg = Bool()
        danger_msg.data = immediate_danger
        self.immediate_danger_pub.publish(danger_msg)
        
        # Publish TTC information
        if ttc_info:
            ttc_msg = Float32()
            ttc_msg.data = ttc_info['min_ttc']
            self.ttc_pub.publish(ttc_msg)
            
            velocity_msg = Float32()
            velocity_msg.data = ttc_info['max_velocity']
            self.vehicle_velocity_pub.publish(velocity_msg)
        
        # Publish threat status
        threat_msg = String()
        if immediate_danger:
            threat_msg.data = f"IMMEDIATE_DANGER: TTC={ttc_info['min_ttc']:.1f}s, Vel={ttc_info['max_velocity']:.1f}m/s"
        else:
            threat_msg.data = f"SAFE: {len(vehicle_detections)} vehicles monitored, TTC>{self.get_parameter('ttc_safety_threshold').value}s"
        self.vehicle_threat_pub.publish(threat_msg)
        
        if immediate_danger:
            self.get_logger().warn(f'🚨 IMMEDIATE DANGER: TTC={ttc_info["min_ttc"]:.1f}s, Vehicle={ttc_info["threatening_vehicle"]}')
    
    def project_to_3d(self, bbox):
        """Project 2D bounding box center to 3D using depth and camera intrinsics"""
        try:
            # Get bounding box center
            center_x = int(bbox.center.position.x)
            center_y = int(bbox.center.position.y)
            
            # Check if position is valid
            if (0 <= center_y < self.current_depth_image.shape[0] and 
                0 <= center_x < self.current_depth_image.shape[1]):
                
                # Get depth value
                depth_mm = self.current_depth_image[center_y, center_x]
                if depth_mm == 0:  # Invalid depth
                    return None
                
                depth_m = depth_mm / 1000.0  # Convert to meters
                
                # Project to 3D using camera intrinsics
                fx = self.camera_intrinsics['fx']
                fy = self.camera_intrinsics['fy']
                cx = self.camera_intrinsics['cx']
                cy = self.camera_intrinsics['cy']
                
                # 3D projection formula
                X = (center_x - cx) * depth_m / fx
                Y = (center_y - cy) * depth_m / fy
                Z = depth_m
                
                return Point(x=X, y=Y, z=Z)
            
        except Exception as e:
            self.get_logger().warn(f'3D projection failed: {e}')
        
        return None
    
    def update_vehicle_tracking(self, vehicle_detections, current_time):
        """Update vehicle tracking with ego-motion compensation"""
        tracking_window = self.get_parameter('tracking_window').value
        
        for vehicle in vehicle_detections:
            vehicle_id = f"{vehicle['class_id']}_{vehicle['position_3d'].x:.1f}_{vehicle['position_3d'].y:.1f}"
            
            # Add current detection to tracking history
            self.vehicle_tracks[vehicle_id].append({
                'position_3d': vehicle['position_3d'],
                'timestamp': current_time,
                'class_name': vehicle['class_name'],
                'confidence': vehicle['confidence']
            })
            
            # Remove old entries outside tracking window
            self.vehicle_tracks[vehicle_id] = [
                track for track in self.vehicle_tracks[vehicle_id]
                if current_time - track['timestamp'] <= tracking_window
            ]
    
    def calculate_relative_velocity(self, vehicle_track):
        """Calculate relative velocity with ego-motion compensation"""
        if len(vehicle_track) < 2:
            return 0.0
        
        # Get recent positions
        current = vehicle_track[-1]
        previous = vehicle_track[-2]
        
        dt = current['timestamp'] - previous['timestamp']
        if dt <= 0:
            return 0.0
        
        # Calculate apparent movement
        dx = current['position_3d'].x - previous['position_3d'].x
        dy = current['position_3d'].y - previous['position_3d'].y
        dz = current['position_3d'].z - previous['position_3d'].z
        
        apparent_velocity = math.sqrt(dx**2 + dy**2 + dz**2) / dt
        
        # Compensate for ego-motion if available
        if self.current_camera_pose and self.previous_camera_pose:
            # Calculate camera movement
            cam_dx = self.current_camera_pose['position'].x - self.previous_camera_pose['position'].x
            cam_dy = self.current_camera_pose['position'].y - self.previous_camera_pose['position'].y
            cam_dz = self.current_camera_pose['position'].z - self.previous_camera_pose['position'].z
            
            ego_velocity = math.sqrt(cam_dx**2 + cam_dy**2 + cam_dz**2) / dt
            
            # Relative velocity = apparent - ego
            relative_velocity = max(0.0, apparent_velocity - ego_velocity)
        else:
            relative_velocity = apparent_velocity
        
        return relative_velocity
    
    def analyze_enhanced_threats(self, vehicle_detections):
        """Enhanced threat analysis using TTC and relative motion"""
        immediate_danger = False
        min_ttc = float('inf')
        max_velocity = 0.0
        threatening_vehicle = None
        
        ttc_threshold = self.get_parameter('ttc_safety_threshold').value
        
        for vehicle in vehicle_detections:
            vehicle_id = f"{vehicle['class_id']}_{vehicle['position_3d'].x:.1f}_{vehicle['position_3d'].y:.1f}"
            
            if vehicle_id in self.vehicle_tracks:
                # Calculate relative velocity
                relative_velocity = self.calculate_relative_velocity(self.vehicle_tracks[vehicle_id])
                max_velocity = max(max_velocity, relative_velocity)
                
                # Calculate distance to crossing path (simplified - using Z distance)
                distance_to_crossing = abs(vehicle['position_3d'].z)
                
                # Calculate time to collision
                if relative_velocity > 0.1:  # Minimum velocity threshold
                    ttc = distance_to_crossing / relative_velocity
                    min_ttc = min(min_ttc, ttc)
                    
                    # Check if this vehicle is a threat
                    if ttc < ttc_threshold and relative_velocity > self.get_parameter('motion_velocity_threshold').value:
                        immediate_danger = True
                        threatening_vehicle = vehicle['class_name']
                        
                        self.get_logger().debug(f'🚗 Vehicle {vehicle["class_name"]}: TTC={ttc:.1f}s, Vel={relative_velocity:.1f}m/s, Dist={distance_to_crossing:.1f}m')
        
        ttc_info = {
            'min_ttc': min_ttc if min_ttc != float('inf') else 999.0,
            'max_velocity': max_velocity,
            'threatening_vehicle': threatening_vehicle
        }
        
        return immediate_danger, ttc_info

def main():
    rclpy.init()
    node = VehicleMovementAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
