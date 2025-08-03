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
        
        # Motorized vehicle classes (Priority 1 Safety - for threat analysis)
        self.vehicle_classes = {
            1: 'bus',        # Class 1 - Large vehicles
            2: 'car',        # Class 2 - Standard vehicles  
            5: 'motorcycle', # Class 5 - Fast vehicles
            9: 'truck'       # Class 9 - Heavy vehicles
        }
        
        # Safety parameters
        self.declare_parameter('horizontal_threat_distance', 15.0)  # meters
        self.declare_parameter('motion_velocity_threshold', 2.0)    # m/s
        self.declare_parameter('ttc_safety_threshold', 4.0)        # seconds - time to collision
        self.declare_parameter('tracking_window', 2.0)             # seconds for velocity calculation
        
        # Vehicle-specific confidence thresholds (matching YOLOv8 camera node exactly)
        self.vehicle_confidence_thresholds = {
            1: 0.55,   # bus - lowered for temporal filtering
            2: 0.55,   # car - lowered for temporal filtering
            5: 0.65,   # motorcycle - lowered for temporal filtering
            9: 0.7     # truck - lowered for temporal filtering
        }
        
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
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
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
        
        # Debug publisher
        self.debug_pub = self.create_publisher(
            String, '/vehicle_analyzer_debug', 10)
        
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
        use_2d_fallback = False
        
        if self.current_depth_image is None:
            debug_msg = String()
            debug_msg.data = "⚠️ No depth image available - using 2D fallback analysis"
            self.debug_pub.publish(debug_msg)
            use_2d_fallback = True
        else:
            # Check if depth image has valid data
            valid_depth_pixels = np.count_nonzero(self.current_depth_image)
            total_pixels = self.current_depth_image.shape[0] * self.current_depth_image.shape[1]
            valid_ratio = valid_depth_pixels / total_pixels
            
            if valid_ratio < 0.1:  # Less than 10% valid depth pixels
                debug_msg = String()
                debug_msg.data = f"⚠️ Insufficient valid depth data ({valid_ratio:.1%}) - using 2D fallback analysis"
                self.debug_pub.publish(debug_msg)
                use_2d_fallback = True
        
        current_time = time.time()
        
        # Extract vehicle detections with 3D projection
        vehicle_detections = []
        total_detections = len(msg.detections)
        vehicle_detections_found = 0
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # Debug: Log vehicle detections only
            if class_id in self.vehicle_classes:
                debug_msg = String()
                debug_msg.data = f'🚗 Vehicle detected: {self.vehicle_classes[class_id]} (Class {class_id}), Confidence: {confidence:.3f}'
                self.debug_pub.publish(debug_msg)
                vehicle_detections_found += 1
            
            # Check if vehicle class with sufficient confidence (using vehicle-specific thresholds)
            threshold = self.vehicle_confidence_thresholds.get(class_id, 0.6)
            if (class_id in self.vehicle_classes and confidence >= threshold):
                debug_msg = String()
                debug_msg.data = f'✅ Vehicle passed threshold: {self.vehicle_classes[class_id]} (conf: {confidence:.3f} >= {threshold:.3f})'
                self.debug_pub.publish(debug_msg)
                
                if use_2d_fallback:
                    # Use 2D analysis when depth is unavailable
                    threat_level = self.analyze_2d_threat(detection.bbox, class_id, confidence)
                    debug_msg = String()
                    debug_msg.data = f'🎯 2D Threat Analysis: {self.vehicle_classes[class_id]} threat={threat_level:.3f} (threshold: 0.3)'
                    self.debug_pub.publish(debug_msg)
                    
                    # Always add vehicle to detections for monitoring, regardless of threat level
                    vehicle_data = {
                        'class_id': class_id,
                        'class_name': self.vehicle_classes[class_id],
                        'confidence': confidence,
                        'bbox': detection.bbox,
                        'position_3d': None,  # No 3D data
                        'threat_level': threat_level,
                        'timestamp': current_time
                    }
                    vehicle_detections.append(vehicle_data)
                else:
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
                    else:
                        # 3D projection failed, fall back to 2D analysis
                        debug_msg = String()
                        debug_msg.data = f"🔄 3D projection failed for {self.vehicle_classes[class_id]} - falling back to 2D analysis"
                        self.debug_pub.publish(debug_msg)
                        
                        threat_level = self.analyze_2d_threat(detection.bbox, class_id, confidence)
                        if threat_level > 0.3:  # Only consider significant threats
                            vehicle_data = {
                                'class_id': class_id,
                                'class_name': self.vehicle_classes[class_id],
                                'confidence': confidence,
                                'bbox': detection.bbox,
                                'position_3d': None,  # No 3D data
                                'threat_level': threat_level,
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
        
        # Debug: Log what we're publishing with timestamp
        current_time = time.time()
        debug_msg = String()
        debug_msg.data = f'📤 PUBLISHING: immediate_danger={immediate_danger}, ttc={ttc_info["min_ttc"]:.1f}s at {current_time:.3f}'
        self.debug_pub.publish(debug_msg)
        
        # Also log to ROS logger for immediate visibility
        if immediate_danger:
            self.get_logger().warn(f'🚨 PUBLISHING VEHICLE THREAT: immediate_danger=True at {current_time:.3f}')
        else:
            self.get_logger().info(f'✅ PUBLISHING SAFE: immediate_danger=False at {current_time:.3f}')
        
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
        
        # Debug: Log summary
        if vehicle_detections_found > 0:
            analysis_type = "2D fallback" if use_2d_fallback else "3D projection"
            debug_msg = String()
            debug_msg.data = f'📊 Vehicle Analysis: {vehicle_detections_found} vehicles found, {len(vehicle_detections)} processed with {analysis_type}'
            self.debug_pub.publish(debug_msg)
        
        if immediate_danger:
            self.get_logger().warn(f'🚨 IMMEDIATE DANGER: TTC={ttc_info["min_ttc"]:.1f}s, Vehicle={ttc_info["threatening_vehicle"]}')
    
    def project_to_3d(self, bbox):
        """Project 2D bounding box center to 3D using depth and camera intrinsics"""
        try:
            # Get bounding box center
            center_x = int(bbox.center.position.x)
            center_y = int(bbox.center.position.y)
            
            # Debug: Log projection attempt
            debug_msg = String()
            debug_msg.data = f'🔍 3D Projection: center=({center_x},{center_y}), depth_shape={self.current_depth_image.shape if self.current_depth_image is not None else "None"}'
            self.debug_pub.publish(debug_msg)
            
            # Check if position is valid
            if (0 <= center_y < self.current_depth_image.shape[0] and 
                0 <= center_x < self.current_depth_image.shape[1]):
                
                # Get depth value
                depth_mm = self.current_depth_image[center_y, center_x]
                if depth_mm == 0:  # Invalid depth
                    debug_msg = String()
                    debug_msg.data = f'❌ Invalid depth: {depth_mm}mm at ({center_x},{center_y})'
                    self.debug_pub.publish(debug_msg)
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
            debug_msg = String()
            debug_msg.data = f'❌ 3D projection failed: {e}'
            self.debug_pub.publish(debug_msg)
        
        return None
    
    def analyze_2d_threat(self, bbox, class_id, confidence):
        """Analyze vehicle threat using 2D bounding box analysis when depth is unavailable"""
        
        # Get bounding box center and size
        center_x = bbox.center.position.x
        center_y = bbox.center.position.y
        bbox_width = bbox.size_x
        bbox_height = bbox.size_y
        
        # Image center (camera view)
        image_center_x = self.camera_intrinsics['cx']
        image_center_y = self.camera_intrinsics['cy']
        
        # Calculate distance from image center (horizontal threat)
        horizontal_distance = abs(center_x - image_center_x)
        vertical_distance = abs(center_y - image_center_y)
        
        # Normalize distances to 0-1 range
        max_horizontal = self.camera_intrinsics['cx']  # Half image width
        max_vertical = self.camera_intrinsics['cy']    # Half image height
        
        horizontal_threat = 1.0 - (horizontal_distance / max_horizontal)
        vertical_threat = 1.0 - (vertical_distance / max_vertical)
        
        # Higher threat if vehicle is in the center of the image (crossing path)
        # Lower threat if vehicle is at the edges (not in crossing path)
        crossing_path_threat = horizontal_threat * 0.7 + vertical_threat * 0.3
        
        # Size-based threat (larger bounding box = closer vehicle = higher threat)
        bbox_area = bbox_width * bbox_height
        max_area = 640 * 480  # Maximum possible bbox area
        size_threat = min(bbox_area / max_area * 2.0, 1.0)  # Scale up small areas
        
        # Vehicle type threat weights
        vehicle_weights = {
            1: 1.2,  # bus - higher threat
            2: 1.0,  # car - baseline
            5: 0.8,  # motorcycle - lower threat
            9: 1.5   # truck - highest threat
        }
        
        vehicle_weight = vehicle_weights.get(class_id, 1.0)
        
        # Combine threats
        total_threat = (
            crossing_path_threat * 0.6 +    # Position is most important
            size_threat * 0.3 +             # Size matters
            confidence * 0.1                # Confidence matters
        ) * vehicle_weight
        
        # Debug output
        debug_msg = String()
        debug_msg.data = f'🎯 2D Threat: {self.vehicle_classes[class_id]} at ({center_x:.0f},{center_y:.0f}), threat={total_threat:.3f}'
        self.debug_pub.publish(debug_msg)
        
        return np.clip(total_threat, 0.0, 1.0)
    
    def update_vehicle_tracking(self, vehicle_detections, current_time):
        """Update vehicle tracking with ego-motion compensation (supports 2D fallback)"""
        tracking_window = self.get_parameter('tracking_window').value
        
        for vehicle in vehicle_detections:
            if vehicle.get('position_3d') is not None:
                # 3D tracking (when depth is available)
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
            else:
                # 2D tracking (when depth is unavailable) - simplified
                vehicle_id = f"{vehicle['class_id']}_2d_{vehicle['bbox'].center.position.x:.0f}_{vehicle['bbox'].center.position.y:.0f}"
                
                # Add current detection to tracking history
                self.vehicle_tracks[vehicle_id].append({
                    'position_3d': None,  # No 3D data
                    'threat_level': vehicle.get('threat_level', 0.0),
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
        """Enhanced threat analysis using TTC and relative motion (supports 2D fallback)"""
        immediate_danger = False
        min_ttc = float('inf')
        max_velocity = 0.0
        threatening_vehicle = None
        
        ttc_threshold = self.get_parameter('ttc_safety_threshold').value
        
        # Debug: Log input
        debug_msg = String()
        debug_msg.data = f'🔍 analyze_enhanced_threats: {len(vehicle_detections)} vehicles to analyze'
        self.debug_pub.publish(debug_msg)
        
        for vehicle in vehicle_detections:
            # Handle both 3D and 2D data
            if vehicle.get('position_3d') is not None:
                # 3D analysis (when depth is available)
                vehicle_id = f"{vehicle['class_id']}_{vehicle['position_3d'].x:.1f}_{vehicle['position_3d'].y:.1f}"
                
                if vehicle_id in self.vehicle_tracks:
                    # Calculate relative velocity
                    relative_velocity = self.calculate_relative_velocity(self.vehicle_tracks[vehicle_id])
                    max_velocity = max(max_velocity, relative_velocity)
                    
                    # Calculate TTC for 3D vehicles
                    if relative_velocity > 0:
                        distance = math.sqrt(vehicle['position_3d'].x**2 + vehicle['position_3d'].y**2)
                        ttc = distance / relative_velocity
                        if ttc < min_ttc:
                            min_ttc = ttc
                            threatening_vehicle = vehicle['class_name']
                            
                            if ttc < ttc_threshold:
                                immediate_danger = True
            else:
                # 2D analysis (when depth is unavailable)
                threat_level = vehicle.get('threat_level', 0.0)
                
                # Debug: Log threat analysis
                debug_msg = String()
                debug_msg.data = f'🔍 2D threat check: {vehicle["class_name"]} threat={threat_level:.3f} > 0.3 = {threat_level > 0.3}'
                self.debug_pub.publish(debug_msg)
                
                # High threat level indicates immediate danger
                if threat_level > 0.3:  # Lower threshold for 2D analysis (consistent with fallback)
                    immediate_danger = True
                    min_ttc = 1.0  # Immediate threat
                    max_velocity = 20.0  # Assume typical vehicle speed
                    threatening_vehicle = vehicle['class_name']
                    
                    debug_msg = String()
                    debug_msg.data = f'🚨 2D IMMEDIATE DANGER: {vehicle["class_name"]} (threat: {threat_level:.3f})'
                    self.debug_pub.publish(debug_msg)
        
        # Debug: Log final result
        debug_msg = String()
        debug_msg.data = f'🔍 analyze_enhanced_threats RESULT: immediate_danger={immediate_danger}, ttc={min_ttc if min_ttc != float("inf") else 999.0}'
        self.debug_pub.publish(debug_msg)
        
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
