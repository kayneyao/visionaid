#!/usr/bin/env python3
"""
Vehicle Movement Analyzer - Priority 1 Safety Component
Enhanced with SORT tracking, Kalman filters, and HMM traffic light tracking
Updated for 11-class model: Only motorized vehicles are safety threats
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
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
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Import new tracking modules
from .sort_tracker import SORTTracker
from .hmm_traffic_light_tracker import HMMTrafficLightTracker

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
        self.declare_parameter('horizontal_threat_distance', 25.0)  # meters (relaxed)
        self.declare_parameter('motion_velocity_threshold', 0.8)    # m/s (relaxed)
        self.declare_parameter('ttc_safety_threshold', 8.0)        # seconds - time to collision (more conservative)
        self.declare_parameter('tracking_window', 2.0)             # seconds for velocity calculation
        
        # Vehicle-specific confidence thresholds (matching YOLOv8 camera node exactly)
        self.vehicle_confidence_thresholds = {
            1: 0.45,   # bus - more permissive
            2: 0.45,   # car - more permissive
            5: 0.55,   # motorcycle - more permissive
            9: 0.60    # truck - more permissive
        }
        
        # RealSense D435 camera intrinsics (from configuration)
        self.camera_intrinsics = {
            'fx': 617.0, 'fy': 617.0,
            'cx': 320.0, 'cy': 240.0,
            'width': 640, 'height': 480
        }
        self.have_camera_info = False
        self.color_resolution = (640, 480)
        self.depth_resolution = None
        
        # NEW: SORT tracker for robust multi-object tracking
        self.sort_tracker = SORTTracker(
            max_age=5,         # Fewer frames without update (faster drop)
            min_hits=2,        # Confirm tracks sooner
            iou_threshold=0.2  # Easier association
        )
        
        # NEW: HMM traffic light tracker for temporal stability
        self.traffic_light_tracker = HMMTrafficLightTracker(
            window_size=5,           # 5-frame sliding window
            confidence_threshold=0.6  # Minimum confidence for state changes
        )
        
        # State tracking
        self.current_depth_image = None
        self.vehicle_tracks = defaultdict(list)  # Legacy tracking (kept for compatibility)
        self.bridge = CvBridge()
        self.last_depth_time = 0.0
        
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
        
        # Use sensor-data QoS for depth (RealSense publishes best-effort)
        # Match RealSense publisher QoS: RELIABLE + TRANSIENT_LOCAL
        depth_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, depth_qos)
        
        # Subscribe to Color CameraInfo to use true intrinsics and resolution
        self.color_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.color_info_callback, 10)
        # Depth CameraInfo (aligned to color); try both possible namespaces
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            self.depth_info_callback, 10)
        self.depth_info_sub_alt = self.create_subscription(
            CameraInfo, '/camera/aligned_depth_to_color/camera_info',
            self.depth_info_callback, 10)
        
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
        
        # NEW: Enhanced tracking publishers
        self.tracking_stats_pub = self.create_publisher(
            String, '/vehicle_tracking_stats', 10)

        # Timing metrics publisher (JSON string)
        self.tracking_timing_pub = self.create_publisher(
            String, '/traffic_safety/tracking_timing', 10)
        
        self.traffic_light_state_pub = self.create_publisher(
            String, '/traffic_light_state', 10)
        
        self.traffic_light_confidence_pub = self.create_publisher(
            Float32, '/traffic_light_confidence', 10)
        
        # Debug publisher
        self.debug_pub = self.create_publisher(
            String, '/vehicle_analyzer_debug', 10)
        
        # Depth status publisher
        self.depth_status_pub = self.create_publisher(
            String, '/traffic_safety/depth_status', 10)
        
        self.get_logger().info('Enhanced Vehicle Movement Analyzer initialized (Priority 1)')
        self.get_logger().info('SORT tracking + Kalman filters + HMM traffic light tracking')
        self.get_logger().info('Robust multi-object tracking with data association')
    
    def depth_callback(self, msg):
        """Store depth image for 3D position calculation (robust to encoding)"""
        try:
            # Use passthrough to preserve original encoding
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Convert to uint16 millimeters if float
            if depth_img.dtype == np.float32:
                depth_img = np.nan_to_num(depth_img, nan=0.0, posinf=0.0, neginf=0.0)
                # Assume meters -> convert to mm
                depth_img = (depth_img * 1000.0).astype(np.uint16)
            elif depth_img.dtype != np.uint16:
                # Fallback: try explicit 16UC1 conversion
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            self.current_depth_image = depth_img
            self.depth_resolution = (depth_img.shape[1], depth_img.shape[0])
            import time as _t
            self.last_depth_time = _t.time()
            
            # Publish depth status
            depth_status_msg = String()
            depth_status_msg.data = f"DEPTH RECEIVED: {depth_img.shape[1]}x{depth_img.shape[0]} @ {depth_img.dtype}"
            self.depth_status_pub.publish(depth_status_msg)
            
            dbg = String(); dbg.data = "Depth frame received"; self.debug_pub.publish(dbg)
        except Exception as e:
            try:
                # Last resort: attempt 16UC1 directly
                self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                self.depth_resolution = (self.current_depth_image.shape[1], self.current_depth_image.shape[0])
                import time as _t
                self.last_depth_time = _t.time()
                
                # Publish depth status
                depth_status_msg = String()
                depth_status_msg.data = f"DEPTH RECEIVED: {self.current_depth_image.shape[1]}x{self.current_depth_image.shape[0]} @ {self.current_depth_image.dtype}"
                self.depth_status_pub.publish(depth_status_msg)
                
                dbg = String(); dbg.data = "Depth frame received (16UC1 fallback)"; self.debug_pub.publish(dbg)
            except Exception:
                self.current_depth_image = None
                
                # Publish depth failure status
                depth_status_msg = String()
                depth_status_msg.data = f"DEPTH FAILED: {e}"
                self.depth_status_pub.publish(depth_status_msg)
                
                self.get_logger().warn(f'Depth conversion failed: {e}')

    def color_info_callback(self, msg: CameraInfo):
        self.camera_intrinsics['fx'] = msg.k[0]
        self.camera_intrinsics['fy'] = msg.k[4]
        self.camera_intrinsics['cx'] = msg.k[2]
        self.camera_intrinsics['cy'] = msg.k[5]
        self.camera_intrinsics['width'] = msg.width
        self.camera_intrinsics['height'] = msg.height
        self.color_resolution = (msg.width, msg.height)
        self.have_camera_info = True

    def depth_info_callback(self, msg: CameraInfo):
        # Track depth resolution for scaling if not equal to color
        self.depth_resolution = (msg.width, msg.height)
    
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
            debug_msg.data = "No depth image available - using 2D fallback analysis"
            self.debug_pub.publish(debug_msg)
            use_2d_fallback = True
        else:
            # Check if depth image has valid data
            valid_depth_pixels = np.count_nonzero(self.current_depth_image)
            total_pixels = self.current_depth_image.shape[0] * self.current_depth_image.shape[1]
            valid_ratio = valid_depth_pixels / total_pixels
            
            if valid_ratio < 0.01:  # Less than 1% valid depth pixels (more permissive)
                debug_msg = String()
                debug_msg.data = f"Insufficient valid depth data ({valid_ratio:.1%}) - using 2D fallback analysis"
                self.debug_pub.publish(debug_msg)
                use_2d_fallback = True
        
        current_time = time.time()
        
        # Extract vehicle detections with 3D projection
        vehicle_detections = []
        total_detections = len(msg.detections)
        vehicle_detections_found = 0
        
        # Treat depth as available only if a recent frame arrived (<= 0.5s, relaxed)
        has_recent_depth = (self.current_depth_image is not None and (current_time - (self.last_depth_time or 0)) <= 0.5)
        if not has_recent_depth:
            use_2d_fallback = True
            
            # Publish no depth status
            depth_status_msg = String()
            depth_status_msg.data = "NO DEPTH: Using 2D fallback analysis"
            self.depth_status_pub.publish(depth_status_msg)
            
            debug_msg = String()
            debug_msg.data = "No recent depth frame - using 2D fallback analysis"
            self.debug_pub.publish(debug_msg)
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # Debug: Log vehicle detections only
            if class_id in self.vehicle_classes:
                debug_msg = String()
                debug_msg.data = f'Vehicle detected: {self.vehicle_classes[class_id]} (Class {class_id}), Confidence: {confidence:.3f}'
                self.debug_pub.publish(debug_msg)
                vehicle_detections_found += 1
            
            # Check if vehicle class with sufficient confidence (using vehicle-specific thresholds)
            threshold = self.vehicle_confidence_thresholds.get(class_id, 0.6)
            if (class_id in self.vehicle_classes and confidence >= threshold):
                debug_msg = String()
                debug_msg.data = f'Vehicle passed threshold: {self.vehicle_classes[class_id]} (conf: {confidence:.3f} >= {threshold:.3f})'
                self.debug_pub.publish(debug_msg)
                
                if use_2d_fallback:
                    # Use 2D analysis when depth is unavailable
                    threat_level = self.analyze_2d_threat(detection.bbox, class_id, confidence)
                    debug_msg = String()
                    debug_msg.data = f'2D Threat Analysis: {self.vehicle_classes[class_id]} threat={threat_level:.3f} (threshold: 0.3)'
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
                        debug_msg.data = f"3D projection failed for {self.vehicle_classes[class_id]} - falling back to 2D analysis"
                        self.debug_pub.publish(debug_msg)
                        
                        threat_level = self.analyze_2d_threat(detection.bbox, class_id, confidence)
                        if threat_level > 0.2:  # Only consider significant threats (relaxed)
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
        
        # NEW: Update SORT tracking for robust multi-object tracking (timed)
        _t0 = time.time()
        sort_tracks = self.update_sort_tracking(vehicle_detections)
        sort_update_ms = (time.time() - _t0) * 1000.0
        
        # Timing: vehicle association (Hungarian) is inside SORT; we approximate with sort_update_ms
        # NEW: Update HMM traffic light tracking for temporal stability
        all_detections = []
        for detection in msg.detections:
            if detection.results:
                class_id = int(detection.results[0].hypothesis.class_id)
                confidence = detection.results[0].hypothesis.score
                all_detections.append({
                    'class_id': class_id,
                    'confidence': confidence
                })
        
        traffic_light_state, traffic_light_confidence = self.update_traffic_light_tracking(all_detections)
        
        # NEW: Analyze threats using SORT tracker results (timed)
        _t1 = time.time()
        immediate_danger, min_ttc, threatening_vehicle = self.analyze_sort_threats(sort_tracks)
        sort_threat_ms = (time.time() - _t1) * 1000.0

        # Publish timing metrics JSON
        try:
            timing_json = {
                'sort_update_ms': float(sort_update_ms),
                'sort_threat_ms': float(sort_threat_ms)
            }
            timing_msg = String()
            timing_msg.data = json.dumps(timing_json)
            self.tracking_timing_pub.publish(timing_msg)
        except Exception:
            pass
        
        # Publish immediate danger status
        danger_msg = Bool()
        danger_msg.data = immediate_danger
        self.immediate_danger_pub.publish(danger_msg)
        
        # Publish TTC information
        ttc_msg = Float32()
        ttc_msg.data = min_ttc if min_ttc != float('inf') else 999.0
        self.ttc_pub.publish(ttc_msg)
        
        # Publish vehicle threat status
        threat_msg = String()
        if immediate_danger:
            threat_msg.data = f"IMMEDIATE DANGER: {threatening_vehicle} (TTC: {min_ttc:.2f}s)"
        else:
            threat_msg.data = f"Safe: Closest vehicle TTC: {min_ttc:.2f}s"
        self.vehicle_threat_pub.publish(threat_msg)
        
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
            self.get_logger().warn(f'PUBLISHING VEHICLE THREAT: immediate_danger=True at {current_time:.3f}')
        else:
            self.get_logger().info(f'PUBLISHING SAFE: immediate_danger=False at {current_time:.3f}')
        
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
            debug_msg.data = f'Vehicle Analysis: {vehicle_detections_found} vehicles found, {len(vehicle_detections)} processed with {analysis_type}'
            self.debug_pub.publish(debug_msg)
        
        if immediate_danger:
            self.get_logger().warn(f'IMMEDIATE DANGER: TTC={ttc_info["min_ttc"]:.1f}s, Vehicle={ttc_info["threatening_vehicle"]}')
    
    def project_to_3d(self, bbox):
        """Project 2D bounding box center to 3D using depth and camera intrinsics"""
        try:
            # Get bounding box center
            center_x = int(bbox.center.position.x)
            center_y = int(bbox.center.position.y)
            
            # Debug: Log projection attempt
            debug_msg = String()
            debug_msg.data = f'3D Projection: center=({center_x},{center_y}), depth_shape={self.current_depth_image.shape if self.current_depth_image is not None else "None"}'
            self.debug_pub.publish(debug_msg)
            
            # Check if position is valid
            if self.current_depth_image is None:
                return None

            # If depth and color resolutions differ, scale color pixel to depth pixel space
            dx, dy = center_x, center_y
            if self.depth_resolution is not None and self.color_resolution is not None:
                dw, dh = self.depth_resolution
                cw, ch = self.color_resolution
                if dw != cw or dh != ch:
                    scale_x = dw / max(cw, 1)
                    scale_y = dh / max(ch, 1)
                    dx = int(round(center_x * scale_x))
                    dy = int(round(center_y * scale_y))

            if (0 <= dy < self.current_depth_image.shape[0] and 
                0 <= dx < self.current_depth_image.shape[1]):
                
                # Robust depth: median of a growing window around the center to avoid zeros
                depth_mm = 0
                for radius in (1, 2, 3):  # up to 7x7 window
                    y0 = max(dy - radius, 0)
                    y1 = min(dy + radius + 1, self.current_depth_image.shape[0])
                    x0 = max(dx - radius, 0)
                    x1 = min(dx + radius + 1, self.current_depth_image.shape[1])
                    patch = self.current_depth_image[y0:y1, x0:x1]
                    valid = patch[patch > 0]
                    if valid.size > 0:
                        depth_mm = int(np.median(valid))
                        break
                if depth_mm == 0:  # Invalid depth
                    debug_msg = String()
                    debug_msg.data = f'Invalid depth: {depth_mm}mm at ({dx},{dy})'
                    self.debug_pub.publish(debug_msg)
                    return None
                
                depth_m = depth_mm / 1000.0  # Convert to meters
                
                # Project to 3D using camera intrinsics
                fx = self.camera_intrinsics['fx']
                fy = self.camera_intrinsics['fy']
                cx = self.camera_intrinsics['cx']
                cy = self.camera_intrinsics['cy']
                
                # 3D projection formula
                X = (center_x - cx) * depth_m / max(fx, 1e-6)
                Y = (center_y - cy) * depth_m / max(fy, 1e-6)
                Z = depth_m
                
                return Point(x=X, y=Y, z=Z)
            
        except Exception as e:
            debug_msg = String()
            debug_msg.data = f'3D projection failed: {e}'
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
        debug_msg.data = f'2D Threat: {self.vehicle_classes[class_id]} at ({center_x:.0f},{center_y:.0f}), threat={total_threat:.3f}'
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
        debug_msg.data = f'analyze_enhanced_threats: {len(vehicle_detections)} vehicles to analyze'
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
                debug_msg.data = f'2D threat check: {vehicle["class_name"]} threat={threat_level:.3f} > 0.3 = {threat_level > 0.3}'
                self.debug_pub.publish(debug_msg)
                
                # High threat level indicates immediate danger
                if threat_level > 0.3:  # Lower threshold for 2D analysis (consistent with fallback)
                    immediate_danger = True
                    min_ttc = 1.0  # Immediate threat
                    max_velocity = 20.0  # Assume typical vehicle speed
                    threatening_vehicle = vehicle['class_name']
                    
                    debug_msg = String()
                    debug_msg.data = f'2D IMMEDIATE DANGER: {vehicle["class_name"]} (threat: {threat_level:.3f})'
                    self.debug_pub.publish(debug_msg)
        
        # Debug: Log final result
        debug_msg = String()
        debug_msg.data = f'analyze_enhanced_threats RESULT: immediate_danger={immediate_danger}, ttc={min_ttc if min_ttc != float("inf") else 999.0}'
        self.debug_pub.publish(debug_msg)
        
        ttc_info = {
            'min_ttc': min_ttc if min_ttc != float('inf') else 999.0,
            'max_velocity': max_velocity,
            'threatening_vehicle': threatening_vehicle
        }
        
        return immediate_danger, ttc_info
    
    def update_sort_tracking(self, vehicle_detections):
        """Update SORT tracker with new vehicle detections"""
        # Convert detections to format expected by SORT tracker
        sort_detections = []
        
        for detection in vehicle_detections:
            sort_detection = {
                'class_id': detection['class_id'],
                'class_name': detection['class_name'],
                'confidence': detection['confidence'],
                'bbox': detection.get('bbox'),
                'position_3d': detection.get('position_3d')
            }
            sort_detections.append(sort_detection)
        
        # Update SORT tracker
        tracks = self.sort_tracker.update(sort_detections)
        
        # Publish tracking statistics
        stats_msg = String()
        stats_msg.data = f'SORT Tracking: {len(tracks)} active tracks, {len(vehicle_detections)} detections'
        self.tracking_stats_pub.publish(stats_msg)
        
        return tracks
    
    def update_traffic_light_tracking(self, all_detections):
        """Update HMM traffic light tracker"""
        # Extract all detections for traffic light analysis
        detection_results = []
        
        for detection in all_detections:
            detection_results.append({
                'class_id': detection['class_id'],
                'confidence': detection['confidence']
            })
        
        # Update HMM tracker
        current_state = self.traffic_light_tracker.update(detection_results)
        confidence = self.traffic_light_tracker.get_state_confidence()
        persistence = self.traffic_light_tracker.get_state_persistence()
        
        # Publish traffic light state and confidence
        state_msg = String()
        state_msg.data = current_state
        self.traffic_light_state_pub.publish(state_msg)
        
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.traffic_light_confidence_pub.publish(confidence_msg)
        
        # Debug logging
        debug_msg = String()
        debug_msg.data = f'Traffic Light: {current_state} (conf: {confidence:.3f}, persistence: {persistence})'
        self.debug_pub.publish(debug_msg)
        
        return current_state, confidence
    
    def calculate_enhanced_ttc(self, track):
        """Calculate enhanced TTC using Kalman filter state"""
        if not track.is_confirmed():
            return float('inf')
        
        # Get current position and velocity from Kalman filter
        position = track.get_position()
        velocity = track.get_velocity()
        
        # Calculate distance to camera (assuming camera at origin)
        distance = np.linalg.norm(position[:2])  # Only x, y components
        
        # Calculate velocity magnitude
        velocity_magnitude = np.linalg.norm(velocity[:2])  # Only x, y components
        
        if velocity_magnitude < 0.1:  # Very slow or stationary
            return float('inf')
        
        # Calculate TTC
        ttc = distance / velocity_magnitude
        
        # Apply ego-motion compensation if available
        if self.current_camera_pose and self.previous_camera_pose:
            # Calculate camera velocity
            dt = time.time() - self.camera_pose_timestamp if self.camera_pose_timestamp else 1/30.0
            cam_dx = self.current_camera_pose['position'].x - self.previous_camera_pose['position'].x
            cam_dy = self.current_camera_pose['position'].y - self.previous_camera_pose['position'].y
            cam_velocity = math.sqrt(cam_dx**2 + cam_dy**2) / dt
            
            # Adjust TTC for camera motion
            relative_velocity = max(0.1, velocity_magnitude - cam_velocity)
            ttc = distance / relative_velocity
        
        return ttc
    
    def analyze_sort_threats(self, tracks):
        """Analyze threats using SORT tracker results"""
        immediate_danger = False
        min_ttc = float('inf')
        threatening_vehicle = None
        
        ttc_threshold = self.get_parameter('ttc_safety_threshold').value
        
        for track in tracks:
            if track.is_confirmed():
                # Calculate enhanced TTC using Kalman filter state
                ttc = self.calculate_enhanced_ttc(track)
                
                if ttc < min_ttc:
                    min_ttc = ttc
                    threatening_vehicle = track.class_name
                    
                    if ttc < ttc_threshold:
                        immediate_danger = True
                        
                        # Debug logging
                        debug_msg = String()
                        debug_msg.data = f'SORT Threat: {track.class_name} TTC={ttc:.2f}s (ID: {track.track_id})'
                        self.debug_pub.publish(debug_msg)
        
        return immediate_danger, min_ttc, threatening_vehicle

def main():
    rclpy.init()
    node = VehicleMovementAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
