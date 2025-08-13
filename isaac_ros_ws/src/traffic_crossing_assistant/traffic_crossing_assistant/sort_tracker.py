#!/usr/bin/env python3
"""
SORT (Simple Online and Realtime Tracking) Implementation
Enhanced with Kalman filters and data association for robust multi-object tracking
"""

import numpy as np
import cv2
from scipy.optimize import linear_sum_assignment
from collections import deque
import time

class KalmanFilter:
    """Kalman filter for 3D object tracking with constant velocity model"""
    
    def __init__(self, dt=1/30.0):
        # State: [x, y, z, vx, vy, vz]
        self.dt = dt
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix (we only observe position)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Process noise covariance
        sigma_p = 0.1  # position noise (m)
        sigma_v = 0.5  # velocity noise (m/s)
        self.Q = np.diag([sigma_p**2, sigma_p**2, sigma_p**2, 
                          sigma_v**2, sigma_v**2, sigma_v**2])
        
        # Measurement noise covariance
        sigma_m = 0.05  # measurement noise (m)
        self.R = np.diag([sigma_m**2, sigma_m**2, sigma_m**2])
        
        # Initial state uncertainty
        self.P = np.eye(6) * 1000
        
        # State vector
        self.x = np.zeros(6)
        
    def predict(self):
        """Predict next state"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x
    
    def update(self, measurement):
        """Update state with measurement"""
        # Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = measurement - self.H @ self.x  # innovation
        self.x = self.x + K @ y
        
        # Update uncertainty
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x
    
    def get_state(self):
        """Get current state"""
        return self.x.copy()
    
    def get_position(self):
        """Get current position"""
        return self.x[:3].copy()
    
    def get_velocity(self):
        """Get current velocity"""
        return self.x[3:].copy()

class Track:
    """Individual object track with Kalman filter"""
    
    def __init__(self, detection, class_id, track_id):
        self.track_id = track_id
        self.class_id = class_id
        self.class_name = detection.get('class_name', f'class_{class_id}')
        self.hits = 1
        self.age = 0
        self.time_since_update = 0
        self.state = 'tentative'  # tentative, confirmed, deleted
        
        # Initialize Kalman filter with first detection
        if detection.get('position_3d') is not None:
            pos = detection['position_3d']
            initial_state = np.array([pos.x, pos.y, pos.z, 0, 0, 0])
        else:
            # 2D fallback
            bbox = detection['bbox']
            initial_state = np.array([bbox.center.position.x, bbox.center.position.y, 0, 0, 0, 0])
        
        self.kalman = KalmanFilter()
        self.kalman.x = initial_state
        
        # Track history
        self.history = deque(maxlen=30)  # Keep last 30 detections
        self.history.append({
            'position': self.kalman.get_position(),
            'timestamp': time.time(),
            'confidence': detection.get('confidence', 0.0)
        })
        
        # Bounding box for 2D tracking
        self.bbox = detection.get('bbox')
        self.confidence = detection.get('confidence', 0.0)
    
    def predict(self):
        """Predict next state"""
        self.age += 1
        self.time_since_update += 1
        return self.kalman.predict()
    
    def update(self, detection):
        """Update track with new detection"""
        self.hits += 1
        self.time_since_update = 0
        
        # Update confidence
        self.confidence = detection.get('confidence', self.confidence)
        
        # Update bounding box
        if detection.get('bbox') is not None:
            self.bbox = detection['bbox']
        
        # Update Kalman filter
        if detection.get('position_3d') is not None:
            pos = detection['position_3d']
            measurement = np.array([pos.x, pos.y, pos.z])
        else:
            # 2D fallback
            bbox = detection['bbox']
            measurement = np.array([bbox.center.position.x, bbox.center.position.y, 0])
        
        self.kalman.update(measurement)
        
        # Add to history
        self.history.append({
            'position': self.kalman.get_position(),
            'timestamp': time.time(),
            'confidence': self.confidence
        })
        
        # Update state
        if self.hits >= 3:
            self.state = 'confirmed'
    
    def get_position(self):
        """Get current position"""
        return self.kalman.get_position()
    
    def get_velocity(self):
        """Get current velocity"""
        return self.kalman.get_velocity()
    
    def is_confirmed(self):
        """Check if track is confirmed"""
        return self.state == 'confirmed'
    
    def should_delete(self, max_age=10):
        """Check if track should be deleted"""
        return self.time_since_update > max_age

class SORTTracker:
    """SORT tracker with Kalman filters and data association"""
    
    def __init__(self, max_age=10, min_hits=3, iou_threshold=0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks = []
        self.next_id = 0
        self.frame_count = 0
        
    def update(self, detections):
        """Update tracks with new detections"""
        self.frame_count += 1
        
        # Predict new locations of existing tracks
        for track in self.tracks:
            track.predict()
        
        # Associate detections to existing tracks
        if len(self.tracks) > 0 and len(detections) > 0:
            # Calculate cost matrix
            cost_matrix = self._calculate_cost_matrix(detections)
            
            # Use Hungarian algorithm for optimal assignment
            if cost_matrix.size > 0:
                row_indices, col_indices = linear_sum_assignment(cost_matrix)
                
                # Update matched tracks
                for row, col in zip(row_indices, col_indices):
                    if cost_matrix[row, col] < self.iou_threshold:
                        self.tracks[row].update(detections[col])
                
                # Find unmatched detections and tracks
                unmatched_detections = [i for i in range(len(detections)) 
                                      if i not in col_indices]
                unmatched_tracks = [i for i in range(len(self.tracks)) 
                                  if i not in row_indices]
            else:
                unmatched_detections = list(range(len(detections)))
                unmatched_tracks = list(range(len(self.tracks)))
        else:
            unmatched_detections = list(range(len(detections)))
            unmatched_tracks = list(range(len(self.tracks)))
        
        # Create new tracks for unmatched detections
        for detection_idx in unmatched_detections:
            detection = detections[detection_idx]
            class_id = detection.get('class_id', 0)
            new_track = Track(detection, class_id, self.next_id)
            self.tracks.append(new_track)
            self.next_id += 1
        
        # Delete old tracks
        self.tracks = [track for track in self.tracks if not track.should_delete(self.max_age)]
        
        return self.tracks
    
    def _calculate_cost_matrix(self, detections):
        """Calculate cost matrix for data association"""
        if len(self.tracks) == 0 or len(detections) == 0:
            return np.array([])
        
        cost_matrix = np.zeros((len(self.tracks), len(detections)))
        
        for i, track in enumerate(self.tracks):
            for j, detection in enumerate(detections):
                # Calculate IoU for 2D bounding boxes
                if track.bbox is not None and detection.get('bbox') is not None:
                    iou = self._calculate_iou(track.bbox, detection['bbox'])
                    cost_matrix[i, j] = 1 - iou  # Convert IoU to cost (lower is better)
                else:
                    # Fallback to position-based cost
                    track_pos = track.get_position()
                    if detection.get('position_3d') is not None:
                        det_pos = detection['position_3d']
                        distance = np.linalg.norm(track_pos - np.array([det_pos.x, det_pos.y, det_pos.z]))
                        cost_matrix[i, j] = min(distance / 10.0, 1.0)  # Normalize distance
                    else:
                        cost_matrix[i, j] = 1.0  # Maximum cost for no match
        
        return cost_matrix
    
    def _calculate_iou(self, bbox1, bbox2):
        """Calculate Intersection over Union between two bounding boxes"""
        # Extract bounding box coordinates (BoundingBox2D has size_x and size_y)
        x1_1 = bbox1.center.position.x - bbox1.size_x / 2.0
        y1_1 = bbox1.center.position.y - bbox1.size_y / 2.0
        x2_1 = bbox1.center.position.x + bbox1.size_x / 2.0
        y2_1 = bbox1.center.position.y + bbox1.size_y / 2.0
        
        x1_2 = bbox2.center.position.x - bbox2.size_x / 2.0
        y1_2 = bbox2.center.position.y - bbox2.size_y / 2.0
        x2_2 = bbox2.center.position.x + bbox2.size_x / 2.0
        y2_2 = bbox2.center.position.y + bbox2.size_y / 2.0
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def get_tracks(self):
        """Get all active tracks"""
        return [track for track in self.tracks if track.is_confirmed()]
    
    def get_track_by_id(self, track_id):
        """Get track by ID"""
        for track in self.tracks:
            if track.track_id == track_id:
                return track
        return None 