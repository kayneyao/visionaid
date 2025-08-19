#!/usr/bin/env python3
"""
Complete Traffic Crossing Assistant System Monitor
Monitors all components and shows decision reasoning

Enhanced:
- Subscribes to camera images and computes latencies
- Tracks YOLOv8 inference latency (image stamp -> detection arrival)
- Tracks detection->vehicle threat and detection->TTC latencies
- Tracks end-to-end decision latency (camera stamp -> decision time)
- Subscribes to SORT timing topic and aggregates update/threat times
- Maintains decision counts and prints concise summaries
- Periodically writes a JSON summary to monitor_logs/monitor_summary.json
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import defaultdict, deque
from datetime import datetime
import json
import os

class CompleteSystemMonitor(Node):
    def __init__(self):
        super().__init__('complete_system_monitor')
        
        self.bridge = CvBridge()
        
        # System status tracking
        self.system_status = {
            'yolov8_detection': False,
            'ego_motion_compensator': False,
            'vehicle_analyzer': False,
            'crosswalk_analyzer': False,
            'traffic_light_analyzer': False,
            'decision_engine': False,
            'audio_system': False,
            'multimodal_coordinator': False
        }
        
        # Decision tracking
        self.latest_decision = "No decision yet"
        self.latest_reasoning = "No reasoning available"
        self.decision_history = deque(maxlen=10)
        self.decision_counts = defaultdict(int)
        
        # Traffic light status
        self.traffic_light_state = "Unknown"
        self.traffic_light_confidence = 0.0
        
        # Vehicle threat status
        self.immediate_danger = False
        self.time_to_collision = 0.0
        self.vehicle_threat_status = "No threat"
        
        # Crosswalk status
        self.crosswalk_detected = False
        self.crosswalk_confidence = 0.0
        
        # Motion compensation status
        self.motion_compensation_quality = 0.0
        self.excessive_motion_detected = False
        
        # Statistics
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_stats = deque(maxlen=30)
        
        # Detection tracking
        self.detection_stats = defaultdict(int)  # Current frame detections
        self.latest_confidences = {}
        self.last_detection_time = 0
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Vehicle analyzer debug messages
        self.vehicle_debug_messages = []
        
        # Latency tracking
        self.image_stamp_window = deque(maxlen=5000)  # (stamp_sec, wall_time_sec)
        self.last_detection_wall_time = None
        self.detection_latency_ms = deque(maxlen=20000)
        self.end_to_end_decision_ms = deque(maxlen=20000)
        self.det_to_threat_ms = deque(maxlen=20000)
        self.det_to_ttc_ms = deque(maxlen=20000)
        self.sort_update_ms = deque(maxlen=20000)
        self.sort_threat_ms = deque(maxlen=20000)
        
        # Output directory for summaries
        self.monitor_logs_dir = os.path.join(os.getcwd(), 'monitor_logs')
        os.makedirs(self.monitor_logs_dir, exist_ok=True)
        self.summary_path = os.path.join(self.monitor_logs_dir, 'monitor_summary.json')
        
        # Subscribe to all system topics
        self.setup_subscriptions()
        
        # Display and summary timers
        self.create_timer(2.0, self.display_system_status)
        self.create_timer(10.0, self.write_summary_json)
        
        self.get_logger().info('Monitoring all system components and decision reasoning')
    
    def setup_subscriptions(self):
        """Setup subscriptions to all system topics"""
        
        # Camera images for timing reference
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # Detection system
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections', self.detection_callback, 10)
        
        # Decision system
        self.decision_sub = self.create_subscription(
            String, '/traffic_safety/crossing_decision', self.decision_callback, 10)
        
        self.reasoning_sub = self.create_subscription(
            String, '/traffic_safety/decision_reasoning', self.reasoning_callback, 10)
        
        # Traffic light analysis
        self.traffic_light_state_sub = self.create_subscription(
            String, '/traffic_safety/traffic_light_state', self.traffic_light_state_callback, 10)
        
        self.traffic_light_conf_sub = self.create_subscription(
            Float32, '/traffic_safety/traffic_light_confidence', self.traffic_light_conf_callback, 10)
        
        # Vehicle analysis
        self.immediate_danger_sub = self.create_subscription(
            Bool, '/traffic_safety/immediate_crossing_danger', self.immediate_danger_callback, 10)
        
        self.ttc_sub = self.create_subscription(
            Float32, '/traffic_safety/time_to_collision', self.ttc_callback, 10)
        
        self.vehicle_threat_sub = self.create_subscription(
            String, '/traffic_safety/vehicle_threat_status', self.vehicle_threat_callback, 10)
        
        # Crosswalk analysis
        self.crosswalk_detected_sub = self.create_subscription(
            Bool, '/traffic_safety/crosswalk_detected', self.crosswalk_detected_callback, 10)
        
        self.crosswalk_conf_sub = self.create_subscription(
            Float32, '/traffic_safety/crosswalk_confidence', self.crosswalk_conf_callback, 10)
        
        # Motion compensation
        self.motion_quality_sub = self.create_subscription(
            Float32, '/traffic_safety/motion_compensation_quality', self.motion_quality_callback, 10)
        
        self.excessive_motion_sub = self.create_subscription(
            Bool, '/traffic_safety/excessive_motion_detected', self.excessive_motion_callback, 10)
        
        # Audio system
        self.audio_message_sub = self.create_subscription(
            String, '/traffic_safety/audio_message', self.audio_message_callback, 10)
        
        # VLM system
        self.vlm_description_sub = self.create_subscription(
            String, '/traffic_safety/vlm_scene_description', self.vlm_description_callback, 10)
        
        self.safety_reasoning_sub = self.create_subscription(
            String, '/traffic_safety/safety_reasoning_explanation', self.safety_reasoning_callback, 10)
        
        # Vehicle analyzer debug messages
        self.vehicle_debug_sub = self.create_subscription(
            String, '/vehicle_analyzer_debug', self.vehicle_debug_callback, 10)
        
        # Tracking timing (JSON) from analyzer
        self.tracking_timing_sub = self.create_subscription(
            String, '/traffic_safety/tracking_timing', self.tracking_timing_callback, 10)
    
    def _to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9
    
    def image_callback(self, msg: Image):
        now = time.time()
        self.image_stamp_window.append((self._to_sec(msg.header.stamp), now))
    
    def detection_callback(self, msg):
        """Track detection system activity and latency"""
        self.system_status['yolov8_detection'] = True
        self.frame_count += 1
        
        # Calculate FPS
        current_time = time.time()
        if self.last_time > 0:
            fps = 1.0 / (current_time - self.last_time)
            self.fps_stats.append(fps)
        self.last_time = current_time
        
        # Detection latency (image stamp -> detection arrival)
        if msg.header.stamp:
            det_latency = max(0.0, (current_time - self._to_sec(msg.header.stamp)) * 1000.0)
            self.detection_latency_ms.append(det_latency)
        self.last_detection_wall_time = current_time
        
        # Track current frame detections by class
        detections_this_frame = 0
        
        # Clear stats only if we have new detections (to avoid stale data)
        if msg.detections:
            self.detection_stats.clear()
        
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                
                # Update current frame statistics
                self.detection_stats[class_id] += 1
                self.latest_confidences[class_id] = confidence
                detections_this_frame += 1
        
        # Update last detection time
        self.last_detection_time = current_time
    
    def decision_callback(self, msg):
        """Track crossing decisions and end-to-end latency"""
        self.system_status['decision_engine'] = True
        self.latest_decision = msg.data
        self.decision_counts[msg.data] += 1
        now = time.time()
        # Approximate end-to-end: now - last image stamp
        if self.image_stamp_window:
            last_stamp, _ = self.image_stamp_window[-1]
            e2e_ms = max(0.0, (now - last_stamp) * 1000.0)
            self.end_to_end_decision_ms.append(e2e_ms)
        self.decision_history.append({
            'decision': msg.data,
            'timestamp': now
        })
    
    def reasoning_callback(self, msg):
        """Track decision reasoning"""
        self.latest_reasoning = msg.data
    
    def traffic_light_state_callback(self, msg):
        """Track traffic light state"""
        self.system_status['traffic_light_analyzer'] = True
        self.traffic_light_state = msg.data
    
    def traffic_light_conf_callback(self, msg):
        """Track traffic light confidence"""
        self.traffic_light_confidence = msg.data
    
    def immediate_danger_callback(self, msg):
        """Track immediate danger status"""
        self.system_status['vehicle_analyzer'] = True
        self.immediate_danger = msg.data
    
    def ttc_callback(self, msg):
        """Track time to collision and detection->TTC latency"""
        self.system_status['vehicle_analyzer'] = True
        self.time_to_collision = msg.data
        if self.last_detection_wall_time is not None:
            self.det_to_ttc_ms.append(max(0.0, (time.time() - self.last_detection_wall_time) * 1000.0))
    
    def vehicle_threat_callback(self, msg):
        """Track vehicle threat status and detection->threat latency"""
        self.system_status['vehicle_analyzer'] = True
        self.vehicle_threat_status = msg.data
        if self.last_detection_wall_time is not None:
            self.det_to_threat_ms.append(max(0.0, (time.time() - self.last_detection_wall_time) * 1000.0))
    
    def crosswalk_detected_callback(self, msg):
        """Track crosswalk detection"""
        self.system_status['crosswalk_analyzer'] = True
        self.crosswalk_detected = msg.data
    
    def crosswalk_conf_callback(self, msg):
        """Track crosswalk confidence"""
        self.crosswalk_confidence = msg.data
    
    def motion_quality_callback(self, msg):
        """Track motion compensation quality"""
        self.system_status['ego_motion_compensator'] = True
        self.motion_compensation_quality = msg.data
    
    def excessive_motion_callback(self, msg):
        """Track excessive motion detection"""
        self.excessive_motion_detected = msg.data
    
    def audio_message_callback(self, msg):
        """Track audio system activity"""
        self.system_status['audio_system'] = True
    
    def vlm_description_callback(self, msg):
        """Track VLM scene description"""
        self.system_status['multimodal_coordinator'] = True
    
    def safety_reasoning_callback(self, msg):
        """Track safety reasoning"""
        pass  # Already handled by reasoning callback
    
    def vehicle_debug_callback(self, msg):
        """Track vehicle analyzer debug messages"""
        self.vehicle_debug_messages.append(msg.data)
        # Keep only last 5 messages to avoid spam
        if len(self.vehicle_debug_messages) > 5:
            self.vehicle_debug_messages.pop(0)
    
    def tracking_timing_callback(self, msg: String):
        """Receive SORT timing JSON from analyzer"""
        try:
            data = json.loads(msg.data)
            su = float(data.get('sort_update_ms', 0.0))
            st = float(data.get('sort_threat_ms', 0.0))
            if su > 0:
                self.sort_update_ms.append(su)
            if st > 0:
                self.sort_threat_ms.append(st)
        except Exception:
            pass
    
    def _summarize(self, samples: deque):
        if not samples:
            return {}
        arr = np.array(samples, dtype=np.float64)
        return {
            'count': int(arr.size),
            'mean_ms': float(arr.mean()),
            'min_ms': float(arr.min()),
            'max_ms': float(arr.max()),
            'std_ms': float(arr.std(ddof=1)) if arr.size > 1 else 0.0,
        }

    
    def display_system_status(self):
        """Display comprehensive system status"""
        avg_fps = np.mean(self.fps_stats) if self.fps_stats else 0
        
        print("\n" + "="*80)
        print("COMPLETE TRAFFIC CROSSING ASSISTANT SYSTEM STATUS")
        print("="*80)
        
        # System component status
        print("SYSTEM COMPONENTS:")
        for component, status in self.system_status.items():
            status_text = "OK" if status else "DOWN"
            print(f"   {status_text} {component.replace('_', ' ').title()}")
        
        # Performance
        print(f"\nPERFORMANCE: {avg_fps:.1f} FPS, {self.frame_count} frames processed")
        
        # Current frame detections
        current_time = time.time()
        time_since_last_detection = current_time - self.last_detection_time
        
        if self.detection_stats and time_since_last_detection < 2.0:
            print(f"\nCURRENT YOLOV8 DETECTIONS:")
            sorted_stats = sorted(self.detection_stats.items(), key=lambda x: x[1], reverse=True)
            for class_id, count in sorted_stats:
                class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                confidence = self.latest_confidences.get(class_id, 0.0)
                print(f"   {class_name}: {count} objects (conf: {confidence:.3f})")
            print(f"   (Last detection: {time_since_last_detection:.1f}s ago)")
        else:
            print(f"\nCURRENT YOLOV8 DETECTIONS: No objects detected in current frame")
            if time_since_last_detection < 10.0:
                print(f"   (Last detection: {time_since_last_detection:.1f}s ago)")
        
        # Latency summaries
        det = self._summarize(self.detection_latency_ms)
        e2e = self._summarize(self.end_to_end_decision_ms)
        d2t = self._summarize(self.det_to_threat_ms)
        d2c = self._summarize(self.det_to_ttc_ms)
        su = self._summarize(self.sort_update_ms)
        st = self._summarize(self.sort_threat_ms)
        
        print(f"\nLATENCIES:")
        print(f"   YOLOv8 inference: {det if det else 'n/a'}")
        print(f"   End-to-end decision: {e2e if e2e else 'n/a'}")
        print(f"   Detection->Vehicle Threat: {d2t if d2t else 'n/a'}")
        print(f"   Detection->TTC: {d2c if d2c else 'n/a'}")
        print(f"   SORT update: {su if su else 'n/a'}, SORT threat: {st if st else 'n/a'}")
        
        # Current decision and reasoning
        print(f"\nCURRENT DECISION: {self.latest_decision}")
        print(f"DECISION REASONING: {self.latest_reasoning}")
        
        # Decision counts
        if self.decision_counts:
            print(f"   Decision counts: {dict(self.decision_counts)}")
        
        # Traffic light analysis
        print(f"\nTRAFFIC LIGHT ANALYSIS:")
        print(f"   State: {self.traffic_light_state}")
        print(f"   Confidence: {self.traffic_light_confidence:.3f}")
        
        # Vehicle threat analysis
        print(f"\nVEHICLE THREAT ANALYSIS:")
        print(f"   Immediate Danger: {'YES' if self.immediate_danger else 'No'}")
        print(f"   Time to Collision: {self.time_to_collision:.2f}s")
        print(f"   Threat Status: {self.vehicle_threat_status}")
        
        # Vehicle analyzer debug messages
        if self.vehicle_debug_messages:
            print(f"\nVEHICLE ANALYZER DEBUG:")
            for msg in self.vehicle_debug_messages:
                print(f"   {msg}")
        
        # Crosswalk analysis
        print(f"\nCROSSWALK ANALYSIS:")
        print(f"   Detected: {'Yes' if self.crosswalk_detected else 'No'}")
        print(f"   Confidence: {self.crosswalk_confidence:.3f}")
        
        # Motion compensation
        print(f"\nMOTION COMPENSATION:")
        print(f"   Quality: {self.motion_compensation_quality:.3f}")
        print(f"   Excessive Motion: {'Yes' if self.excessive_motion_detected else 'No'}")
        
        # Recent decision history
        if self.decision_history:
            print(f"\nRECENT DECISIONS:")
            for i, decision_info in enumerate(list(self.decision_history)[-3:]):
                time_ago = time.time() - decision_info['timestamp']
                decision_str = decision_info['decision']
                print(f"   {i+1}. {decision_str} ({time_ago:.1f}s ago)")
        
        print("="*80)
        print("Press Ctrl+C to stop monitoring")
    
    def write_summary_json(self):
        summary = {
            'latency_ms': {
                'yolov8_inference': self._summarize(self.detection_latency_ms),
                'end_to_end_decision': self._summarize(self.end_to_end_decision_ms),
                'detection_to_vehicle_threat': self._summarize(self.det_to_threat_ms),
                'detection_to_ttc': self._summarize(self.det_to_ttc_ms),
                'sort_update': self._summarize(self.sort_update_ms),
                'sort_threat': self._summarize(self.sort_threat_ms),
            },
            'decision_counts': dict(self.decision_counts),
            'traffic_light': {
                'state': self.traffic_light_state,
                'confidence': float(self.traffic_light_confidence),
            },
            'vehicle': {
                'immediate_danger': bool(self.immediate_danger),
                'ttc_s': float(self.time_to_collision),
                'threat_status': self.vehicle_threat_status,
            },
            'crosswalk': {
                'detected': bool(self.crosswalk_detected),
                'confidence': float(self.crosswalk_confidence),
            },
            'motion_compensation': {
                'quality': float(self.motion_compensation_quality),
                'excessive_motion': bool(self.excessive_motion_detected)
            },
            'performance': {
                'avg_detection_fps': float(np.mean(self.fps_stats)) if self.fps_stats else 0.0,
                'frames_processed': int(self.frame_count),
            },
            'timestamp': datetime.now().isoformat(),
        }
        try:
            with open(self.summary_path, 'w') as f:
                json.dump(summary, f, indent=2)
        except Exception:
            pass


def main():
    rclpy.init()
    monitor = CompleteSystemMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nComplete system monitoring stopped by user")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 