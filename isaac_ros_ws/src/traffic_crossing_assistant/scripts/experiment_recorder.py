#!/usr/bin/env python3

"""
Experiment Recorder for Taiwan Traffic Safety System

- Subscribes to RealSense color images and writes rolling 10-second video clips
- Subscribes to detection, decision, and safety topics to log timestamps and compute latencies
- Optionally subscribes to detection visualization images
- Optionally launches tegrastats in a subprocess for power/thermal/resource logging

Output directory structure:
  <output_dir>/
    session_<YYYYmmdd_HHMMSS>/
      clips/
        clip_<idx>_<start_ts>.mp4
      images/  (optional snapshots)
      logs/
        messages.jsonl
        latency_summary.json
        tegrastats.log (optional)

Run:
  python3 experiment_recorder.py \
    --output /home/sophie/visionaid-1/experiment_data \
    --segment-seconds 10 \
    --max-segments 0 \
    --record-viz false \
    --enable-tegrastats true

This node assumes the full system is launched via
  ros2 launch traffic_crossing_assistant taiwan_complete_system.launch.py
which provides the topics remapped under /traffic_safety.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Deque, Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


@dataclass
class SegmentInfo:
    index: int
    start_wall_time_s: float
    writer: Optional[cv2.VideoWriter]
    frame_count: int


class ExperimentRecorder(Node):
    def __init__(self,
                 output_dir: str,
                 segment_seconds: int,
                 max_segments: int,
                 record_viz: bool,
                 enable_tegrastats: bool,
                 camera_topic: str,
                 detection_topic: str,
                 detection_viz_topic: str,
                 decision_topic: str,
                 reasoning_topic: str,
                 traffic_light_state_topic: str,
                 traffic_light_conf_topic: str,
                 ttc_topic: str,
                 vehicle_threat_topic: str,
                 immediate_danger_topic: str,
                 motion_quality_topic: str,
                 tracking_timing_topic: str):
        super().__init__('experiment_recorder')

        self.output_dir = os.path.abspath(output_dir)
        self.segment_seconds = max(1, int(segment_seconds))
        self.max_segments = int(max_segments)  # 0 = unlimited
        self.record_viz = bool(record_viz)
        self.enable_tegrastats = bool(enable_tegrastats)

        # Topics
        self.camera_topic = camera_topic
        self.detection_topic = detection_topic
        self.detection_viz_topic = detection_viz_topic
        self.decision_topic = decision_topic
        self.reasoning_topic = reasoning_topic
        self.traffic_light_state_topic = traffic_light_state_topic
        self.traffic_light_conf_topic = traffic_light_conf_topic
        self.ttc_topic = ttc_topic
        self.vehicle_threat_topic = vehicle_threat_topic
        self.immediate_danger_topic = immediate_danger_topic
        self.motion_quality_topic = motion_quality_topic
        self.tracking_timing_topic = tracking_timing_topic

        # Session setup
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.output_dir, f'session_{timestamp_str}')
        self.clips_dir = os.path.join(self.session_dir, 'clips')
        self.images_dir = os.path.join(self.session_dir, 'images')
        self.logs_dir = os.path.join(self.session_dir, 'logs')
        os.makedirs(self.clips_dir, exist_ok=True)
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        self.messages_log_path = os.path.join(self.logs_dir, 'messages.jsonl')
        self.latency_summary_path = os.path.join(self.logs_dir, 'latency_summary.json')

        self.bridge = CvBridge()

        # Video recording state
        self.current_segment: Optional[SegmentInfo] = None
        self.segment_idx = 0
        self.video_width = 640
        self.video_height = 480
        self.estimated_fps = 30.0
        self.last_frame_time = None
        self.video_codec = 'mp4v'  # Fallback to XVID if mp4v not available

        # Timers and windows for latency calculations
        self.image_stamp_window: Deque[Tuple[float, float]] = deque(maxlen=5000)  # (ros_stamp_sec, recv_wall_time_sec)
        self.detection_latency_samples_ms: Deque[float] = deque(maxlen=20000)
        self.end_to_end_decision_latency_ms: Deque[float] = deque(maxlen=20000)
        self.detection_to_vehicle_threat_latency_ms: Deque[float] = deque(maxlen=20000)
        self.detection_to_ttc_latency_ms: Deque[float] = deque(maxlen=20000)
        self.decision_values_count: Dict[str, int] = {}
        
        # Timing metrics (from analyzer JSON)
        self.sort_update_ms_samples: Deque[float] = deque(maxlen=20000)
        self.sort_threat_ms_samples: Deque[float] = deque(maxlen=20000)

        self._messages_log_fp = open(self.messages_log_path, 'a', buffering=1)
        self._tegrastats_proc: Optional[subprocess.Popen] = None

        # QoS
        qos = QoSProfile(depth=10)

        # Subscriptions
        self.create_subscription(Image, self.camera_topic, self.on_image, qos)
        self.create_subscription(Detection2DArray, self.detection_topic, self.on_detection, qos)
        if self.record_viz and self.detection_viz_topic:
            self.create_subscription(Image, self.detection_viz_topic, self.on_detection_viz, qos)
        self.create_subscription(String, self.decision_topic, self.on_decision, qos)
        if self.reasoning_topic:
            self.create_subscription(String, self.reasoning_topic, self.on_reasoning, qos)
        self.create_subscription(String, self.traffic_light_state_topic, self.on_traffic_light_state, qos)
        self.create_subscription(Float32, self.traffic_light_conf_topic, self.on_traffic_light_conf, qos)
        self.create_subscription(Float32, self.ttc_topic, self.on_ttc, qos)
        self.create_subscription(String, self.vehicle_threat_topic, self.on_vehicle_threat, qos)
        self.create_subscription(Bool, self.immediate_danger_topic, self.on_immediate_danger, qos)
        if self.motion_quality_topic:
            self.create_subscription(Float32, self.motion_quality_topic, self.on_motion_quality, qos)
        if self.tracking_timing_topic:
            self.create_subscription(String, self.tracking_timing_topic, self.on_tracking_timing, qos)

        # Periodic summary save
        self.create_timer(10.0, self._write_latency_summary)

        # Optional tegrastats
        if self.enable_tegrastats:
            self._start_tegrastats()

        self.get_logger().info(f'📁 Session directory: {self.session_dir}')
        self.get_logger().info(f'🎥 Writing {self.segment_seconds}s clips to {self.clips_dir}')

    # ---------- Utility ----------
    def _to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _open_new_segment(self) -> None:
        if self.max_segments and self.segment_idx >= self.max_segments:
            return
        segment_start = time.time()
        filename = f'clip_{self.segment_idx:04d}_{int(segment_start)}.mp4'
        clip_path = os.path.join(self.clips_dir, filename)
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
        writer = cv2.VideoWriter(clip_path, fourcc, self.estimated_fps,
                                 (self.video_width, self.video_height))
        self.current_segment = SegmentInfo(
            index=self.segment_idx,
            start_wall_time_s=segment_start,
            writer=writer,
            frame_count=0,
        )
        self.segment_idx += 1
        self.get_logger().info(f'🆕 New segment started: {clip_path}')

    def _close_segment(self) -> None:
        if self.current_segment and self.current_segment.writer:
            try:
                self.current_segment.writer.release()
            except Exception:
                pass
        self.current_segment = None

    def _log_message(self, record: Dict) -> None:
        try:
            self._messages_log_fp.write(json.dumps(record) + "\n")
        except Exception as e:
            self.get_logger().warn(f'Failed to write log record: {e}')

    def _write_latency_summary(self) -> None:
        def summarize(samples: Deque[float]) -> Dict[str, float]:
            if not samples:
                return {}
            arr = np.array(list(samples), dtype=np.float64)
            return {
                'count': int(arr.size),
                'mean_ms': float(arr.mean()),
                'min_ms': float(arr.min()),
                'max_ms': float(arr.max()),
                'std_ms': float(arr.std(ddof=1)) if arr.size > 1 else 0.0,
            }

        summary = {
            'yolov8_inference_latency_ms': summarize(self.detection_latency_samples_ms),
            'end_to_end_decision_latency_ms': summarize(self.end_to_end_decision_latency_ms),
            'detection_to_vehicle_threat_latency_ms': summarize(self.detection_to_vehicle_threat_latency_ms),
            'detection_to_ttc_latency_ms': summarize(self.detection_to_ttc_latency_ms),
            'sort_update_ms': summarize(self.sort_update_ms_samples),
            'sort_threat_ms': summarize(self.sort_threat_ms_samples),
            'decision_counts': dict(self.decision_values_count),
            'segment_seconds': self.segment_seconds,
            'estimated_fps': self.estimated_fps,
        }
        try:
            with open(self.latency_summary_path, 'w') as fp:
                json.dump(summary, fp, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Failed to write latency summary: {e}')

    def _start_tegrastats(self) -> None:
        tegra_log = os.path.join(self.logs_dir, 'tegrastats.log')
        tegra_cmd = ['tegrastats', '--interval', '1000']
        try:
            self._tegrastats_proc = subprocess.Popen(
                tegra_cmd, stdout=open(tegra_log, 'w'), stderr=subprocess.STDOUT
            )
            self.get_logger().info('🧪 tegrastats started (1s interval)')
        except FileNotFoundError:
            self.get_logger().warn('tegrastats not found; skipping power/thermal logging')
        except Exception as e:
            self.get_logger().warn(f'Failed to start tegrastats: {e}')

    def _stop_tegrastats(self) -> None:
        if self._tegrastats_proc is not None:
            try:
                self._tegrastats_proc.send_signal(signal.SIGINT)
                self._tegrastats_proc.terminate()
            except Exception:
                pass
            self._tegrastats_proc = None

    # ---------- Callbacks ----------
    def on_image(self, msg: Image) -> None:
        now = time.time()
        ros_stamp_sec = self._to_sec(msg.header.stamp)
        self.image_stamp_window.append((ros_stamp_sec, now))

        # Convert to BGR image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        # Initialize resolution and FPS on first frame
        h, w = frame.shape[:2]
        self.video_width, self.video_height = int(w), int(h)
        if self.last_frame_time is not None:
            delta = max(1e-6, now - self.last_frame_time)
            # Exponential moving average for FPS
            inst_fps = 1.0 / delta
            self.estimated_fps = 0.9 * self.estimated_fps + 0.1 * inst_fps
        self.last_frame_time = now

        # Open segment if needed
        if self.current_segment is None:
            self._open_new_segment()
        elif now - self.current_segment.start_wall_time_s >= self.segment_seconds:
            self._close_segment()
            if self.max_segments and self.segment_idx >= self.max_segments:
                # Stop recording after final segment
                self.get_logger().info('Reached max segments; shutting down.')
                self._write_latency_summary()
                self._shutdown()
                return
            self._open_new_segment()

        # Write frame
        if self.current_segment and self.current_segment.writer:
            self.current_segment.writer.write(frame)
            self.current_segment.frame_count += 1

        # Log minimal info for image frame
        self._log_message({
            'type': 'image',
            'topic': self.camera_topic,
            'header_stamp_sec': ros_stamp_sec,
            'recv_wall_time_sec': now,
            'width': self.video_width,
            'height': self.video_height,
            'segment_index': self.current_segment.index if self.current_segment else -1,
            'segment_start': self.current_segment.start_wall_time_s if self.current_segment else None,
        })

    def on_detection(self, msg: Detection2DArray) -> None:
        now = time.time()
        ros_stamp_sec = self._to_sec(msg.header.stamp)
        inference_latency_ms = max(0.0, (now - ros_stamp_sec) * 1000.0)
        self.detection_latency_samples_ms.append(inference_latency_ms)

        # Extract simple class list for quick per-frame context
        classes = []
        confidences = []
        for det in msg.detections:
            if det.results:
                try:
                    class_id = int(det.results[0].hypothesis.class_id)
                except Exception:
                    # class id might be stringified
                    class_id = int(str(det.results[0].hypothesis.class_id))
                classes.append(class_id)
                confidences.append(float(det.results[0].hypothesis.score))

        self._log_message({
            'type': 'detection',
            'topic': self.detection_topic,
            'header_stamp_sec': ros_stamp_sec,
            'recv_wall_time_sec': now,
            'inference_latency_ms': inference_latency_ms,
            'num_detections': len(msg.detections),
            'classes': classes,
            'confidences': confidences,
        })

        # For downstream timing correlation, store last detection arrival
        # (We infer detection->vehicle_threat and detection->ttc latencies using latest detection time)
        self._last_detection_arrival_wall_time = now
        self._last_detection_stamp_sec = ros_stamp_sec

    def on_detection_viz(self, msg: Image) -> None:
        # Save a snapshot image approximately once per second per segment
        now = time.time()
        ros_stamp_sec = self._to_sec(msg.header.stamp)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        if self.current_segment and self.current_segment.frame_count % int(max(1, self.estimated_fps)) == 0:
            fname = f'viz_{self.current_segment.index:04d}_{int(now)}.jpg'
            cv2.imwrite(os.path.join(self.images_dir, fname), frame)
        self._log_message({
            'type': 'viz_image',
            'topic': self.detection_viz_topic,
            'header_stamp_sec': ros_stamp_sec,
            'recv_wall_time_sec': now,
        })

    def on_decision(self, msg: String) -> None:
        now = time.time()
        # Match to the most recent image stamp at or before now
        img_stamp = self._nearest_image_stamp(before_wall_time=now)
        end_to_end_ms = (now - img_stamp) * 1000.0 if img_stamp is not None else None
        if end_to_end_ms is not None:
            self.end_to_end_decision_latency_ms.append(max(0.0, end_to_end_ms))
        value = msg.data
        self.decision_values_count[value] = self.decision_values_count.get(value, 0) + 1
        self._log_message({
            'type': 'decision',
            'topic': self.decision_topic,
            'recv_wall_time_sec': now,
            'value': value,
            'estimated_end_to_end_latency_ms': end_to_end_ms,
        })

    def on_reasoning(self, msg: String) -> None:
        now = time.time()
        self._log_message({
            'type': 'reasoning',
            'topic': self.reasoning_topic,
            'recv_wall_time_sec': now,
            'text': msg.data,
        })

    def on_traffic_light_state(self, msg: String) -> None:
        now = time.time()
        self._log_message({
            'type': 'traffic_light_state',
            'topic': self.traffic_light_state_topic,
            'recv_wall_time_sec': now,
            'state': msg.data,
        })

    def on_traffic_light_conf(self, msg: Float32) -> None:
        now = time.time()
        self._log_message({
            'type': 'traffic_light_confidence',
            'topic': self.traffic_light_conf_topic,
            'recv_wall_time_sec': now,
            'confidence': float(msg.data),
        })

    def on_ttc(self, msg: Float32) -> None:
        now = time.time()
        # Infer detection -> TTC latency if possible
        det_wall = getattr(self, '_last_detection_arrival_wall_time', None)
        if det_wall is not None:
            self.detection_to_ttc_latency_ms.append(max(0.0, (now - det_wall) * 1000.0))
        self._log_message({
            'type': 'time_to_collision',
            'topic': self.ttc_topic,
            'recv_wall_time_sec': now,
            'ttc_s': float(msg.data),
        })

    def on_vehicle_threat(self, msg: String) -> None:
        now = time.time()
        det_wall = getattr(self, '_last_detection_arrival_wall_time', None)
        if det_wall is not None:
            self.detection_to_vehicle_threat_latency_ms.append(max(0.0, (now - det_wall) * 1000.0))
        self._log_message({
            'type': 'vehicle_threat_status',
            'topic': self.vehicle_threat_topic,
            'recv_wall_time_sec': now,
            'status': msg.data,
        })

    def on_immediate_danger(self, msg: Bool) -> None:
        now = time.time()
        self._log_message({
            'type': 'immediate_crossing_danger',
            'topic': self.immediate_danger_topic,
            'recv_wall_time_sec': now,
            'danger': bool(msg.data),
        })

    def on_motion_quality(self, msg: Float32) -> None:
        now = time.time()
        self._log_message({
            'type': 'motion_compensation_quality',
            'topic': self.motion_quality_topic,
            'recv_wall_time_sec': now,
            'quality': float(msg.data),
        })

    def on_tracking_timing(self, msg: String) -> None:
        now = time.time()
        # message is JSON: {"sort_update_ms": x, "sort_threat_ms": y}
        try:
            data = json.loads(msg.data)
            su = float(data.get('sort_update_ms', 0.0))
            st = float(data.get('sort_threat_ms', 0.0))
            if su > 0:
                self.sort_update_ms_samples.append(su)
            if st > 0:
                self.sort_threat_ms_samples.append(st)
            self._log_message({
                'type': 'tracking_timing',
                'topic': self.tracking_timing_topic,
                'recv_wall_time_sec': now,
                'sort_update_ms': su,
                'sort_threat_ms': st,
            })
        except Exception:
            pass

    # ---------- Helpers ----------
    def _nearest_image_stamp(self, before_wall_time: float) -> Optional[float]:
        """Return the ROS image header stamp (sec) whose arrival time is closest to, but not after, before_wall_time."""
        if not self.image_stamp_window:
            return None
        # Since we don't store in sorted by wall time order necessarily, scan from right (most recent)
        for stamp_sec, wall_sec in reversed(self.image_stamp_window):
            if wall_sec <= before_wall_time:
                return stamp_sec
        return self.image_stamp_window[-1][0]

    def _shutdown(self) -> None:
        try:
            self._write_latency_summary()
        except Exception:
            pass
        try:
            self._close_segment()
        except Exception:
            pass
        try:
            self._stop_tegrastats()
        except Exception:
            pass
        # Close log file
        try:
            self._messages_log_fp.close()
        except Exception:
            pass
        # Initiate node shutdown
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Experiment Recorder for Taiwan Traffic Safety System')
    parser.add_argument('--output', type=str, default='/home/sophie/visionaid-1/experiment_data', help='Output directory for session data')
    parser.add_argument('--segment-seconds', type=int, default=10, help='Clip segment duration in seconds')
    parser.add_argument('--max-segments', type=int, default=0, help='Max number of segments (0 for unlimited)')
    # Accept string booleans to work with ROS launch substitutions
    parser.add_argument('--record-viz', type=str, default='false', help='true/false: Record detection visualization snapshots')
    parser.add_argument('--enable-tegrastats', type=str, default='false', help='true/false: Run tegrastats during recording (Jetson)')

    # Topics
    parser.add_argument('--camera-topic', type=str, default='/camera/camera/color/image_raw')
    parser.add_argument('--detection-topic', type=str, default='/camera/detections')
    parser.add_argument('--detection-viz-topic', type=str, default='/camera/detections/visualization')
    parser.add_argument('--decision-topic', type=str, default='/traffic_safety/crossing_decision')
    parser.add_argument('--reasoning-topic', type=str, default='/traffic_safety/decision_reasoning')
    parser.add_argument('--traffic-light-state-topic', type=str, default='/traffic_safety/traffic_light_state')
    parser.add_argument('--traffic-light-conf-topic', type=str, default='/traffic_safety/traffic_light_confidence')
    parser.add_argument('--ttc-topic', type=str, default='/traffic_safety/time_to_collision')
    parser.add_argument('--vehicle-threat-topic', type=str, default='/traffic_safety/vehicle_threat_status')
    parser.add_argument('--immediate-danger-topic', type=str, default='/traffic_safety/immediate_crossing_danger')
    parser.add_argument('--motion-quality-topic', type=str, default='/traffic_safety/motion_compensation_quality')
    parser.add_argument('--tracking-timing-topic', type=str, default='/traffic_safety/tracking_timing')

    args = parser.parse_args(argv)

    def str2bool(val: str) -> bool:
        return str(val).strip().lower() in ('1', 'true', 'yes', 'y', 'on')

    rclpy.init()
    node = ExperimentRecorder(
        output_dir=args.output,
        segment_seconds=args.segment_seconds,
        max_segments=args.max_segments,
        record_viz=str2bool(args.record_viz),
        enable_tegrastats=str2bool(args.enable_tegrastats),
        camera_topic=args.camera_topic,
        detection_topic=args.detection_topic,
        detection_viz_topic=args.detection_viz_topic,
        decision_topic=args.decision_topic,
        reasoning_topic=args.reasoning_topic,
        traffic_light_state_topic=args.traffic_light_state_topic,
        traffic_light_conf_topic=args.traffic_light_conf_topic,
        ttc_topic=args.ttc_topic,
        vehicle_threat_topic=args.vehicle_threat_topic,
        immediate_danger_topic=args.immediate_danger_topic,
        motion_quality_topic=args.motion_quality_topic,
        tracking_timing_topic=args.tracking_timing_topic,
    )

    # Handle SIGINT/SIGTERM gracefully
    def handle_signal(signum, frame):
        try:
            node.get_logger().info('Received termination signal, shutting down...')
        except Exception:
            pass
        node._shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._shutdown()


if __name__ == '__main__':
    main() 