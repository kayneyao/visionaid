#!/usr/bin/env python3
"""
Traffic Detection Processor - Updated for 11-class system
2-Priority Safety System: Vehicles > Traffic Lights
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np
from collections import defaultdict
import time

class DetectionProcessor(Node):
    def __init__(self):
        super().__init__('detection_processor')

        # Parameters
        self.declare_parameter('obstacle_timeout', 2.0)
        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('max_distance', 10.0)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid_size', 200)
        self.declare_parameter('filter_classes', [])

        self.obstacle_timeout = self.get_parameter('obstacle_timeout').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.max_distance = self.get_parameter('max_distance').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_size = self.get_parameter('grid_size').value
        self.filter_classes = self.get_parameter('filter_classes').value

        # NEW: 11-class mapping
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # NEW: 2-Priority Safety System
        self.safety_priorities = {
            # Priority 1: Vehicle Safety (ABSOLUTE OVERRIDE)
            'vehicles': [1, 2, 5, 9],           # bus, car, motorcycle, truck
            
            # Priority 2: Traffic Lights  
            'traffic_lights': [4, 7, 10],      # greenlight, redlight, yellowlight
            
            # Context classes (no safety override)
            'pedestrian_context': [0, 6],      # bicycle, pedestrian (cross together)
            'infrastructure': [3, 8]           # crosswalk, sidewalk
        }

        # State
        self.obstacle_history = defaultdict(list)
        self.detection_stats = defaultdict(int)

        # Subscriptions & Publications
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)

        self.obstacle_sub = self.create_subscription(
            PointStamped, '/obstacles/points',  
            self.obstacle_callback, 10)

        self.filtered_detections_pub = self.create_publisher(
            Detection2DArray, '/detections/filtered', 10)

        self.obstacle_poses_pub = self.create_publisher(
            PoseArray, '/obstacles/poses', 10)

        self.obstacle_grid_pub = self.create_publisher(
            OccupancyGrid, '/obstacles/grid', 10)

        # Timers
        self.create_timer(0.1, self.process_obstacles)
        self.create_timer(1.0, self.log_detection_stats)

        self.get_logger().info('Taiwan Detection Processor initialized')

    def detection_callback(self, msg: Detection2DArray):
        """Filter detections with Taiwan priorities"""
        filtered = Detection2DArray()
        filtered.header = msg.header

        for det in msg.detections:
            if not det.results:
                continue

            hyp = det.results[0].hypothesis
            # FIXED: Convert string class_id back to int
            try:
                cid = int(hyp.class_id)
            except ValueError:
                self.get_logger().warning(f'Invalid class_id: {hyp.class_id}')
                continue
                
            conf = float(hyp.score)

            # Update stats
            self.detection_stats[cid] += 1

            # Class filter
            if self.filter_classes and cid not in self.filter_classes:
                continue

            # Taiwan-specific confidence thresholds
            min_conf = self.get_taiwan_threshold(cid)
            if conf >= min_conf:
                filtered.detections.append(det)

        self.filtered_detections_pub.publish(filtered)

    def get_taiwan_threshold(self, class_id: int) -> float:
        """Get Taiwan-specific confidence thresholds"""
        if class_id in self.taiwan_priorities['critical']:
            return 0.5  # Lower threshold for critical Taiwan classes
        elif class_id in self.taiwan_priorities['safety']:
            return self.min_confidence  # Standard threshold for safety
        elif class_id in self.taiwan_priorities['fallback']:
            return 0.7  # Higher threshold for fallback classes
        else:
            return self.min_confidence  # Default threshold

    def obstacle_callback(self, msg: PointStamped):
        """Collect obstacle points, filter by distance, cluster by grid cell."""
        now = time.time()
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        dist = np.hypot(np.hypot(x, y), z)
        if dist > self.max_distance:
            return

        gx = int(x / self.grid_resolution)
        gy = int(y / self.grid_resolution)
        key = (gx, gy)

        self.obstacle_history[key].append({
            'point': msg.point,
            'ts': now
        })

    def process_obstacles(self):
        """Prune stale obstacles, then publish poses and occupancy grid."""
        now = time.time()

        # Remove old entries
        for key in list(self.obstacle_history):
            pts = [
                o for o in self.obstacle_history[key]
                if now - o['ts'] <= self.obstacle_timeout
            ]
            if pts:
                self.obstacle_history[key] = pts
            else:
                del self.obstacle_history[key]

        self.publish_obstacle_poses()
        self.publish_obstacle_grid()

    def publish_obstacle_poses(self):
        """Average each grid‐cell cluster into one Pose for visualization."""
        array = PoseArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = 'camera_color_optical_frame'

        for (gx, gy), pts in self.obstacle_history.items():
            xs = [p['point'].x for p in pts]
            ys = [p['point'].y for p in pts]
            zs = [p['point'].z for p in pts]

            pose = Pose()
            pose.position.x = float(np.mean(xs))
            pose.position.y = float(np.mean(ys))
            pose.position.z = float(np.mean(zs))
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            array.poses.append(pose)

        self.obstacle_poses_pub.publish(array)

    def publish_obstacle_grid(self):
        """Build and publish a binary occupancy grid around the camera."""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'camera_color_optical_frame'

        info = grid.info
        info.resolution = self.grid_resolution
        info.width = self.grid_size
        info.height = self.grid_size

        half = self.grid_size * self.grid_resolution / 2.0
        info.origin.position.x = -half
        info.origin.position.y = -half
        info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Initialize all cells as free (0)
        data = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        # Mark occupied cells
        for (gx, gy), pts in self.obstacle_history.items():
            ix = gx + self.grid_size // 2
            iy = gy + self.grid_size // 2
            if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
                data[iy, ix] = 100  # occupied

        grid.data = data.flatten().tolist()
        self.obstacle_grid_pub.publish(grid)

    def log_detection_stats(self):
        """Log Taiwan detection statistics"""
        if not self.detection_stats:
            return

        # Group by Taiwan priorities
        critical_stats = []
        safety_stats = []
        fallback_stats = []
        context_stats = []

        for cid, count in self.detection_stats.items():
            class_name = self.class_names.get(cid, f'class_{cid}')
            stat_str = f"{class_name}:{count}"
            
            if cid in self.taiwan_priorities['critical']:
                critical_stats.append(stat_str)
            elif cid in self.taiwan_priorities['safety']:
                safety_stats.append(stat_str)
            elif cid in self.taiwan_priorities['fallback']:
                fallback_stats.append(stat_str)
            else:
                context_stats.append(stat_str)

        # Log with Taiwan context
        if critical_stats:
            self.get_logger().info(f'🇹🇼 Taiwan Critical: {", ".join(critical_stats)}')
        if safety_stats:
            self.get_logger().info(f'🚨 Safety Classes: {", ".join(safety_stats)}')
        if fallback_stats:
            self.get_logger().info(f'🔄 Fallback Classes: {", ".join(fallback_stats)}')
        if context_stats:
            self.get_logger().info(f'📋 Context Classes: {", ".join(context_stats)}')

        self.detection_stats.clear()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
