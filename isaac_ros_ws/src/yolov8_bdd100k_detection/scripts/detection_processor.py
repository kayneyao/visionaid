#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
from collections import defaultdict
import time

class DetectionProcessor(Node):
    def __init__(self):
        super().__init__('detection_processor')
        
        # Parameters
        self.declare_parameter('obstacle_timeout', 2.0)  # seconds
        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('max_distance', 10.0)  # meters
        self.declare_parameter('grid_resolution', 0.1)  # meters per cell
        self.declare_parameter('grid_size', 200)  # cells
        
        self.obstacle_timeout = self.get_parameter('obstacle_timeout').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.max_distance = self.get_parameter('max_distance').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_size = self.get_parameter('grid_size').value
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/camera/detections',
            self.detection_callback,
            10
        )
        
        self.obstacle_sub = self.create_subscription(
            PointStamped,
            '/obstacles/points',
            self.obstacle_callback,
            10
        )
        
        # Publishers
        self.filtered_detections_pub = self.create_publisher(
            Detection2DArray,
            '/detections/filtered',
            10
        )
        
        self.obstacle_poses_pub = self.create_publisher(
            PoseArray,
            '/obstacles/poses',
            10
        )
        
        self.obstacle_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/obstacles/grid',
            10
        )
        
        # Internal state
        self.obstacle_history = defaultdict(list)
        self.detection_stats = defaultdict(int)
        
        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_obstacles)
        
        self.get_logger().info('Detection Processor initialized')
    
    def detection_callback(self, msg):
        """Process incoming detections"""
        filtered_detections = Detection2DArray()
        filtered_detections.header = msg.header
        
        for detection in msg.detections:
            if detection.results:
                confidence = detection.results[0].hypothesis.score
                class_id = detection.results[0].hypothesis.class_id
                
                # Update statistics
                self.detection_stats[class_id] += 1
                
                # Filter by confidence
                if confidence >= self.min_confidence:
                    filtered_detections.detections.append(detection)
        
        # Publish filtered detections
        self.filtered_detections_pub.publish(filtered_detections)
    
    def obstacle_callback(self, msg):
        """Process obstacle points"""
        current_time = time.time()
        
        # Calculate distance
        distance = np.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2)
        
        # Filter by distance
        if distance <= self.max_distance:
            # Add to history with timestamp
            obstacle_data = {
                'point': msg.point,
                'timestamp': current_time,
                'distance': distance
            }
            
            # Use grid cell as key for clustering nearby obstacles
            grid_x = int(msg.point.x / self.grid_resolution)
            grid_y = int(msg.point.y / self.grid_resolution)
            grid_key = (grid_x, grid_y)
            
            self.obstacle_history[grid_key].append(obstacle_data)
    
    def process_obstacles(self):
        """Process and publish obstacle information"""
        current_time = time.time()
        
        # Clean up old obstacles
        for grid_key in list(self.obstacle_history.keys()):
            self.obstacle_history[grid_key] = [
                obs for obs in self.obstacle_history[grid_key]
                if current_time - obs['timestamp'] <= self.obstacle_timeout
            ]
            
            # Remove empty entries
            if not self.obstacle_history[grid_key]:
                del self.obstacle_history[grid_key]
        
        # Create pose array for current obstacles
        self.publish_obstacle_poses()
        
        # Create occupancy grid
        self.publish_obstacle_grid()
    
    def publish_obstacle_poses(self):
        """Publish obstacle poses for visualization"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera_color_optical_frame'
        
        for grid_key, obstacles in self.obstacle_history.items():
            if obstacles:
                # Average position of obstacles in this grid cell
                avg_x = np.mean([obs['point'].x for obs in obstacles])
                avg_y = np.mean([obs['point'].y for obs in obstacles])
                avg_z = np.mean([obs['point'].z for obs in obstacles])
                
                pose = Pose()
                pose.position.x = avg_x
                pose.position.y = avg_y
                pose.position.z = avg_z
                pose.orientation.w = 1.0
                
                pose_array.poses.append(pose)
        
        self.obstacle_poses_pub.publish(pose_array)
    
    def publish_obstacle_grid(self):
        """Publish occupancy grid for navigation"""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'camera_color_optical_frame'
        
        # Grid info
        grid.info.resolution = self.grid_resolution
        grid.info.width = self.grid_size
        grid.info.height = self.grid_size
        grid.info.origin.position.x = -self.grid_size * self.grid_resolution / 2
        grid.info.origin.position.y = -self.grid_size * self.grid_resolution / 2
        grid.info.origin.orientation.w = 1.0
        
        # Initialize grid data
        grid_data = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        
        # Mark obstacle cells
        for grid_key, obstacles in self.obstacle_history.items():
            if obstacles:
                grid_x, grid_y = grid_key
                
                # Convert to grid coordinates
                grid_x_idx = grid_x + self.grid_size // 2
                grid_y_idx = grid_y + self.grid_size // 2
                
                # Check bounds
                if 0 <= grid_x_idx < self.grid_size and 0 <= grid_y_idx < self.grid_size:
                    # Mark as occupied (100 = occupied, 0 = free, -1 = unknown)
                    grid_data[grid_y_idx, grid_x_idx] = 100
        
        # Flatten and publish
        grid.data = grid_data.flatten().tolist()
        self.obstacle_grid_pub.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DetectionProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
