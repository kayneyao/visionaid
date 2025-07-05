#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class DetectionProcessor(Node):
    def __init__(self):
        super().__init__('detection_processor')
        self.declare_parameter('confidence_threshold', 0.25)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        filtered = [d for d in msg.detections if d.results and d.results[0].hypothesis.score >= self.confidence_threshold]
        self.get_logger().info(f"Detections above threshold: {len(filtered)}")
        for det in filtered:
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            self.get_logger().info(f"Class: {class_id}, Score: {score:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
