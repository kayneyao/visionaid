#!/usr/bin/env python3
"""
Complete System Integration Test
Tests all components of the Taiwan traffic safety system
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String, Bool, Float32
import time
import threading

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # Test results tracking
        self.test_results = {
            'vehicle_analysis': False,
            'taiwan_crossing': False,
            'traffic_lights': False,
            'decision_engine': False,
            'audio_system': False,
            'motion_compensation': False
        }
        
        # Subscribers for monitoring
        self.setup_monitoring_subscriptions()
        
        # Publisher for test detections
        self.test_detection_pub = self.create_publisher(
            Detection2DArray, '/camera/detections', 10)
        
        self.get_logger().info('System Integration Tester initialized')
    
    def setup_monitoring_subscriptions(self):
        """Setup subscriptions to monitor all system components"""
        
        # Monitor vehicle analysis
        self.create_subscription(String, '/vehicle_threat_status', 
                               self.check_vehicle_analysis, 10)
        
        # Monitor Taiwan crossing analysis 
        self.create_subscription(Bool, '/crossing_path_confirmed',
                               self.check_taiwan_crossing, 10)
        
        # Monitor traffic light analysis
        self.create_subscription(String, '/traffic_light_state',
                               self.check_traffic_lights, 10)
        
        # Monitor decision engine
        self.create_subscription(String, '/crossing_decision',
                               self.check_decision_engine, 10)
        
        # Monitor motion compensation
        self.create_subscription(Float32, '/motion_compensation_quality',
                               self.check_motion_compensation, 10)
    
    def run_system_test(self):
        """Run comprehensive system test"""
        self.get_logger().info('Starting Taiwan traffic safety system test...')
        
        # Test 1: Vehicle detection simulation
        self.test_vehicle_detection()
        time.sleep(2)
        
        # Test 2: Taiwan crossing simulation
        self.test_taiwan_crossing_detection()
        time.sleep(2)
        
        # Test 3: Traffic light simulation
        self.test_traffic_light_detection()
        time.sleep(2)
        
        # Generate test report
        self.generate_test_report()
    
    def test_vehicle_detection(self):
        """Simulate vehicle detection"""
        detection_msg = Detection2DArray()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "camera_color_optical_frame"
        
        # Simulate car detection (Class 2)
        car_detection = Detection2D()
        car_detection.bbox.center.position.x = 320.0
        car_detection.bbox.center.position.y = 240.0
        car_detection.bbox.size_x = 100.0
        car_detection.bbox.size_y = 60.0
        
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "2"  # Car
        hypothesis.hypothesis.score = 0.85
        car_detection.results.append(hypothesis)
        
        detection_msg.detections.append(car_detection)
        self.test_detection_pub.publish(detection_msg)
        
        self.get_logger().info('📱 Published vehicle detection test')
    
    def test_taiwan_crossing_detection(self):
        """Simulate Taiwan crossing_crosswalk detection"""
        detection_msg = Detection2DArray()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "camera_color_optical_frame"
        
        # Simulate crossing_crosswalk detection (Class 3 - 85.5% mAP50 model)
        crossing_detection = Detection2D()
        crossing_detection.bbox.center.position.x = 320.0
        crossing_detection.bbox.center.position.y = 400.0
        crossing_detection.bbox.size_x = 200.0
        crossing_detection.bbox.size_y = 50.0
        
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "3"  # crossing_crosswalk
        hypothesis.hypothesis.score = 0.87    # Above the 85.5% threshold
        crossing_detection.results.append(hypothesis)
        
        detection_msg.detections.append(crossing_detection)
        self.test_detection_pub.publish(detection_msg)
        
        self.get_logger().info('🇹🇼 Published Taiwan crossing detection test')
    
    def test_traffic_light_detection(self):
        """Simulate traffic light detection"""
        detection_msg = Detection2DArray()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "camera_color_optical_frame"
        
        # Simulate red_light detection (Class 14)
        red_light_detection = Detection2D()
        red_light_detection.bbox.center.position.x = 320.0
        red_light_detection.bbox.center.position.y = 100.0
        red_light_detection.bbox.size_x = 30.0
        red_light_detection.bbox.size_y = 30.0
        
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "14"  # red_light
        hypothesis.hypothesis.score = 0.75
        red_light_detection.results.append(hypothesis)
        
        detection_msg.detections.append(red_light_detection)
        self.test_detection_pub.publish(detection_msg)
        
        self.get_logger().info('🚦 Published traffic light detection test')
    
    def check_vehicle_analysis(self, msg):
        self.test_results['vehicle_analysis'] = True
        self.get_logger().info(f'Vehicle analysis: {msg.data}')
    
    def check_taiwan_crossing(self, msg):
        self.test_results['taiwan_crossing'] = True
        self.get_logger().info(f'Taiwan crossing: {msg.data}')
    
    def check_traffic_lights(self, msg):
        self.test_results['traffic_lights'] = True
        self.get_logger().info(f'Traffic lights: {msg.data}')
    
    def check_decision_engine(self, msg):
        self.test_results['decision_engine'] = True
        self.get_logger().info(f'Decision engine: {msg.data}')
    
    def check_motion_compensation(self, msg):
        self.test_results['motion_compensation'] = True
        self.get_logger().info(f'Motion compensation: {msg.data:.2f}')
    
    def generate_test_report(self):
        """Generate comprehensive test report"""
        self.get_logger().info('TAIWAN TRAFFIC SAFETY SYSTEM TEST REPORT')
        self.get_logger().info('=' * 50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        for test_name, passed in self.test_results.items():
            status = 'PASS' if passed else 'FAIL'
            self.get_logger().info(f'{test_name.upper()}: {status}')
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'OVERALL: {passed_tests}/{total_tests} tests passed')
        
        if passed_tests == total_tests:
            self.get_logger().info('🎉 ALL SYSTEMS OPERATIONAL - Ready for deployment!')
        else:
            self.get_logger().warn('Some systems failed - Check component status')

def main():
    rclpy.init()
    tester = SystemTester()
    
    # Run test after brief startup delay
    threading.Timer(3.0, tester.run_system_test).start()
    
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
