#!/usr/bin/env python3
"""
Enhanced 3D Depth-Aware Detection System Test
Validates full integration of RealSense depth, ego-motion compensation, and TTC analysis
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import time
import threading
from collections import defaultdict

class Enhanced3DSystemTester(Node):
    def __init__(self):
        super().__init__('enhanced_3d_system_tester')
        
        # Test state tracking
        self.test_results = defaultdict(bool)
        self.test_data = defaultdict(dict)
        self.test_start_time = time.time()
        
        # Subscriptions to monitor system outputs
        self.immediate_danger_sub = self.create_subscription(
            Bool, '/traffic_safety/immediate_crossing_danger',
            self.immediate_danger_callback, 10)
        
        self.ttc_sub = self.create_subscription(
            Float32, '/traffic_safety/time_to_collision',
            self.ttc_callback, 10)
        
        self.velocity_sub = self.create_subscription(
            Float32, '/traffic_safety/vehicle_relative_velocity',
            self.velocity_callback, 10)
        
        self.motion_quality_sub = self.create_subscription(
            Float32, '/traffic_safety/motion_compensation_quality',
            self.motion_quality_callback, 10)
        
        self.crossing_decision_sub = self.create_subscription(
            String, '/traffic_safety/crossing_decision',
            self.crossing_decision_callback, 10)
        
        self.decision_reasoning_sub = self.create_subscription(
            String, '/traffic_safety/decision_reasoning',
            self.decision_reasoning_callback, 10)
        
        self.motion_stats_sub = self.create_subscription(
            String, '/traffic_safety/motion_compensation_stats',
            self.motion_stats_callback, 10)
        
        # Test timer
        self.create_timer(5.0, self.run_system_tests)
        
        self.get_logger().info('🧪 Enhanced 3D System Tester initialized')
        self.get_logger().info('Testing: 3D depth projection, ego-motion compensation, TTC analysis')
    
    def immediate_danger_callback(self, msg: Bool):
        """Monitor immediate danger detection"""
        self.test_data['immediate_danger'] = {
            'value': msg.data,
            'timestamp': time.time()
        }
        self.test_results['immediate_danger_received'] = True
    
    def ttc_callback(self, msg: Float32):
        """Monitor time-to-collision analysis"""
        self.test_data['ttc'] = {
            'value': msg.data,
            'timestamp': time.time()
        }
        self.test_results['ttc_received'] = True
        
        # Validate TTC values
        if 0 <= msg.data <= 999.0:  # Valid TTC range
            self.test_results['ttc_valid_range'] = True
    
    def velocity_callback(self, msg: Float32):
        """Monitor vehicle relative velocity"""
        self.test_data['velocity'] = {
            'value': msg.data,
            'timestamp': time.time()
        }
        self.test_results['velocity_received'] = True
        
        # Validate velocity values
        if msg.data >= 0.0:  # Non-negative velocity
            self.test_results['velocity_valid_range'] = True
    
    def motion_quality_callback(self, msg: Float32):
        """Monitor motion compensation quality"""
        self.test_data['motion_quality'] = {
            'value': msg.data,
            'timestamp': time.time()
        }
        self.test_results['motion_quality_received'] = True
        
        # Validate motion quality
        if 0.0 <= msg.data <= 1.0:  # Valid quality range
            self.test_results['motion_quality_valid_range'] = True
    
    def crossing_decision_callback(self, msg: String):
        """Monitor crossing decisions"""
        decision_data = msg.data.split('|')
        decision = decision_data[0]
        confidence = float(decision_data[1]) if len(decision_data) > 1 else 0.0
        
        self.test_data['crossing_decision'] = {
            'decision': decision,
            'confidence': confidence,
            'timestamp': time.time()
        }
        self.test_results['crossing_decision_received'] = True
        
        # Validate decision format
        if decision in ['DONT_CROSS', 'CROSS', 'MANUAL']:
            self.test_results['crossing_decision_valid'] = True
        if 0.0 <= confidence <= 1.0:
            self.test_results['crossing_confidence_valid'] = True
    
    def decision_reasoning_callback(self, msg: String):
        """Monitor decision reasoning"""
        self.test_data['decision_reasoning'] = {
            'reasoning': msg.data,
            'timestamp': time.time()
        }
        self.test_results['decision_reasoning_received'] = True
    
    def motion_stats_callback(self, msg: String):
        """Monitor motion compensation statistics"""
        self.test_data['motion_stats'] = {
            'stats': msg.data,
            'timestamp': time.time()
        }
        self.test_results['motion_stats_received'] = True
    
    def run_system_tests(self):
        """Run comprehensive system validation tests"""
        elapsed_time = time.time() - self.test_start_time
        
        self.get_logger().info('🧪 Running Enhanced 3D System Tests...')
        
        # Test 1: Basic Topic Reception
        self.test_topic_reception()
        
        # Test 2: Data Validation
        self.test_data_validation()
        
        # Test 3: System Integration
        self.test_system_integration()
        
        # Test 4: Performance Metrics
        self.test_performance_metrics()
        
        # Print comprehensive test report
        self.print_test_report()
        
        # Stop testing after 30 seconds
        if elapsed_time > 30.0:
            self.get_logger().info('✅ Enhanced 3D System Testing Complete')
            rclpy.shutdown()
    
    def test_topic_reception(self):
        """Test that all required topics are being published"""
        required_topics = [
            'immediate_danger_received',
            'ttc_received', 
            'velocity_received',
            'motion_quality_received',
            'crossing_decision_received',
            'decision_reasoning_received',
            'motion_stats_received'
        ]
        
        for topic in required_topics:
            if self.test_results[topic]:
                self.get_logger().info(f'✅ {topic}: Topic received')
            else:
                self.get_logger().warn(f'❌ {topic}: Topic not received')
    
    def test_data_validation(self):
        """Test data validity and ranges"""
        validation_tests = [
            ('ttc_valid_range', 'Time-to-Collision'),
            ('velocity_valid_range', 'Vehicle Velocity'),
            ('motion_quality_valid_range', 'Motion Quality'),
            ('crossing_decision_valid', 'Crossing Decision'),
            ('crossing_confidence_valid', 'Decision Confidence')
        ]
        
        for test_key, test_name in validation_tests:
            if self.test_results[test_key]:
                self.get_logger().info(f'✅ {test_name}: Valid data range')
            else:
                self.get_logger().warn(f'❌ {test_name}: Invalid data range')
    
    def test_system_integration(self):
        """Test system integration and data flow"""
        if (self.test_results['immediate_danger_received'] and 
            self.test_results['ttc_received'] and 
            self.test_results['crossing_decision_received']):
            self.get_logger().info('✅ System Integration: All core components communicating')
        else:
            self.get_logger().warn('❌ System Integration: Missing core component communication')
    
    def test_performance_metrics(self):
        """Test performance metrics and timing"""
        current_time = time.time()
        
        # Check data freshness (should be within last 2 seconds)
        for data_type, data in self.test_data.items():
            if 'timestamp' in data:
                age = current_time - data['timestamp']
                if age < 2.0:
                    self.get_logger().info(f'✅ {data_type}: Fresh data ({age:.1f}s old)')
                else:
                    self.get_logger().warn(f'❌ {data_type}: Stale data ({age:.1f}s old)')
    
    def print_test_report(self):
        """Print comprehensive test report"""
        self.get_logger().info('📊 Enhanced 3D System Test Report')
        self.get_logger().info('=' * 50)
        
        # Current system state
        if 'crossing_decision' in self.test_data:
            decision = self.test_data['crossing_decision']
            self.get_logger().info(f'Current Decision: {decision["decision"]} (conf: {decision["confidence"]:.2f})')
        
        if 'ttc' in self.test_data:
            ttc = self.test_data['ttc']
            self.get_logger().info(f'Time-to-Collision: {ttc["value"]:.1f}s')
        
        if 'velocity' in self.test_data:
            velocity = self.test_data['velocity']
            self.get_logger().info(f'Vehicle Velocity: {velocity["value"]:.1f}m/s')
        
        if 'motion_quality' in self.test_data:
            quality = self.test_data['motion_quality']
            self.get_logger().info(f'Motion Quality: {quality["value"]:.2f}')
        
        if 'immediate_danger' in self.test_data:
            danger = self.test_data['immediate_danger']
            self.get_logger().info(f'Immediate Danger: {danger["value"]}')
        
        # Test summary
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Test Summary: {passed_tests}/{total_tests} tests passed ({success_rate:.1f}%)')
        
        if success_rate >= 80:
            self.get_logger().info('🎉 Enhanced 3D System: PASSED')
        else:
            self.get_logger().warn('⚠️ Enhanced 3D System: NEEDS ATTENTION')

def main():
    rclpy.init()
    tester = Enhanced3DSystemTester()
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 