#!/usr/bin/env python3
"""
Enhanced Audio Feedback System - Priority-Based TTS
Provides immediate audio feedback with Taiwan-specific safety messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import queue
import time
import subprocess
import os

class EnhancedAudioFeedbackSystem(Node):
    def __init__(self):
        super().__init__('audio_feedback_system')
        
        # Audio configuration
        self.declare_parameter('speech_rate', 180)
        self.declare_parameter('volume', 0.9)
        self.declare_parameter('enable_audio', True)
        self.declare_parameter('language', 'en')
        
        # Audio message queue with priority
        self.audio_queue = queue.PriorityQueue()
        self.is_speaking = False
        self.last_message_time = {}
        
        # Taiwan-specific audio messages with enhanced detail
        self.taiwan_messages = {
            # Priority 1: Vehicle Danger (Immediate)
            'VEHICLE_HORIZONTAL': "STOP! Vehicle moving across your crossing path. Do not cross.",
            'MULTIPLE_VEHICLES': "STOP! Multiple vehicles detected in crossing area. Please wait.",
            'VEHICLE_APPROACHING': "Caution! Vehicle approaching your crossing area.",
            
            # Priority 2: Taiwan Crossing Path (Your 85.5% mAP50 Innovation)
            'TAIWAN_CROSSING_CONFIRMED': "Taiwan crossing path detected. Analyzing safety conditions.",
            'CROSSING_PATH_CLEAR': "Your crossing path is confirmed clear.",
            'SPATIAL_CLASSIFICATION_SUCCESS': "Taiwan crossing analysis complete. Path identified.",
            
            # Priority 3: Traffic Light Guidance
            'RED_LIGHT_STOP': "Red traffic light detected. Do not cross.",
            'GREEN_LIGHT_PROCEED': "Green traffic light confirmed. Proceed with caution.",
            'NO_SIGNAL_MANUAL': "No traffic signal detected. Please verify manually before crossing.",
            
            # Decision Results
            'SAFE_TO_CROSS': "All safety checks passed. Safe to proceed with caution.",
            'UNSAFE_CONDITIONS': "Unsafe crossing conditions detected. Please wait.",
            'MANUAL_VERIFICATION': "Uncertain conditions. Please verify manually before crossing.",
            
            # System Status
            'SYSTEM_READY': "Taiwan traffic safety system ready.",
            'MOTION_COMPENSATION_ACTIVE': "Head motion compensation active.",
            'DETECTION_QUALITY_LOW': "Detection quality reduced due to motion. Please move slowly."
        }
        
        # Message timing controls (prevent spam)
        self.message_cooldown = {
            1: 1.0,   # Priority 1: 1 second cooldown
            2: 2.0,   # Priority 2: 2 second cooldown  
            3: 3.0,   # Priority 3: 3 second cooldown
            4: 5.0    # Priority 4: 5 second cooldown
        }
        
        # Subscriptions
        self.decision_sub = self.create_subscription(
            String, '/crossing_decision',
            self.decision_callback, 10)
        
        self.reasoning_sub = self.create_subscription(
            String, '/decision_reasoning', 
            self.reasoning_callback, 10)
        
        self.vehicle_threat_sub = self.create_subscription(
            String, '/vehicle_threat_status',
            self.vehicle_threat_callback, 10)
        
        self.taiwan_status_sub = self.create_subscription(
            String, '/taiwan_crossing_status',
            self.taiwan_status_callback, 10)
        
        self.motion_alert_sub = self.create_subscription(
            Bool, '/excessive_motion_detected',
            self.motion_alert_callback, 10)
        
        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio_queue, daemon=True)
        self.audio_thread.start()
        
        # Check TTS availability
        self.check_tts_system()
        
        self.get_logger().info('🔊 Enhanced Audio Feedback System initialized')
        if self.get_parameter('enable_audio').value:
            self.speak_message_immediate("Taiwan traffic safety system ready.", 4)
    
    def check_tts_system(self):
        """Check and configure TTS system"""
        try:
            # Test espeak availability
            result = subprocess.run(['which', 'espeak'], capture_output=True, text=True)
            if result.returncode == 0:
                self.tts_engine = 'espeak'
                self.get_logger().info('Using espeak for text-to-speech')
            else:
                # Try festival as fallback
                result = subprocess.run(['which', 'festival'], capture_output=True, text=True)
                if result.returncode == 0:
                    self.tts_engine = 'festival'
                    self.get_logger().info('Using festival for text-to-speech')
                else:
                    self.tts_engine = None
                    self.get_logger().warn('No TTS engine found. Audio feedback disabled.')
        except Exception as e:
            self.tts_engine = None
            self.get_logger().error(f'TTS system check failed: {e}')
    
    def decision_callback(self, msg: String):
        """Handle crossing decision audio with priority"""
        decision_data = msg.data.split('|')
        decision = decision_data[0]
        confidence = float(decision_data[1]) if len(decision_data) > 1 else 0.0
        
        # Map decisions to audio messages
        message_map = {
            'DONT_CROSS': 'UNSAFE_CONDITIONS',
            'CROSS': 'SAFE_TO_CROSS', 
            'MANUAL': 'MANUAL_VERIFICATION'
        }
        
        message_key = message_map.get(decision, 'MANUAL_VERIFICATION')
        priority = 1 if decision == 'DONT_CROSS' else 3
        
        self.add_audio_message(message_key, priority)
    
    def reasoning_callback(self, msg: String):
        """Handle detailed reasoning for audio feedback"""
        reasoning = msg.data.lower()
        
        # Extract priority information from reasoning
        if 'immediate danger' in reasoning or 'vehicle' in reasoning:
            if 'horizontal' in reasoning:
                self.add_audio_message('VEHICLE_HORIZONTAL', 1)
            elif 'multiple' in reasoning:
                self.add_audio_message('MULTIPLE_VEHICLES', 1)
            else:
                self.add_audio_message('VEHICLE_APPROACHING', 1)
                
        elif 'red light' in reasoning:
            self.add_audio_message('RED_LIGHT_STOP', 2)
            
        elif 'green light' in reasoning:
            self.add_audio_message('GREEN_LIGHT_PROCEED', 3)
            
        elif 'taiwan crossing confirmed' in reasoning:
            self.add_
