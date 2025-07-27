#!/usr/bin/env python3
"""
Enhanced Audio System - Handles Safety + VLM Explanations
Prioritizes safety alerts over VLM descriptions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import queue
import threading
import time
import subprocess

class EnhancedAudioSystem(Node):
    def __init__(self):
        super().__init__('enhanced_audio_system')
        
        # Dual audio queues
        self.safety_queue = queue.PriorityQueue()     # High priority safety
        self.vlm_queue = queue.Queue()                # Lower priority explanations
        
        self.is_speaking = False
        
        # Subscriptions
        self.safety_audio_sub = self.create_subscription(
            String, '/audio_guidance',
            self.safety_audio_callback, 10)
        
        self.vlm_explanation_sub = self.create_subscription(
            String, '/safety_reasoning_explanation',
            self.vlm_explanation_callback, 10)
        
        # Start audio processing
        self.audio_thread = threading.Thread(target=self.process_audio_queues, daemon=True)
        self.audio_thread.start()
        
        self.get_logger().info('🔊 Enhanced Audio System initialized')
    
    def safety_audio_callback(self, msg: String):
        """Handle structured safety audio (highest priority)"""
        message = msg.data
        priority = 1 if 'STOP' in message or 'DANGER' in message else 2
        
        self.safety_queue.put((priority, time.time(), message))
    
    def vlm_explanation_callback(self, msg: String):
        """Handle VLM explanations (lower priority)"""
        explanation = msg.data
        if explanation and len(explanation.strip()) > 0:
            self.vlm_queue.put(f"Reason: {explanation}")
    
    def process_audio_queues(self):
        """Process audio with priority: Safety first, then explanations"""
        while True:
            try:
                if not self.is_speaking:
                    # Priority 1: Safety messages
                    if not self.safety_queue.empty():
                        _, timestamp, message = self.safety_queue.get()
                        self.speak_message(message)
                    
                    # Priority 2: VLM explanations (when no safety alerts)
                    elif not self.vlm_queue.empty():
                        message = self.vlm_queue.get()
                        self.speak_message(message)
                
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f'Audio processing error: {e}')
                time.sleep(1.0)
    
    def speak_message(self, message):
        """Text-to-speech with appropriate timing"""
        try:
            self.is_speaking = True
            self.get_logger().info(f'🔊 Speaking: {message}')
            
            # Adjust speech rate based on message length
            speech_rate = 180 if len(message) < 50 else 160
            
            subprocess.run([
                'espeak', '-s', str(speech_rate), '-v', 'en', message
            ], capture_output=True, timeout=10)
            
        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')
        finally:
            self.is_speaking = False
            time.sleep(0.3)

def main():
    rclpy.init()
    audio_system = EnhancedAudioSystem()
    rclpy.spin(audio_system)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
