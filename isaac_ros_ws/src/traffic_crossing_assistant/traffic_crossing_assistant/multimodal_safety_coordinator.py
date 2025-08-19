#!/usr/bin/env python3
"""
Multimodal Safety Coordinator - Direct VLM Integration
Takes decision_engine results and explains them using VLM
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
import torch
from transformers import LlavaNextProcessor, LlavaNextForConditionalGeneration
from PIL import Image as PILImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import queue
import time

class MultimodalSafetyCoordinator(Node):
    def __init__(self):
        super().__init__('multimodal_safety_coordinator')
        
        # Initialize VLM model (LLaVA-1.6-7B)
        self.setup_vlm_model()
        
        # Taiwan 17-class mapping for context
        self.taiwan_classes = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crossing_crosswalk',
            9: 'motorcycle', 10: 'pedestrian', 12: 'truck',
            14: 'red_light', 15: 'green_light'
        }
        
        # Current system state
        self.current_image = None
        self.structured_decision = None
        self.taiwan_detections = {}
        
        # VLM processing queue
        self.vlm_queue = queue.Queue(maxsize=3)
        self.vlm_processing = False
        
        # Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscriptions - Ingest results from decision engine
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10)
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        # KEY: Subscribe to decision engine results
        self.decision_sub = self.create_subscription(
            String, '/crossing_decision',
            self.decision_callback, 10)
        
        # Subscribe to user voice queries
        self.voice_query_sub = self.create_subscription(
            String, '/user_voice_query',
            self.voice_query_callback, 10)
        
        # Publishers for VLM explanations
        self.enhanced_guidance_pub = self.create_publisher(
            String, '/multimodal_guidance', 10)
        
        self.scene_description_pub = self.create_publisher(
            String, '/vlm_scene_description', 10)
        
        self.safety_explanation_pub = self.create_publisher(
            String, '/safety_reasoning_explanation', 10)
        
        # Start VLM processing thread
        self.vlm_thread = threading.Thread(target=self.process_vlm_queue, daemon=True)
        self.vlm_thread.start()
        
        self.get_logger().info('🧠 Multimodal Safety Coordinator initialized')
    
    def setup_vlm_model(self):
        """Initialize VLM model using BLIP-2-style loading flow"""
        try:
            model_name = "llava-hf/llava-v1.6-mistral-7b-hf"
            
            # Apply memory management actions
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
            self.vlm_processor = LlavaNextProcessor.from_pretrained(model_name)
            self.vlm_model = LlavaNextForConditionalGeneration.from_pretrained(
                model_name,
                torch_dtype=torch.float16,
                device_map="auto",
                load_in_4bit=True  # Optimization approach
            )
            
            self.get_logger().info('✅ LLaVA model loaded for traffic safety')
            
        except Exception as e:
            self.get_logger().error(f'LLaVA loading failed: {e}')
            self.vlm_model = None

    
    def decision_callback(self, msg: String):
        """KEY FUNCTION: Receive decision_engine results and explain them"""
        decision_data = msg.data.split('|')
        decision = decision_data[0]  # "DONT_CROSS", "CROSS", "MANUAL"
        confidence = float(decision_data[1]) if len(decision_data) > 1 else 0.0
        
        # Store the structured decision
        self.structured_decision = {
            'action': decision,
            'confidence': confidence,
            'timestamp': time.time()
        }
        
        # Generate VLM explanation based on decision
        if decision == 'DONT_CROSS':
            # Urgent: Can't cross - explain why immediately
            self.generate_safety_explanation(urgent=True)
        elif decision == 'CROSS':
            # Safe to cross - brief confirmation
            self.generate_brief_confirmation()
        else:  # MANUAL
            # Unclear situation - explain what user should check
            self.generate_manual_guidance()
    
    def generate_safety_explanation(self, urgent=False, user_query=None):
        """Generate VLM explanation for why can't cross"""
        if self.current_image is None or self.vlm_model is None:
            return
        
        # Create concise prompt for safety explanation
        prompt = self.create_safety_prompt()
        
        vlm_request = {
            'type': 'safety_explanation',
            'image': self.current_image.copy(),
            'prompt': prompt,
            'priority': 1 if urgent else 2,
            'callback': self.publish_safety_explanation
        }
        
        try:
            if urgent:
                # Clear queue for urgent explanations
                while not self.vlm_queue.empty():
                    try:
                        self.vlm_queue.get_nowait()
                    except queue.Empty:
                        break
            
            self.vlm_queue.put_nowait(vlm_request)
        except queue.Full:
            self.get_logger().warn('VLM queue full')
    
    def create_safety_prompt(self):
        """Create concise prompt for safety explanation"""
        vehicle_count = len(self.taiwan_detections.get('vehicles', []))
        crossing_detected = len(self.taiwan_detections.get('crossing_crosswalk', [])) > 0
        
        prompt = f"""Explain in 10 words or less why crossing is unsafe.

Current situation:
- Decision: {self.structured_decision['action'] if self.structured_decision else 'unknown'}
- Vehicles detected: {vehicle_count}
- Crossing path: {crossing_detected}

Examples of good responses:
- "Red light detected"
- "Car blocking path"
- "Multiple vehicles approaching"
- "Motorcycle in crossing area"

Keep response under 10 words."""
        
        return prompt
    
    def process_vlm_queue(self):
        """Process VLM explanation requests"""
        while True:
            try:
                if not self.vlm_queue.empty() and not self.vlm_processing:
                    vlm_request = self.vlm_queue.get()
                    
                    self.vlm_processing = True
                    response = self.process_vlm_request(vlm_request)
                    
                    if response and vlm_request['callback']:
                        vlm_request['callback'](response, vlm_request['type'])
                    
                    self.vlm_processing = False
                
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f'VLM processing error: {e}')
                self.vlm_processing = False
                time.sleep(1.0)
    
    def process_vlm_request(self, request):
        """Process individual VLM request"""
        try:
            # Convert image to PIL
            image_rgb = cv2.cvtColor(request['image'], cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(image_rgb)
            
            # Prepare inputs
            inputs = self.vlm_processor(
                text=request['prompt'],
                images=pil_image,
                return_tensors="pt"
            ).to(self.vlm_model.device, torch.float16)
            
            # Generate response with length limits
            with torch.no_grad():
                generate_ids = self.vlm_model.generate(
                    **inputs,
                    max_new_tokens=15,  # Very short responses
                    do_sample=True,
                    temperature=0.3,
                    pad_token_id=self.vlm_processor.tokenizer.eos_token_id
                )
            
            # Decode response
            response = self.vlm_processor.batch_decode(
                generate_ids[:, inputs['input_ids'].shape[1]:],
                skip_special_tokens=True,
                clean_up_tokenization_spaces=False
            )[0]
            
            # Clean up response
            cleaned = response.strip().replace("I can see", "").replace("There is", "")
            words = cleaned.split()
            if len(words) > 10:
                cleaned = ' '.join(words[:10])
            
            return cleaned
            
        except Exception as e:
            self.get_logger().error(f'VLM inference error: {e}')
            return None
    
    def publish_safety_explanation(self, explanation, request_type):
        """Publish VLM safety explanation"""
        msg = String()
        msg.data = explanation
        self.safety_explanation_pub.publish(msg)
        
        self.get_logger().info(f'🔍 VLM Explanation: {explanation}')
    
    # Additional callback methods...
    def image_callback(self, msg: Image):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
    
    def detection_callback(self, msg: Detection2DArray):
        self.taiwan_detections = self.extract_taiwan_context(msg.detections)
    
    def voice_query_callback(self, msg: String):
        user_query = msg.data.lower()
        if 'why' in user_query or 'explain' in user_query:
            self.generate_safety_explanation(user_query=user_query)
    
    def extract_taiwan_context(self, detections):
        """Extract context for VLM"""
        context = {'vehicles': [], 'crossing_crosswalk': []}
        
        for detection in detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            if class_id in [0, 1, 2, 9, 12]:  # Vehicles
                context['vehicles'].append({'class_id': class_id, 'confidence': confidence})
            elif class_id == 3:  # crossing_crosswalk
                context['crossing_crosswalk'].append({'confidence': confidence})
        
        return context

def main():
    rclpy.init()
    coordinator = MultimodalSafetyCoordinator()
    rclpy.spin(coordinator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
