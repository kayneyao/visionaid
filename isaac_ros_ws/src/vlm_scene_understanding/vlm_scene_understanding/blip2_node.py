import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
from transformers import Blip2Processor, Blip2ForConditionalGeneration
from PIL import Image as PILImage


class BLIP2SceneNode(Node):
    def __init__(self):
        super().__init__('blip2_scene_understanding')
        
        # Subscribe to camera and user queries
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.image_callback, 10)
        
        self.query_sub = self.create_subscription(
            String, '/scene_query', 
            self.query_callback, 10)
        
        # Publish scene descriptions
        self.description_pub = self.create_publisher(
            String, '/scene_description', 10)
        
        # Clear GPU cache before loading
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
        # Initialize BLIP-2 model with memory optimization
        self.get_logger().info('Loading BLIP-2 model with memory optimization...')
        
        # Load processor
        self.processor = Blip2Processor.from_pretrained("Salesforce/blip2-flan-t5-xl")
        
        # Initialize device tracking
        self.device = "cpu"  # Default fallback
        
        # Load model with memory optimization
        try:
            self.model = Blip2ForConditionalGeneration.from_pretrained(
                "Salesforce/blip2-flan-t5-xl",  # Smaller model for 8GB GPU
                torch_dtype=torch.float16,      # Half precision
                device_map="auto"               # Automatic GPU/CPU distribution
            )
            
            # Detect actual device placement
            if torch.cuda.is_available() and next(self.model.parameters()).is_cuda:
                self.device = "cuda"
                gpu_name = torch.cuda.get_device_name(0)
                gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
                self.get_logger().info(f'BLIP-2 model loaded on GPU: {gpu_name} ({gpu_memory:.1f} GB)')
            else:
                self.device = "cpu"
                self.get_logger().info('BLIP-2 model loaded on CPU')
                
        except torch.cuda.OutOfMemoryError as e:
            self.get_logger().error(f'GPU memory insufficient: {str(e)}')
            self.get_logger().warn('Loading BLIP-2 on CPU (slower performance)')
            
            # Fallback to CPU with regular precision
            self.model = Blip2ForConditionalGeneration.from_pretrained(
                "Salesforce/blip2-flan-t5-xl"
            )
            self.device = "cpu"
            
        except Exception as e:
            self.get_logger().error(f'Model loading failed: {str(e)}')
            self.get_logger().fatal('Cannot initialize BLIP-2 model')
            raise e
        
        # Initialize required components
        self.bridge = CvBridge()
        self.latest_image = None
        
        self.get_logger().info('BLIP-2 Scene Understanding Node Ready')
    
    def image_callback(self, msg):
        """Store latest camera image"""
        self.latest_image = msg
    
    def query_callback(self, msg):
        """Process user scene description request"""
        if self.latest_image is None:
            self.get_logger().warn('No camera image available')
            
            # Publish audio feedback for no camera
            desc_msg = String()
            desc_msg.data = "No camera image available for scene description."
            self.description_pub.publish(desc_msg)
            return
        
        user_query = msg.data
        description = self.generate_scene_description(user_query)
        
        # Publish description
        desc_msg = String()
        desc_msg.data = description
        self.description_pub.publish(desc_msg)
        
        self.get_logger().info(f'Scene description generated: {description[:100]}...')
    
    def generate_scene_description(self, query="Describe this scene for navigation"):
        """Generate scene description using BLIP-2 with memory management"""
        try:
            # Clear cache before inference
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
            # Convert ROS image to PIL
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "rgb8")
            pil_image = PILImage.fromarray(cv_image)
            
            # Resize image to reduce memory usage if needed
            if pil_image.size[0] > 1024 or pil_image.size[1] > 1024:
                pil_image = pil_image.resize((1024, 1024), PILImage.Resampling.LANCZOS)
            
            # Process with BLIP-2
            inputs = self.processor(pil_image, query, return_tensors="pt")
            
            # Move inputs to appropriate device
            if self.device == "cuda":
                inputs = {k: v.to("cuda") for k, v in inputs.items()}
            
            # Generate description with memory optimization
            with torch.no_grad():
                generated_ids = self.model.generate(
                    **inputs, 
                    max_length=100,
                    do_sample=False,      # Deterministic generation (less memory)
                    num_beams=1,         # Single beam (less memory)
                    temperature=0.7,     # Controlled randomness
                    pad_token_id=self.processor.tokenizer.eos_token_id
                )
            
            description = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
            
            # Clear cache after inference
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
            # Clean up description text
            description = description.strip()
            if description.startswith("Question:") or description.startswith("Answer:"):
                description = description.split("Answer:")[-1].strip()
            
            return description
            
        except torch.cuda.OutOfMemoryError as e:
            self.get_logger().error(f'GPU out of memory during inference: {str(e)}')
            return "Scene description temporarily unavailable due to memory constraints."
            
        except Exception as e:
            self.get_logger().error(f'Error generating description: {str(e)}')
            return "Unable to generate scene description."


def main():
    rclpy.init()
    try:
        node = BLIP2SceneNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        # Clean up GPU memory on shutdown
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
