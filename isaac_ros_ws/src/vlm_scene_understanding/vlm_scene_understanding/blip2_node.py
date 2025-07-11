# vlm_scene_understanding/vlm_scene_understanding/blip2_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
from transformers import Blip2Processor, Blip2ForConditionalGeneration
from PIL import Image as PILImage
import io

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
        
        # Initialize BLIP-2 model
        self.get_logger().info('Loading BLIP-2 model...')
        self.processor = Blip2Processor.from_pretrained("Salesforce/blip2-opt-2.7b")
        self.model = Blip2ForConditionalGeneration.from_pretrained("Salesforce/blip2-opt-2.7b")
        
        # Move to GPU if available
        if torch.cuda.is_available():
            self.model = self.model.to("cuda")
            self.get_logger().info('BLIP-2 model loaded on GPU')
        else:
            self.get_logger().info('BLIP-2 model loaded on CPU')
        
     
    
    def image_callback(self, msg):
        """Store latest camera image"""
        self.latest_image = msg
    
    def query_callback(self, msg):
        """Process user scene description request"""
        if self.latest_image is None:
            self.get_logger().warn('No camera image available')
            return
        
        user_query = msg.data
        description = self.generate_scene_description(user_query)
        
        # Publish description
        desc_msg = String()
        desc_msg.data = description
        self.description_pub.publish(desc_msg)
        
        self.get_logger().info(f'Scene description generated: {description[:100]}...')
    
    def generate_scene_description(self, query="Describe this scene"):
        """Generate scene description using BLIP-2"""
        try:
            # Convert ROS image to PIL
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "rgb8")
            pil_image = PILImage.fromarray(cv_image)
            
            # Process with BLIP-2
            inputs = self.processor(pil_image, query, return_tensors="pt")
            
            if torch.cuda.is_available():
                inputs = {k: v.to("cuda") for k, v in inputs.items()}
            
            # Generate description
            with torch.no_grad():
                generated_ids = self.model.generate(**inputs, max_length=100)
            
            description = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
            return description.strip()
            
        except Exception as e:
            self.get_logger().error(f'Error generating description: {str(e)}')
            return "Unable to generate scene description"

def main():
    rclpy.init()
    node = BLIP2SceneNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
