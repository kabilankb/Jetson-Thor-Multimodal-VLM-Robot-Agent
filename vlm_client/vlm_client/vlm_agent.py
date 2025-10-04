import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import requests
import json


class VLMClientNode(Node):
    def __init__(self):
        super().__init__('vlm_client_node')
        
        # Bridge for ROS <-> OpenCV
        self.bridge = CvBridge()
        
        # Store the latest prompt and image
        self.latest_prompt = "Describe the scene in this image."
        self.latest_image = None
        
        # Subscriber: camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscriber: user prompts
        self.prompt_subscription = self.create_subscription(
            String,
            '/vlm_prompt',
            self.prompt_callback,
            10
        )
        
        # Publisher: VLM response
        self.response_publisher = self.create_publisher(String, '/vlm_response', 10)
        
        # VLLM API endpoint
        self.api_url = "http://localhost:8000/v1/chat/completions"
        
        self.get_logger().info("✅ VLM Client Node started")
        self.get_logger().info("📷 Listening to /camera/image_raw")
        self.get_logger().info("💬 Listening to /vlm_prompt")
        self.get_logger().info("📤 Publishing to /vlm_response")

    def prompt_callback(self, msg):
        """Handle new user prompts"""
        self.latest_prompt = msg.data
        self.get_logger().info(f"📝 New prompt received: {self.latest_prompt}")
        
        # If we have both prompt and image, process immediately
        if self.latest_image is not None:
            self.process_vlm_request()

    def image_callback(self, msg):
        """Handle new camera images"""
        try:
            # Convert ROS2 Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            
            # Process VLM request if we have a prompt
            if self.latest_prompt:
                self.process_vlm_request()
                
        except Exception as e:
            self.get_logger().error(f"⚠️ Error in image_callback: {e}")

    def process_vlm_request(self):
        """Send current image + prompt to VLM API"""
        try:
            if self.latest_image is None:
                self.get_logger().warn("⚠️ No image available for VLM request")
                return
                
            # Encode OpenCV image -> JPEG -> base64
            _, buffer = cv2.imencode('.jpg', self.latest_image)
            img_base64 = base64.b64encode(buffer).decode("utf-8")
            
            # Prepare payload for vLLM
            payload = {
                "model": "Qwen/Qwen2.5-VL-32B-Instruct-AWQ",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": self.latest_prompt},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}}
                        ]
                    }
                ],
                "max_tokens": 200,
                "temperature": 0.7
            }
            
            self.get_logger().info(f"🚀 Sending request to vLLM API...")
            
            # Call vLLM API
            response = requests.post(self.api_url, json=payload, timeout=30)
            
            if response.status_code == 200:
                data = response.json()
                text = data['choices'][0]['message']['content']
                
                # Publish to ROS2
                ros_msg = String()
                ros_msg.data = text
                self.response_publisher.publish(ros_msg)
                
                self.get_logger().info(f"🤖 VLM Response: {text}")
            else:
                error_msg = f"API Error {response.status_code}: {response.text}"
                self.get_logger().error(f"❌ {error_msg}")
                
                # Publish error message
                ros_msg = String()
                ros_msg.data = f"Error: {error_msg}"
                self.response_publisher.publish(ros_msg)
                
        except requests.exceptions.Timeout:
            error_msg = "VLM API request timed out"
            self.get_logger().error(f"⏱️ {error_msg}")
            ros_msg = String()
            ros_msg.data = f"Error: {error_msg}"
            self.response_publisher.publish(ros_msg)
            
        except Exception as e:
            error_msg = f"Error in process_vlm_request: {e}"
            self.get_logger().error(f"⚠️ {error_msg}")
            ros_msg = String()
            ros_msg.data = f"Error: {error_msg}"
            self.response_publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLMClientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down VLM Client Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
