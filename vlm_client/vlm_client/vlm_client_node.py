#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import requests


class VLMClientNode(Node):
    def __init__(self):
        super().__init__('vlm_client_node')

        # Bridge for ROS <-> OpenCV
        self.bridge = CvBridge()

        # Subscriber: camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',   # change if your topic is different
            self.image_callback,
            10
        )

        # Publisher: VLM response
        self.publisher = self.create_publisher(String, '/vlm_response', 10)

        # VLLM API endpoint
        self.api_url = "http://localhost:8000/v1/chat/completions"

        self.get_logger().info("✅ VLM Client Node started, listening to /camera/image_raw")

    def image_callback(self, msg):
        try:
            # Convert ROS2 Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode OpenCV image -> JPEG -> base64
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode("utf-8")

            # Prepare payload for vLLM
            payload = {
                "model": "Qwen/Qwen2.5-VL-32B-Instruct-AWQ",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "Describe the scene in this image."},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}}
                        ]
                    }
                ],
                "max_tokens": 200
            }

            # Call vLLM API
            response = requests.post(self.api_url, json=payload, timeout=30)

            if response.status_code == 200:
                data = response.json()
                text = data['choices'][0]['message']['content']

                # Publish to ROS2
                ros_msg = String()
                ros_msg.data = text
                self.publisher.publish(ros_msg)

                self.get_logger().info(f"🤖 VLM Response: {text}")
            else:
                self.get_logger().error(f"❌ API Error {response.status_code}: {response.text}")

        except Exception as e:
            self.get_logger().error(f"⚠️ Error in image_callback: {e}")


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
