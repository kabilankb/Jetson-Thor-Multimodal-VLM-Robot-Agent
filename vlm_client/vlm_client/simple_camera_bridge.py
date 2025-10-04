#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import time


class SimpleCameraBridge(Node):
    def __init__(self):
        super().__init__('simple_camera_bridge')
        
        # Bridge for ROS <-> OpenCV
        self.bridge = CvBridge()
        
        # Subscriber: camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher: base64 encoded images for web interface
        self.web_image_publisher = self.create_publisher(
            String, 
            '/camera/web_feed', 
            10
        )
        
        # Throttling for web interface
        self.last_publish_time = 0
        self.publish_interval = 0.2  # 5 FPS
        
        self.get_logger().info("✅ Simple Camera Bridge started")
        self.get_logger().info("📷 Subscribing to /camera/image_raw")
        self.get_logger().info("🌐 Publishing base64 images to /camera/web_feed")

    def image_callback(self, msg):
        """Convert ROS image to base64 for web interface"""
        try:
            current_time = time.time()
            
            # Throttle publishing to avoid overwhelming web interface
            if current_time - self.last_publish_time < self.publish_interval:
                return
                
            # Convert ROS2 Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize image for web display (optional, for performance)
            height, width = cv_image.shape[:2]
            if width > 640:  # Resize large images for web performance
                scale = 640 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Encode as JPEG and convert to base64
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Publish as ROS2 String message
            web_msg = String()
            web_msg.data = image_base64
            self.web_image_publisher.publish(web_msg)
            
            self.last_publish_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in camera bridge: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCameraBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down Camera Bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
