#!/usr/bin/env python3
"""
Test Camera Publisher
Publishes test pattern images to simulate a camera feed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime

class TestCameraPublisher(Node):
    def __init__(self):
        super().__init__('test_camera_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_topic', '/test_camera/image')
        self.declare_parameter('frame_rate', 10.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Get parameters
        self.publish_topic = self.get_parameter('publish_topic').value
        frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create publisher
        self.publisher = self.create_publisher(Image, self.publish_topic, 10)
        
        # Create timer for publishing
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        # Frame counter
        self.frame_count = 0
        
        self.get_logger().info(f'Publishing test images to {self.publish_topic} at {frame_rate} fps')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
        
    def publish_frame(self):
        """Generate and publish a test frame"""
        try:
            # Create test pattern
            img = self.generate_test_pattern()
            
            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'test_camera'
            
            # Publish
            self.publisher.publish(img_msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')
    
    def generate_test_pattern(self):
        """Generate a test pattern image"""
        # Create base image
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # Add gradient background
        for y in range(self.image_height):
            for x in range(self.image_width):
                img[y, x] = [
                    int(255 * x / self.image_width),  # Red gradient
                    int(255 * y / self.image_height), # Green gradient
                    128  # Blue constant
                ]
        
        # Add moving circle
        center_x = int(self.image_width/2 + 100 * np.sin(self.frame_count * 0.1))
        center_y = int(self.image_height/2 + 50 * np.cos(self.frame_count * 0.1))
        cv2.circle(img, (center_x, center_y), 30, (255, 255, 255), -1)
        
        # Add text with timestamp and frame count
        timestamp = datetime.now().strftime("%H:%M:%S")
        text = f"Test Camera - Frame: {self.frame_count} - {timestamp}"
        
        # Background rectangle for text
        (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (10, 10), (10 + text_width + 10, 10 + text_height + 10), (0, 0, 0), -1)
        
        # Text
        cv2.putText(img, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add grid pattern
        for i in range(0, self.image_width, 50):
            cv2.line(img, (i, 0), (i, self.image_height), (100, 100, 100), 1)
        for i in range(0, self.image_height, 50):
            cv2.line(img, (0, i), (self.image_width, i), (100, 100, 100), 1)
        
        return img


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_camera = TestCameraPublisher()
        rclpy.spin(test_camera)
    except KeyboardInterrupt:
        pass
    finally:
        test_camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()