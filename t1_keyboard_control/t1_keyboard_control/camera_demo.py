#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np


class CameraDemo(Node):
    """
    Demo node that publishes synthetic camera images for testing the camera viewer.
    
    This creates several synthetic camera feeds to demonstrate the camera viewer functionality.
    """
    
    def __init__(self):
        super().__init__('camera_demo')
        
        self.bridge = CvBridge()
        
        # Create publishers for different camera topics
        self.front_camera_pub = self.create_publisher(Image, '/t1/camera/front/image_raw', 10)
        self.back_camera_pub = self.create_publisher(Image, '/t1/camera/back/image_raw', 10)
        self.left_camera_pub = self.create_publisher(CompressedImage, '/t1/camera/left/image_raw/compressed', 10)
        self.right_camera_pub = self.create_publisher(CompressedImage, '/t1/camera/right/image_raw/compressed', 10)
        
        # Create timer to publish images
        self.timer = self.create_timer(0.1, self.publish_images)  # 10 Hz
        
        self.frame_count = 0
        
        self.get_logger().info("Camera demo started - publishing synthetic camera feeds")
        self.get_logger().info("Publishing to:")
        self.get_logger().info("  /t1/camera/front/image_raw (Image)")  
        self.get_logger().info("  /t1/camera/back/image_raw (Image)")
        self.get_logger().info("  /t1/camera/left/image_raw/compressed (CompressedImage)")
        self.get_logger().info("  /t1/camera/right/image_raw/compressed (CompressedImage)")
        
    def create_synthetic_image(self, width, height, camera_name, frame_count):
        """Create a synthetic camera image with text overlay"""
        # Create a colorful background
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Different colors for different cameras
        if 'front' in camera_name:
            img[:, :] = [100, 150, 200]  # Light blue
        elif 'back' in camera_name:
            img[:, :] = [100, 200, 100]  # Light green
        elif 'left' in camera_name:
            img[:, :] = [200, 100, 100]  # Light red
        elif 'right' in camera_name:
            img[:, :] = [150, 100, 200]  # Light purple
        
        # Add moving rectangle
        x = int((width * 0.8) * (0.5 + 0.4 * np.sin(frame_count * 0.1)))
        y = int((height * 0.8) * (0.5 + 0.4 * np.cos(frame_count * 0.1)))
        cv2.rectangle(img, (x, y), (x + 50, y + 30), (255, 255, 255), -1)
        
        # Add camera name and frame info
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f"{camera_name.upper()}", (10, 30), font, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Frame: {frame_count}", (10, height - 20), font, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"{width}x{height}", (10, 70), font, 0.7, (255, 255, 255), 2)
        
        # Add timestamp
        timestamp = self.get_clock().now().nanoseconds // 1_000_000  # Convert to milliseconds
        cv2.putText(img, f"Time: {timestamp}", (10, height - 50), font, 0.5, (255, 255, 255), 1)
        
        return img
    
    def publish_images(self):
        """Publish synthetic images to all camera topics"""
        self.frame_count += 1
        current_time = self.get_clock().now().to_msg()
        
        try:
            # Front camera - raw image
            front_img = self.create_synthetic_image(640, 480, "front_camera", self.frame_count)
            front_msg = self.bridge.cv2_to_imgmsg(front_img, encoding="bgr8")
            front_msg.header = Header()
            front_msg.header.stamp = current_time
            front_msg.header.frame_id = "front_camera_frame"
            self.front_camera_pub.publish(front_msg)
            
            # Back camera - raw image
            back_img = self.create_synthetic_image(640, 480, "back_camera", self.frame_count)
            back_msg = self.bridge.cv2_to_imgmsg(back_img, encoding="bgr8")
            back_msg.header = Header()
            back_msg.header.stamp = current_time
            back_msg.header.frame_id = "back_camera_frame"
            self.back_camera_pub.publish(back_msg)
            
            # Left camera - compressed image
            left_img = self.create_synthetic_image(320, 240, "left_camera", self.frame_count)
            left_compressed = CompressedImage()
            left_compressed.header = Header()
            left_compressed.header.stamp = current_time
            left_compressed.header.frame_id = "left_camera_frame"
            left_compressed.format = "jpeg"
            _, encoded_img = cv2.imencode('.jpg', left_img)
            left_compressed.data = encoded_img.tobytes()
            self.left_camera_pub.publish(left_compressed)
            
            # Right camera - compressed image
            right_img = self.create_synthetic_image(320, 240, "right_camera", self.frame_count)
            right_compressed = CompressedImage()
            right_compressed.header = Header()
            right_compressed.header.stamp = current_time
            right_compressed.header.frame_id = "right_camera_frame"
            right_compressed.format = "jpeg"
            _, encoded_img = cv2.imencode('.jpg', right_img)
            right_compressed.data = encoded_img.tobytes()
            self.right_camera_pub.publish(right_compressed)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing images: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera demo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()