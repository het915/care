#!/usr/bin/env python3
"""
Image Converter Node
Converts between different image encodings for RViz compatibility
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('input_topic', '/k1_camera/image_raw')
        self.declare_parameter('output_topic', '/k1_camera/image_raw/converted')
        self.declare_parameter('output_encoding', 'bgr8')
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.output_encoding = self.get_parameter('output_encoding').get_parameter_value().string_value
        
        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        self.get_logger().info(f'Image Converter started')
        self.get_logger().info(f'Input: {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')
        self.get_logger().info(f'Output encoding: {self.output_encoding}')

    def image_callback(self, msg):
        """Convert and republish image with proper encoding"""
        try:
            # Convert ROS image to OpenCV format
            if msg.encoding == 'nv12':
                # Handle NV12 format specifically
                cv_image = self.convert_nv12_to_bgr(msg)
            else:
                # Use cv_bridge for other formats
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert back to ROS message with desired encoding
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=self.output_encoding)
            output_msg.header = msg.header
            
            # Publish converted image
            self.publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')

    def convert_nv12_to_bgr(self, msg):
        """Convert NV12 format to BGR"""
        try:
            # NV12 format: Y plane followed by interleaved UV plane
            height = msg.height
            width = msg.width
            
            # Extract Y and UV data
            y_size = height * width
            uv_size = (height // 2) * width
            
            y_data = msg.data[:y_size]
            uv_data = msg.data[y_size:y_size + uv_size]
            
            # Create numpy arrays
            import numpy as np
            y_plane = np.frombuffer(y_data, dtype=np.uint8).reshape((height, width))
            uv_plane = np.frombuffer(uv_data, dtype=np.uint8).reshape((height // 2, width))
            
            # Convert to YUV420
            yuv = np.zeros((int(height * 1.5), width), dtype=np.uint8)
            yuv[:height, :] = y_plane
            yuv[height:height + height//4, :] = uv_plane[::2, ::2]  # U
            yuv[height + height//4:, :] = uv_plane[1::2, 1::2]      # V
            
            # Convert to BGR
            yuv_image = yuv.reshape((int(height * 1.5), width))
            bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV420p2BGR)
            
            return bgr_image
            
        except Exception as e:
            self.get_logger().error(f'NV12 conversion failed: {e}')
            # Return a black image as fallback
            import numpy as np
            return np.zeros((msg.height, msg.width, 3), dtype=np.uint8)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImageConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()