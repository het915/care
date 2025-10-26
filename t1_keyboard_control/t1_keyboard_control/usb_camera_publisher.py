#!/usr/bin/env python3
"""
USB Camera Publisher Node
Captures video from USB camera and publishes as ROS2 Image messages
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class USBCameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')
        
        # Declare parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('publish_topic', '/camera/image_raw')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', False)
        
        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.publish_topic = self.get_parameter('publish_topic').value
        frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        self.flip_vertical = self.get_parameter('flip_vertical').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create publisher
        self.publisher = self.create_publisher(Image, self.publish_topic, 10)
        
        # Initialize camera
        self.cap = None
        self.camera_thread = None
        self.running = False
        
        # Try to initialize camera
        self.init_camera()
        
        # Create timer for publishing status
        self.timer = self.create_timer(5.0, self.log_status)
        
        self.frame_count = 0
        
        if self.cap and self.cap.isOpened():
            self.get_logger().info(f'USB camera initialized successfully')
            self.get_logger().info(f'Publishing to {self.publish_topic}')
            self.get_logger().info(f'Resolution: {self.image_width}x{self.image_height}')
            self.get_logger().info(f'Target frame rate: {frame_rate} fps')
            
            # Start camera capture thread
            self.running = True
            self.camera_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.camera_thread.start()
        else:
            self.get_logger().error(f'Failed to initialize USB camera {self.device_id}')
            self.get_logger().error('Check if camera is connected and not in use by another application')
    
    def init_camera(self):
        """Initialize USB camera"""
        try:
            self.cap = cv2.VideoCapture(self.device_id)
            
            if self.cap.isOpened():
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                # Get actual camera properties
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
                
                self.get_logger().info(f'Camera actual resolution: {actual_width}x{actual_height}')
                self.get_logger().info(f'Camera actual FPS: {actual_fps}')
                
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error initializing camera: {str(e)}')
            return False
    
    def capture_loop(self):
        """Main camera capture loop"""
        while self.running and rclpy.ok():
            try:
                ret, frame = self.cap.read()
                
                if ret:
                    # Apply flips if requested
                    if self.flip_horizontal and self.flip_vertical:
                        frame = cv2.flip(frame, -1)  # Both axes
                    elif self.flip_horizontal:
                        frame = cv2.flip(frame, 1)   # Horizontal
                    elif self.flip_vertical:
                        frame = cv2.flip(frame, 0)   # Vertical
                    
                    # Convert to ROS Image message
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera_frame'
                    
                    # Publish
                    self.publisher.publish(img_msg)
                    self.frame_count += 1
                else:
                    self.get_logger().warn('Failed to read frame from camera')
                    
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {str(e)}')
                break
    
    def log_status(self):
        """Log periodic status information"""
        if self.cap and self.cap.isOpened():
            self.get_logger().info(f'Camera running - Published {self.frame_count} frames')
        else:
            self.get_logger().warn('Camera not available')
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        self.running = False
        if self.cap:
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = USBCameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()