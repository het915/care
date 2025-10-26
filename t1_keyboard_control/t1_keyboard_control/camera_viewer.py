#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import threading
import time
import numpy as np


class CameraViewer(Node):
    """
    A ROS2 node that subscribes to camera topics and republishes them for RViz visualization.
    
    This node automatically discovers camera topics and creates subscribers for them.
    It handles both raw Image messages and CompressedImage messages.
    """
    
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Dictionary to store subscribers and publishers
        self.camera_subscribers = {}
        self.camera_publishers = {}
        
        # Dictionary to store latest images for each camera
        self.latest_images = {}
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        self.get_logger().info("Camera Viewer Node started")
        
        # Create a timer to periodically discover new camera topics
        self.discovery_timer = self.create_timer(5.0, self.discover_camera_topics)
        
        # Initial discovery
        self.discover_camera_topics()
        
    def discover_camera_topics(self):
        """Discover and subscribe to camera topics"""
        try:
            # Get all available topics
            topic_names_and_types = self.get_topic_names_and_types()
            
            camera_topics = []
            
            # Look for camera-related topics (but not our own republished ones)
            for topic_name, topic_types in topic_names_and_types:
                # Skip our own republished topics to avoid infinite loop
                if topic_name.startswith('/camera_viewer/'):
                    continue
                    
                # Check if it's an image topic
                if 'sensor_msgs/msg/Image' in topic_types:
                    camera_topics.append((topic_name, 'Image'))
                elif 'sensor_msgs/msg/CompressedImage' in topic_types:
                    camera_topics.append((topic_name, 'CompressedImage'))
                # Also check for common camera topic patterns
                elif any(pattern in topic_name.lower() for pattern in 
                        ['camera', 'image', 'rgb', 'depth', 'stereo', 'front', 'back', 'left', 'right']):
                    if 'sensor_msgs/msg/Image' in topic_types:
                        camera_topics.append((topic_name, 'Image'))
                    elif 'sensor_msgs/msg/CompressedImage' in topic_types:
                        camera_topics.append((topic_name, 'CompressedImage'))
            
            # Subscribe to new camera topics
            for topic_name, msg_type in camera_topics:
                if topic_name not in self.camera_subscribers:
                    self.subscribe_to_camera_topic(topic_name, msg_type)
                    
        except Exception as e:
            self.get_logger().warn(f"Error discovering camera topics: {str(e)}")
    
    def subscribe_to_camera_topic(self, topic_name, msg_type):
        """Subscribe to a specific camera topic"""
        try:
            # Create safe topic name for republishing (replace / with _)
            safe_name = topic_name.replace('/', '_').strip('_')
            output_topic = f"/camera_viewer{topic_name}"
            
            if msg_type == 'Image':
                subscriber = self.create_subscription(
                    Image,
                    topic_name,
                    lambda msg, tn=topic_name: self.image_callback(msg, tn),
                    10
                )
                
                # Create publisher for RViz
                publisher = self.create_publisher(Image, output_topic, 10)
                
            elif msg_type == 'CompressedImage':
                subscriber = self.create_subscription(
                    CompressedImage,
                    topic_name,
                    lambda msg, tn=topic_name: self.compressed_image_callback(msg, tn),
                    10
                )
                
                # Create publisher for RViz (convert compressed to raw)
                publisher = self.create_publisher(Image, output_topic, 10)
            
            self.camera_subscribers[topic_name] = subscriber
            self.camera_publishers[topic_name] = publisher
            self.latest_images[topic_name] = None
            
            self.get_logger().info(f"Subscribed to {topic_name} ({msg_type}) -> {output_topic}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic_name}: {str(e)}")
    
    def image_callback(self, msg, topic_name):
        """Callback for raw Image messages"""
        try:
            with self.lock:
                self.latest_images[topic_name] = msg
                
            # Republish for RViz
            if topic_name in self.camera_publishers:
                # Create new message with updated timestamp
                output_msg = Image()
                output_msg.header = Header()
                output_msg.header.stamp = self.get_clock().now().to_msg()
                output_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_frame"
                output_msg.height = msg.height
                output_msg.width = msg.width
                output_msg.encoding = msg.encoding
                output_msg.is_bigendian = msg.is_bigendian
                output_msg.step = msg.step
                output_msg.data = msg.data
                
                self.camera_publishers[topic_name].publish(output_msg)
                
            self.get_logger().debug(f"Received image from {topic_name}: {msg.width}x{msg.height}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image from {topic_name}: {str(e)}")
    
    def compressed_image_callback(self, msg, topic_name):
        """Callback for CompressedImage messages"""
        try:
            # Convert compressed image to raw image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                # Convert to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                ros_image.header = Header()
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_frame"
                
                with self.lock:
                    self.latest_images[topic_name] = ros_image
                
                # Republish for RViz
                if topic_name in self.camera_publishers:
                    self.camera_publishers[topic_name].publish(ros_image)
                
                self.get_logger().debug(f"Received compressed image from {topic_name}: {cv_image.shape[1]}x{cv_image.shape[0]}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image from {topic_name}: {str(e)}")
    
    def get_camera_info(self):
        """Get information about subscribed cameras"""
        with self.lock:
            info = {}
            for topic_name, image_msg in self.latest_images.items():
                if image_msg is not None:
                    info[topic_name] = {
                        'width': image_msg.width,
                        'height': image_msg.height,
                        'encoding': image_msg.encoding,
                        'frame_id': image_msg.header.frame_id
                    }
            return info


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraViewer()
    
    try:
        # Print initial information
        node.get_logger().info("=== T1 Robot Camera Viewer ===")
        node.get_logger().info("This node will automatically discover and subscribe to camera topics.")
        node.get_logger().info("Camera feeds will be republished to /camera_viewer/* topics for RViz visualization.")
        node.get_logger().info("")
        node.get_logger().info("To view cameras in RViz:")
        node.get_logger().info("1. Start RViz: rviz2")
        node.get_logger().info("2. Add -> By topic -> /camera_viewer/* -> Image")
        node.get_logger().info("3. Set Fixed Frame to 'camera_frame' or appropriate frame")
        node.get_logger().info("")
        
        # Spin with a separate thread to allow for periodic status updates
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        # Create a thread for status updates
        def status_updater():
            while rclpy.ok():
                time.sleep(10)  # Update every 10 seconds
                try:
                    camera_info = node.get_camera_info()
                    if camera_info:
                        node.get_logger().info(f"Active cameras: {len(camera_info)}")
                        for topic, info in camera_info.items():
                            node.get_logger().info(
                                f"  {topic}: {info['width']}x{info['height']} ({info['encoding']}) "
                                f"[{info['frame_id']}]"
                            )
                    else:
                        node.get_logger().info("No camera topics detected. Searching...")
                except Exception as e:
                    node.get_logger().debug(f"Status update error: {str(e)}")
        
        status_thread = threading.Thread(target=status_updater, daemon=True)
        status_thread.start()
        
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera viewer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()