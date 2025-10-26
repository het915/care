#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time


class StreamingHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG streaming"""
    
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            
            try:
                while True:
                    if hasattr(self.server, 'current_frame') and self.server.current_frame is not None:
                        # Encode frame as JPEG
                        ret, jpeg = cv2.imencode('.jpg', self.server.current_frame, 
                                                [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if ret:
                            self.wfile.write(b'--jpgboundary\r\n')
                            self.send_header('Content-type', 'image/jpeg')
                            self.send_header('Content-length', str(len(jpeg)))
                            self.end_headers()
                            self.wfile.write(jpeg.tobytes())
                            self.wfile.write(b'\r\n')
                    
                    time.sleep(0.033)  # ~30 FPS
                    
            except (BrokenPipeError, ConnectionResetError):
                pass
                
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head><title>Camera Stream</title></head>
            <body>
            <h1>NV12 to RGB Camera Stream</h1>
            <img src="/stream" width="640" height="480"/>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
        else:
            self.send_error(404)
    
    def log_message(self, format, *args):
        # Suppress default HTTP logging
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Threaded HTTP server for handling multiple clients"""
    pass


class NV12ToRGBBridge(Node):
    """ROS2 node to convert NV12 encoded images to RGB/BGR and stream via HTTP"""
    
    def __init__(self):
        super().__init__('nv12_to_rgb_bridge')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/booster_camera_bridge/StereoNetNode/rectified_image')
        self.declare_parameter('output_topic', '/rgb/image')
        self.declare_parameter('output_encoding', 'bgr8')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('stream_enabled', True)
        self.declare_parameter('stream_host', '0.0.0.0')
        self.declare_parameter('stream_port', 8080)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_encoding = self.get_parameter('output_encoding').value
        queue_size = self.get_parameter('queue_size').value
        self.stream_enabled = self.get_parameter('stream_enabled').value
        self.stream_host = self.get_parameter('stream_host').value
        self.stream_port = self.get_parameter('stream_port').value
        
        self.get_logger().info('Initializing NV12 to RGB Bridge')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Output encoding: {self.output_encoding}')
        
        # Create CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Frame counter for statistics
        self.frame_count = 0
        
        # Current frame for streaming
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Subscribe to NV12 image topic
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            queue_size
        )
        
        # Publisher for RGB/BGR image
        self.image_pub = self.create_publisher(
            Image,
            self.output_topic,
            queue_size
        )
        
        # Start HTTP streaming server
        if self.stream_enabled:
            self.start_streaming_server()
        
        self.get_logger().info('Bridge ready - converting NV12 to RGB')
    
    def start_streaming_server(self):
        """Start the HTTP streaming server in a separate thread"""
        try:
            self.server = ThreadedHTTPServer((self.stream_host, self.stream_port), StreamingHandler)
            self.server.current_frame = None
            
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()
            
            self.get_logger().info(f'Streaming server started at http://{self.stream_host}:{self.stream_port}')
            self.get_logger().info(f'View stream at: http://localhost:{self.stream_port}/stream')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start streaming server: {e}')
            self.stream_enabled = False
    
    def image_callback(self, msg: Image):
        """Process incoming NV12 image and convert to RGB/BGR"""
        
        # Verify encoding
        if msg.encoding.lower() not in ['nv12', 'yuv420sp']:
            self.get_logger().warn(
                f'Expected NV12 encoding but got: {msg.encoding}',
                throttle_duration_sec=5.0
            )
        
        try:
            # Convert NV12 to RGB/BGR
            rgb_msg = self.convert_nv12_to_rgb(msg)
            
            # Publish converted image
            self.image_pub.publish(rgb_msg)
            
            # Update current frame for streaming
            if self.stream_enabled:
                bgr_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                with self.frame_lock:
                    self.server.current_frame = bgr_image
            
            # Log statistics occasionally
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().debug(
                    f'Converted frame {self.frame_count}: '
                    f'{msg.width}x{msg.height} NV12 -> {self.output_encoding}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
    
    def convert_nv12_to_rgb(self, nv12_msg: Image) -> Image:
        """
        Convert NV12 image message to RGB/BGR
        
        NV12 format:
        - Y plane: width x height bytes (luminance)
        - UV plane: width x height/2 bytes (chrominance, interleaved U and V)
        - Total size: width x height x 3/2
        
        Args:
            nv12_msg: Input image message in NV12 format
            
        Returns:
            Image message in RGB or BGR format
        """
        width = nv12_msg.width
        height = nv12_msg.height
        
        # Validate image size
        expected_size = width * height * 3 // 2
        if len(nv12_msg.data) != expected_size:
            raise ValueError(
                f'Invalid NV12 data size. Expected: {expected_size}, '
                f'Got: {len(nv12_msg.data)}'
            )
        
        # Convert ROS image data to numpy array
        nv12_data = np.frombuffer(nv12_msg.data, dtype=np.uint8)
        
        # Reshape to NV12 format (Y plane + UV plane stacked vertically)
        nv12_array = nv12_data.reshape((height * 3 // 2, width))
        
        # Convert NV12 to BGR using OpenCV
        bgr = cv2.cvtColor(nv12_array, cv2.COLOR_YUV2BGR_NV12)
        
        # Convert BGR to RGB if needed
        if self.output_encoding == 'rgb8':
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            output_image = rgb
        else:
            output_image = bgr
        
        # Convert numpy array to ROS Image message
        rgb_msg = self.bridge.cv2_to_imgmsg(output_image, encoding=self.output_encoding)
        
        # Copy header from original message to preserve timestamp and frame_id
        rgb_msg.header = nv12_msg.header
        
        return rgb_msg
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.stream_enabled and hasattr(self, 'server'):
            self.server.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = NV12ToRGBBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()