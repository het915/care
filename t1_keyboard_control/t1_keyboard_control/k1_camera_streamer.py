#!/usr/bin/env python3
"""
K1 Camera Streamer Node
Subscribes to the Zed camera topics and streams video over HTTP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from flask import Flask, Response
import numpy as np

class K1CameraStreamer(Node):
    def __init__(self):
        super().__init__('k1_camera_streamer')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('stream_port', 5000)
        self.declare_parameter('stream_host', '0.0.0.0')
        self.declare_parameter('jpeg_quality', 80)
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        self.stream_port = self.get_parameter('stream_port').value
        self.stream_host = self.get_parameter('stream_host').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Latest frame storage
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Subscribed to {camera_topic}')
        self.get_logger().info(f'Starting HTTP stream on {self.stream_host}:{self.stream_port}')
        
        # Start Flask server in a separate thread
        self.flask_thread = threading.Thread(target=self.start_flask_server, daemon=True)
        self.flask_thread.start()
    
    def image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.frame_lock:
                self.latest_frame = cv_image
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def generate_frames(self):
        """Generator function for streaming frames"""
        while True:
            with self.frame_lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, 
                                      [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            
            if not ret:
                continue
                
            frame_bytes = buffer.tobytes()
            
            # Yield frame in multipart format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    def start_flask_server(self):
        """Start Flask HTTP server for streaming"""
        app = Flask(__name__)
        
        @app.route('/')
        def index():
            html_content = f"""
            <html>
            <head>
                <title>K1 Camera Stream</title>
                <style>
                    body {{
                        margin: 0;
                        padding: 20px;
                        background-color: #1a1a1a;
                        color: white;
                        font-family: Arial, sans-serif;
                    }}
                    h1 {{
                        text-align: center;
                    }}
                    .container {{
                        max-width: 1200px;
                        margin: 0 auto;
                    }}
                    img {{
                        width: 100%;
                        height: auto;
                        border: 2px solid #4CAF50;
                        border-radius: 8px;
                    }}
                    .info {{
                        background-color: #2d2d2d;
                        padding: 15px;
                        border-radius: 8px;
                        margin-top: 20px;
                    }}
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>🤖 K1 Robot Camera Stream</h1>
                    <img src="/video_feed" alt="Camera Feed">
                    <div class="info">
                        <h3>Stream Information</h3>
                        <p><strong>Direct MJPEG Stream URL:</strong> http://192.168.10.10:5000/video_feed</p>
                        <p><strong>Camera:</strong> Zed Depth Camera</p>
                        <p><strong>Quality:</strong> JPEG {self.jpeg_quality}</p>
                        <p><strong>Access from browser:</strong> http://192.168.10.10:5000</p>
                    </div>
                </div>
            </body>
            </html>
            """
            return html_content
        
        @app.route('/video_feed')
        def video_feed():
            return Response(
                self.generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        # Run Flask server
        app.run(host=self.stream_host, port=self.stream_port, threaded=True)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_streamer = K1CameraStreamer()
        rclpy.spin(camera_streamer)
    except KeyboardInterrupt:
        pass
    finally:
        camera_streamer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()