#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import asyncio
import threading
import base64
import json
from aiohttp import web
import aiohttp


class NV12ToRGBBridge(Node):
    """ROS2 node to convert NV12 encoded images to RGB/BGR and stream via WebSocket"""
    
    def __init__(self):
        super().__init__('nv12_to_rgb_bridge')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/booster_camera_bridge/StereoNetNode/rectified_image')
        self.declare_parameter('output_topic', '/rgb/image')
        self.declare_parameter('output_encoding', 'bgr8')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('stream_enabled', True)
        self.declare_parameter('stream_host', '192.168.0.64')  # Changed to robot IP
        self.declare_parameter('stream_port', 8080)
        self.declare_parameter('jpeg_quality', 85)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_encoding = self.get_parameter('output_encoding').value
        queue_size = self.get_parameter('queue_size').value
        self.stream_enabled = self.get_parameter('stream_enabled').value
        self.stream_host = self.get_parameter('stream_host').value
        self.stream_port = self.get_parameter('stream_port').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.get_logger().info('Initializing NV12 to RGB Bridge with WebSocket')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Output encoding: {self.output_encoding}')
        
        # Create CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Frame counter for statistics
        self.frame_count = 0
        
        # WebSocket clients
        self.ws_clients = set()
        self.clients_lock = threading.Lock()
        
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
        
        # Start WebSocket server
        if self.stream_enabled:
            self.start_websocket_server()
        
        self.get_logger().info('Bridge ready - converting NV12 to RGB')
    
    def start_websocket_server(self):
        """Start the WebSocket server in a separate thread"""
        self.loop = asyncio.new_event_loop()
        self.server_thread = threading.Thread(
            target=self._run_server, 
            args=(self.loop,), 
            daemon=True
        )
        self.server_thread.start()
        
        self.get_logger().info(f'WebSocket server starting at ws://{self.stream_host}:{self.stream_port}/ws')
        self.get_logger().info(f'View stream at: http://{self.stream_host}:{self.stream_port}')
    
    def _run_server(self, loop):
        """Run the aiohttp server"""
        asyncio.set_event_loop(loop)
        
        app = web.Application()
        app.router.add_get('/ws', self.websocket_handler)
        app.router.add_get('/', self.index_handler)
        
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', self.stream_port)  # Bind to all interfaces
        loop.run_until_complete(site.start())
        
        self.get_logger().info(f'WebSocket server started successfully on {self.stream_host}:{self.stream_port}')
        loop.run_forever()
    
    async def index_handler(self, request):
        """Serve HTML page with WebSocket client"""
        html = f"""
        <!DOCTYPE html>
        <html>
        
        <body>
            <h1>Robot Camera Stream - PNG RGBA</h1>
            <div class="server-info">
                <strong>Robot IP:</strong> {self.stream_host}<br>
                <strong>WebSocket URL:</strong> ws://{self.stream_host}:{self.stream_port}/ws<br>
                <strong>Format:</strong> PNG with RGBA channels
            </div>
            <div id="status" class="status disconnected">Disconnected</div>
            <canvas id="canvas"></canvas>
            <div class="info">
                <p>FPS: <span id="fps">0</span> | Frames: <span id="frames">0</span></p>
                <p>Resolution: <span id="resolution">-</span></p>
            </div>
            
            <script>
                const canvas = document.getElementById('canvas');
                const ctx = canvas.getContext('2d');
                const statusDiv = document.getElementById('status');
                const fpsSpan = document.getElementById('fps');
                const framesSpan = document.getElementById('frames');
                const resolutionSpan = document.getElementById('resolution');
                
                let frameCount = 0;
                let lastTime = Date.now();
                let fps = 0;
                
                // WebSocket connection to robot
                const ws = new WebSocket('ws://{self.stream_host}:{self.stream_port}/ws');
                ws.binaryType = 'arraybuffer';
                
                ws.onopen = () => {{
                    console.log('WebSocket connected to robot at {self.stream_host}');
                    statusDiv.textContent = 'Connected to Robot ({self.stream_host})';
                    statusDiv.className = 'status connected';
                }};
                
                ws.onclose = () => {{
                    console.log('WebSocket disconnected from robot');
                    statusDiv.textContent = 'Disconnected from Robot';
                    statusDiv.className = 'status disconnected';
                }};
                
                ws.onerror = (error) => {{
                    console.error('WebSocket error:', error);
                    statusDiv.textContent = 'Connection Error';
                    statusDiv.className = 'status disconnected';
                }};
                
                ws.onmessage = (event) => {{
                    // Expect raw base64 string (compatible with TypeScript code)
                    if (typeof event.data === 'string') {{
                        // Decode base64 image
                        const img = new Image();
                        img.onload = () => {{
                            // Set canvas size on first frame
                            if (canvas.width !== img.width || canvas.height !== img.height) {{
                                canvas.width = img.width;
                                canvas.height = img.height;
                                resolutionSpan.textContent = `${{img.width}}x${{img.height}}`;
                            }}
                            
                            // Draw image
                            ctx.drawImage(img, 0, 0);
                            
                            // Update stats
                            frameCount++;
                            framesSpan.textContent = frameCount;
                            
                            const now = Date.now();
                            const elapsed = (now - lastTime) / 1000;
                            if (elapsed >= 1.0) {{
                                fps = Math.round(frameCount / elapsed);
                                fpsSpan.textContent = fps;
                                frameCount = 0;
                                lastTime = now;
                            }}
                        }};
                        img.src = 'data:image/png;base64,' + event.data;
                    }}
                }};
            </script>
        </body>
        </html>
        """
        return web.Response(text=html, content_type='text/html')
    
    async def websocket_handler(self, request):
        """Handle WebSocket connections"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        with self.clients_lock:
            self.ws_clients.add(ws)
        
        client_ip = request.remote
        self.get_logger().info(f'WebSocket client connected from {client_ip}. Total clients: {len(self.ws_clients)}')
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    if msg.data == 'close':
                        await ws.close()
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    self.get_logger().error(f'WebSocket error: {ws.exception()}')
        finally:
            with self.clients_lock:
                self.ws_clients.discard(ws)
            self.get_logger().info(f'WebSocket client disconnected. Total clients: {len(self.ws_clients)}')
        
        return ws
    
    def broadcast_frame(self, frame):
        """Broadcast frame to all connected WebSocket clients as PNG RGBA"""
        if not self.ws_clients:
            return
        
        try:
            # Always convert to RGBA format for WebSocket streaming
            if len(frame.shape) == 3:
                if frame.shape[2] == 3:
                    # Convert BGR to RGBA (add alpha channel with full opacity)
                    rgba_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                elif frame.shape[2] == 4:
                    # Already RGBA
                    rgba_frame = frame
                else:
                    self.get_logger().error(f"Unsupported frame channels: {frame.shape[2]}")
                    return
            else:
                self.get_logger().error(f"Unsupported frame shape: {frame.shape}")
                return
            
            # Encode as PNG to preserve RGBA channels
            encode_params = [cv2.IMWRITE_PNG_COMPRESSION, 3]  # 0-9, 3 is good balance
            ret, png_data = cv2.imencode('.png', rgba_frame, encode_params)
            
            if not ret:
                self.get_logger().error("Failed to encode frame as PNG RGBA")
                return
            
            # Convert to base64 - send raw base64 string for TypeScript compatibility
            png_base64 = base64.b64encode(png_data).decode('utf-8')
            
            # Send raw base64 PNG string (compatible with TypeScript code)
            message = png_base64
            
            # Send to all clients
            with self.clients_lock:
                disconnected = set()
                for client in self.ws_clients:
                    try:
                        # Schedule the send in the event loop
                        asyncio.run_coroutine_threadsafe(
                            client.send_str(message),
                            self.loop
                        )
                    except Exception as e:
                        self.get_logger().debug(f"Client disconnect: {e}")
                        disconnected.add(client)
                
                # Remove disconnected clients
                self.ws_clients -= disconnected
                
        except Exception as e:
            self.get_logger().error(f'Error broadcasting PNG RGBA frame: {e}')
    
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
            
            # Broadcast to WebSocket clients
            if self.stream_enabled and self.ws_clients:
                rgba_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                self.broadcast_frame(rgba_image)
            
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
        if self.stream_enabled and hasattr(self, 'loop'):
            self.loop.call_soon_threadsafe(self.loop.stop)
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