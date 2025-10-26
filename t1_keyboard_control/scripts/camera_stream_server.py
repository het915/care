#!/usr/bin/env python3
"""
Standalone Camera Stream Server
Direct USB camera streaming with Flask (no ROS2 required)
"""

import cv2
import time
import threading
from flask import Flask, Response, render_template_string
import argparse
import logging

# Suppress Flask development server warning
logging.getLogger('werkzeug').setLevel(logging.ERROR)

class CameraStreamServer:
    def __init__(self, camera_id=0, port=5000, host='0.0.0.0', quality=80, width=640, height=480):
        self.camera_id = camera_id
        self.port = port
        self.host = host
        self.quality = quality
        self.width = width
        self.height = height
        
        self.cap = None
        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = False
        
        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Stats
        self.frame_count = 0
        self.start_time = time.time()
    
    def setup_routes(self):
        """Setup Flask routes"""
        @self.app.route('/')
        def index():
            return render_template_string(self.get_html_template())
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(
                self.generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/stats')
        def stats():
            runtime = time.time() - self.start_time
            fps = self.frame_count / runtime if runtime > 0 else 0
            return {
                'frames_captured': self.frame_count,
                'runtime_seconds': round(runtime, 1),
                'average_fps': round(fps, 1),
                'camera_id': self.camera_id,
                'resolution': f"{self.width}x{self.height}",
                'quality': self.quality
            }
    
    def get_html_template(self):
        """HTML template for the web interface"""
        return '''
        <!DOCTYPE html>
        <html>
        <head>
            <title>USB Camera Stream</title>
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <style>
                body {
                    margin: 0;
                    padding: 20px;
                    background-color: #1a1a1a;
                    color: white;
                    font-family: Arial, sans-serif;
                }
                .container {
                    max-width: 1200px;
                    margin: 0 auto;
                    text-align: center;
                }
                h1 {
                    color: #4CAF50;
                    margin-bottom: 20px;
                }
                .video-container {
                    background-color: #2d2d2d;
                    padding: 20px;
                    border-radius: 10px;
                    margin-bottom: 20px;
                    box-shadow: 0 4px 8px rgba(0,0,0,0.3);
                }
                #camera-feed {
                    max-width: 100%;
                    height: auto;
                    border: 3px solid #4CAF50;
                    border-radius: 8px;
                }
                .info-panel {
                    background-color: #2d2d2d;
                    padding: 15px;
                    border-radius: 8px;
                    margin-top: 20px;
                    text-align: left;
                }
                .info-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                    gap: 15px;
                    margin-top: 15px;
                }
                .info-item {
                    background-color: #3d3d3d;
                    padding: 10px;
                    border-radius: 5px;
                }
                .info-label {
                    color: #4CAF50;
                    font-weight: bold;
                }
                .controls {
                    margin: 20px 0;
                }
                button {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 10px 20px;
                    margin: 5px;
                    border-radius: 5px;
                    cursor: pointer;
                    font-size: 14px;
                }
                button:hover {
                    background-color: #45a049;
                }
                .status {
                    color: #4CAF50;
                    font-weight: bold;
                }
                @media (max-width: 768px) {
                    body { padding: 10px; }
                    h1 { font-size: 24px; }
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>📹 USB Camera Stream</h1>
                
                <div class="video-container">
                    <img id="camera-feed" src="/video_feed" alt="Camera Feed">
                </div>
                
                <div class="controls">
                    <button onclick="refreshStream()">🔄 Refresh Stream</button>
                    <button onclick="toggleFullscreen()">⛶ Fullscreen</button>
                    <button onclick="updateStats()">📊 Update Stats</button>
                </div>
                
                <div class="info-panel">
                    <h3>📋 Stream Information</h3>
                    <div class="info-grid">
                        <div class="info-item">
                            <div class="info-label">Direct Stream URL:</div>
                            <div id="stream-url">http://''' + self.host + ''':''' + str(self.port) + '''/video_feed</div>
                        </div>
                        <div class="info-item">
                            <div class="info-label">Camera Device:</div>
                            <div>/dev/video''' + str(self.camera_id) + '''</div>
                        </div>
                        <div class="info-item">
                            <div class="info-label">Resolution:</div>
                            <div>''' + str(self.width) + '''x''' + str(self.height) + '''</div>
                        </div>
                        <div class="info-item">
                            <div class="info-label">JPEG Quality:</div>
                            <div>''' + str(self.quality) + '''%</div>
                        </div>
                        <div class="info-item">
                            <div class="info-label">Status:</div>
                            <div class="status" id="status">🟢 Live</div>
                        </div>
                        <div class="info-item">
                            <div class="info-label">Stats:</div>
                            <div id="stats">Loading...</div>
                        </div>
                    </div>
                </div>
            </div>

            <script>
                function refreshStream() {
                    document.getElementById('camera-feed').src = '/video_feed?' + new Date().getTime();
                }
                
                function toggleFullscreen() {
                    const img = document.getElementById('camera-feed');
                    if (!document.fullscreenElement) {
                        img.requestFullscreen();
                    } else {
                        document.exitFullscreen();
                    }
                }
                
                function updateStats() {
                    fetch('/stats')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('stats').innerHTML = 
                                `Frames: ${data.frames_captured}<br>` +
                                `Runtime: ${data.runtime_seconds}s<br>` +
                                `Avg FPS: ${data.average_fps}`;
                        })
                        .catch(error => {
                            document.getElementById('stats').innerHTML = 'Error loading stats';
                        });
                }
                
                // Auto-update stats every 5 seconds
                setInterval(updateStats, 5000);
                updateStats(); // Initial load
                
                // Check stream status
                const img = document.getElementById('camera-feed');
                img.onerror = function() {
                    document.getElementById('status').innerHTML = '🔴 Stream Error';
                };
                img.onload = function() {
                    document.getElementById('status').innerHTML = '🟢 Live';
                };
            </script>
        </body>
        </html>
        '''
    
    def init_camera(self):
        """Initialize the USB camera"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            
            if not self.cap.isOpened():
                print(f"❌ Error: Could not open camera {self.camera_id}")
                return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Get actual camera properties
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            print(f"📹 Camera {self.camera_id} initialized successfully")
            print(f"   Resolution: {actual_width}x{actual_height}")
            print(f"   FPS: {actual_fps}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error initializing camera: {e}")
            return False
    
    def capture_frames(self):
        """Background thread to capture frames from camera"""
        while self.running:
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.frame = frame
                        self.frame_count += 1
                else:
                    print("⚠️  Warning: Failed to read frame")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"❌ Error capturing frame: {e}")
                time.sleep(1)
    
    def generate_frames(self):
        """Generator function for Flask streaming"""
        while True:
            with self.frame_lock:
                if self.frame is None:
                    time.sleep(0.1)
                    continue
                frame = self.frame.copy()
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, 
                                      [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            
            if not ret:
                continue
            
            frame_bytes = buffer.tobytes()
            
            # Yield frame in multipart format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    def start(self):
        """Start the camera stream server"""
        print("🚀 Starting USB Camera Stream Server")
        print("=" * 50)
        
        # Initialize camera
        if not self.init_camera():
            return False
        
        # Start capture thread
        self.running = True
        capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
        capture_thread.start()
        
        print(f"🌐 Web Interface: http://{self.host}:{self.port}")
        print(f"📹 Direct Stream: http://{self.host}:{self.port}/video_feed")
        print(f"📊 Stats API: http://{self.host}:{self.port}/stats")
        print("=" * 50)
        print("Press Ctrl+C to stop the server")
        
        try:
            # Start Flask server
            self.app.run(host=self.host, port=self.port, debug=False, threaded=True)
        except KeyboardInterrupt:
            print("\n🛑 Shutting down server...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        if self.cap:
            self.cap.release()
        print("✅ Cleanup completed")


def main():
    parser = argparse.ArgumentParser(description='USB Camera Stream Server')
    parser.add_argument('--camera', '-c', type=int, default=0, 
                       help='Camera device ID (default: 0 for /dev/video0)')
    parser.add_argument('--port', '-p', type=int, default=5000,
                       help='HTTP server port (default: 5000)')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='HTTP server host (default: 0.0.0.0)')
    parser.add_argument('--quality', '-q', type=int, default=80,
                       help='JPEG quality 1-100 (default: 80)')
    parser.add_argument('--width', '-w', type=int, default=640,
                       help='Camera width (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                       help='Camera height (default: 480)')
    
    args = parser.parse_args()
    
    # Create and start server
    server = CameraStreamServer(
        camera_id=args.camera,
        port=args.port,
        host=args.host,
        quality=args.quality,
        width=args.width,
        height=args.height
    )
    
    server.start()


if __name__ == '__main__':
    main()