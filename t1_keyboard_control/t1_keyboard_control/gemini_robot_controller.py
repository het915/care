#!/usr/bin/env python3
"""
Gemini Robot Controller
Integrates Google Gemini Vision API with robot neck control and movement
to find objects and approach them based on natural language prompts
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool, Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import json
import base64
from typing import Optional, Dict, List, Tuple
import math

try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    print("⚠️  Google Generative AI not installed. Run: pip install google-generativeai")
    GEMINI_AVAILABLE = False

# Import Booster SDK for actual robot control
try:
    from booster_robotics_sdk_python import (
        B1LocoClient, 
        ChannelFactory,
        RobotMode,
        GetModeResponse,
        B1HandAction
    )
    BOOSTER_SDK_AVAILABLE = True
except ImportError:
    print("⚠️  Booster SDK not found. Robot movement will be simulated.")
    BOOSTER_SDK_AVAILABLE = False

class GeminiRobotController(Node):
    def __init__(self):
        super().__init__('gemini_robot_controller')
        
        # Declare parameters
        self.declare_parameter('gemini_api_key', '')
        self.declare_parameter('camera_topic', '/k1_camera/image_raw')
        self.declare_parameter('neck_control_topic', '/robot/neck_position')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('search_mode', True)
        self.declare_parameter('detection_confidence', 0.7)
        self.declare_parameter('approach_distance', 1.0)  # meters
        
        # Get parameters
        self.gemini_api_key = self.get_parameter('gemini_api_key').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.neck_control_topic = self.get_parameter('neck_control_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.search_mode = self.get_parameter('search_mode').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.approach_distance = self.get_parameter('approach_distance').value
        
        # Initialize Gemini AI
        self.gemini_model = None
        if GEMINI_AVAILABLE and self.gemini_api_key:
            try:
                genai.configure(api_key=self.gemini_api_key)
                self.gemini_model = genai.GenerativeModel('gemini-1.5-pro-vision-latest')
                self.get_logger().info("✅ Gemini Vision API initialized")
            except Exception as e:
                self.get_logger().error(f"❌ Gemini API initialization failed: {e}")
        else:
            self.get_logger().warn("⚠️  Gemini API not available - using mock detection")
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Robot state
        self.current_image = None
        self.target_object = None
        self.object_position = None  # (x, y) in image coordinates
        self.is_searching = False
        self.neck_pan = 0.0  # Current neck pan angle
        self.neck_tilt = 0.0  # Current neck tilt angle
        self.robot_mode = "idle"  # idle, searching, tracking, approaching
        
        # Neck control parameters
        self.neck_search_speed = 0.5  # rad/s
        self.neck_pan_range = (-1.5, 1.5)  # radians
        self.neck_tilt_range = (-0.8, 0.3)  # radians
        self.search_pattern_step = 0
        
        # Movement parameters  
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Robot hardware interface
        self.robot_client = None
        self.vx = 0.0  # Forward/backward velocity
        self.vy = 0.0  # Left/right velocity 
        self.vyaw = 0.0  # Rotation velocity
        
        # Initialize robot SDK
        if BOOSTER_SDK_AVAILABLE:
            try:
                channel_factory = ChannelFactory()
                channel_factory.Init(0, "eno1")
                self.robot_client = channel_factory.CreateLocoClient()
                self.get_logger().info("🤖 Booster SDK initialized for robot control")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to initialize Booster SDK: {e}")
                self.robot_client = None
        else:
            self.get_logger().warn("⚠️  Booster SDK not available - movement will be simulated")
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.neck_control_pub = self.create_publisher(Float64MultiArray, self.neck_control_topic, 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        self.detection_pub = self.create_publisher(String, '/robot/detection', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )
        self.prompt_sub = self.create_subscription(
            String, '/robot/find_object', self.prompt_callback, 10
        )
        self.control_sub = self.create_subscription(
            String, '/robot/control', self.control_callback, 10
        )
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control
        self.neck_timer = self.create_timer(0.05, self.neck_control_loop)  # 20Hz neck control
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1Hz status
        
        # Threading
        self.detection_lock = threading.Lock()
        self.last_detection_time = 0
        
        self.get_logger().info("🤖 Gemini Robot Controller initialized")
        self.get_logger().info(f"📹 Listening on camera topic: {self.camera_topic}")
        self.get_logger().info(f"🎯 Send object detection prompts to: /robot/find_object")
        
    def prompt_callback(self, msg):
        """Handle object detection prompts"""
        prompt = msg.data.strip()
        if not prompt:
            return
            
        self.get_logger().info(f"🎯 New detection request: '{prompt}'")
        self.target_object = prompt
        self.object_position = None
        self.robot_mode = "searching"
        self.is_searching = True
        self.search_pattern_step = 0
        
        # Start object detection in separate thread
        detection_thread = threading.Thread(
            target=self.start_object_detection, 
            args=(prompt,), 
            daemon=True
        )
        detection_thread.start()
    
    def control_callback(self, msg):
        """Handle robot control commands"""
        command = msg.data.strip().lower()
        
        if command == "stop":
            self.robot_mode = "idle"
            self.is_searching = False
            self.stop_robot()
            self.get_logger().info("🛑 Robot stopped")
            
        elif command == "search":
            self.robot_mode = "searching"
            self.is_searching = True
            self.search_pattern_step = 0
            self.get_logger().info("🔍 Starting search mode")
            
        elif command == "home":
            self.move_neck_to_position(0.0, 0.0)
            self.robot_mode = "idle"
            self.is_searching = False
            self.get_logger().info("🏠 Moving to home position")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            if msg.encoding == "nv12":
                # Handle NV12 encoding (common for hardware cameras)
                self.current_image = self.convert_nv12_to_bgr(msg)
            else:
                # Standard encodings
                self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
        except Exception as e:
            self.get_logger().error(f"❌ Image conversion error: {e}")
    
    def convert_nv12_to_bgr(self, img_msg):
        """Convert NV12 image format to BGR"""
        try:
            # NV12 is YUV420 format - Y plane followed by interleaved UV
            height = img_msg.height
            width = img_msg.width
            
            # Extract Y and UV planes
            y_plane = np.frombuffer(img_msg.data[:height * width], dtype=np.uint8).reshape((height, width))
            uv_plane = np.frombuffer(img_msg.data[height * width:], dtype=np.uint8).reshape((height // 2, width))
            
            # Convert to YUV image
            yuv_img = np.zeros((height * 3 // 2, width), dtype=np.uint8)
            yuv_img[:height, :] = y_plane
            yuv_img[height:height + height//4, :] = uv_plane[::2, ::2]  # U plane
            yuv_img[height + height//4:, :] = uv_plane[1::2, 1::2]  # V plane
            
            # Convert YUV to BGR
            bgr_img = cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR_NV12)
            return bgr_img
            
        except Exception as e:
            self.get_logger().error(f"❌ NV12 conversion failed: {e}")
            # Fallback: create a placeholder image
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def start_object_detection(self, target_object):
        """Start continuous object detection for target"""
        self.get_logger().info(f"🔍 Starting detection for: {target_object}")
        
        detection_count = 0
        max_attempts = 50  # 5 seconds at 10Hz
        
        while self.is_searching and detection_count < max_attempts:
            if self.current_image is not None:
                with self.detection_lock:
                    position = self.detect_object_with_gemini(self.current_image, target_object)
                    
                    if position:
                        self.object_position = position
                        self.robot_mode = "tracking"
                        self.get_logger().info(f"✅ Found {target_object} at position {position}")
                        
                        # Publish detection result
                        detection_msg = String()
                        detection_msg.data = json.dumps({
                            "object": target_object,
                            "position": position,
                            "timestamp": time.time()
                        })
                        self.detection_pub.publish(detection_msg)
                        break
                    
            detection_count += 1
            time.sleep(0.1)  # 10Hz detection rate
            
        if detection_count >= max_attempts:
            self.get_logger().warn(f"⚠️  Could not find {target_object} after {max_attempts} attempts")
            self.robot_mode = "idle"
            self.is_searching = False
    
    def detect_object_with_gemini(self, image, target_object):
        """Use Gemini Vision API to detect objects in image"""
        if not self.gemini_model:
            # Mock detection for testing without API
            return self.mock_object_detection(image, target_object)
        
        try:
            # Convert image to base64 for Gemini API
            _, buffer = cv2.imencode('.jpg', image)
            image_data = base64.b64encode(buffer).decode('utf-8')
            
            # Create prompt for object detection
            prompt = f"""
            I'm looking for a {target_object} in this image. Please analyze the image and:
            1. Determine if there is a {target_object} visible
            2. If found, estimate its center position as percentages (0-100) from top-left corner
            3. Return response in JSON format: {{"found": true/false, "x": percentage, "y": percentage, "confidence": 0.0-1.0}}
            
            Be precise with the position coordinates. If multiple {target_object}s are visible, choose the largest/most prominent one.
            """
            
            # Call Gemini API
            response = self.gemini_model.generate_content([prompt, {"mime_type": "image/jpeg", "data": image_data}])
            
            # Parse response
            try:
                # Extract JSON from response text
                response_text = response.text.strip()
                if '```json' in response_text:
                    json_start = response_text.find('```json') + 7
                    json_end = response_text.find('```', json_start)
                    response_text = response_text[json_start:json_end].strip()
                
                result = json.loads(response_text)
                
                if result.get('found', False) and result.get('confidence', 0) > self.detection_confidence:
                    # Convert percentages to pixel coordinates
                    image_height, image_width = image.shape[:2]
                    x_pixel = int(result['x'] * image_width / 100.0)
                    y_pixel = int(result['y'] * image_height / 100.0)
                    
                    return (x_pixel, y_pixel)
                    
            except json.JSONDecodeError as e:
                self.get_logger().error(f"❌ JSON parsing error: {e}")
                self.get_logger().debug(f"Response text: {response.text}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Gemini API error: {e}")
            
        return None
    
    def mock_object_detection(self, image, target_object):
        """Mock object detection for testing without Gemini API"""
        # Simple color-based detection as fallback
        if image is None:
            return None
            
        height, width = image.shape[:2]
        
        # Look for red objects as example (you can expand this)
        if "red" in target_object.lower() or "apple" in target_object.lower():
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Red color range in HSV
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                    # Calculate center
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        return (cx, cy)
        
        return None
    
    def control_loop(self):
        """Main control loop for robot behavior"""
        if self.robot_mode == "searching":
            self.search_behavior()
        elif self.robot_mode == "tracking":
            self.tracking_behavior()
        elif self.robot_mode == "approaching":
            self.approach_behavior()
        elif self.robot_mode == "idle":
            self.idle_behavior()
    
    def search_behavior(self):
        """Search for objects by moving neck in pattern"""
        if not self.is_searching:
            return
        
        # Implement search pattern (sweep neck left-right, up-down)
        search_positions = [
            (0.0, 0.0),    # Center
            (-0.8, 0.0),   # Left
            (0.8, 0.0),    # Right
            (0.0, 0.3),    # Up
            (0.0, -0.5),   # Down
            (-0.8, 0.3),   # Top-left
            (0.8, 0.3),    # Top-right
            (-0.8, -0.5),  # Bottom-left
            (0.8, -0.5),   # Bottom-right
        ]
        
        if self.search_pattern_step < len(search_positions):
            pan, tilt = search_positions[self.search_pattern_step]
            self.move_neck_to_position(pan, tilt)
            
            # Move to next position every 2 seconds
            if time.time() - self.last_detection_time > 2.0:
                self.search_pattern_step += 1
                self.last_detection_time = time.time()
        else:
            # Reset search pattern
            self.search_pattern_step = 0
    
    def tracking_behavior(self):
        """Track detected object with neck movement"""
        if self.object_position is None:
            self.robot_mode = "searching"
            return
        
        if self.current_image is None:
            return
        
        # Calculate neck angles to center object
        image_height, image_width = self.current_image.shape[:2]
        obj_x, obj_y = self.object_position
        
        # Calculate error from center
        center_x = image_width // 2
        center_y = image_height // 2
        
        error_x = obj_x - center_x
        error_y = obj_y - center_y
        
        # Convert pixel error to neck angles (simple proportional control)
        pan_adjustment = -error_x * 0.001  # Negative for correct direction
        tilt_adjustment = error_y * 0.001
        
        new_pan = np.clip(self.neck_pan + pan_adjustment, *self.neck_pan_range)
        new_tilt = np.clip(self.neck_tilt + tilt_adjustment, *self.neck_tilt_range)
        
        self.move_neck_to_position(new_pan, new_tilt)
        
        # If object is centered, start approaching
        if abs(error_x) < 50 and abs(error_y) < 50:  # Within 50 pixels of center
            self.robot_mode = "approaching"
            self.get_logger().info("🎯 Object centered, starting approach")
    
    def approach_behavior(self):
        """Move robot towards the detected object"""
        if self.object_position is None:
            self.robot_mode = "searching"
            return
        
        # Set movement velocities for approach
        self.vx = self.linear_speed * 0.5  # Slow approach
        self.vy = 0.0
        self.vyaw = 0.0
        
        # Add some object tracking while approaching
        if self.current_image is not None:
            image_width = self.current_image.shape[1]
            obj_x = self.object_position[0]
            center_x = image_width // 2
            error_x = obj_x - center_x
            
            # Add slight rotation to keep object centered
            if abs(error_x) > 30:
                self.vyaw = -error_x * 0.001
        
        # Send movement command to real robot
        self.send_movement_command()
        
        # Also publish ROS message for compatibility
        cmd_vel = Twist()
        cmd_vel.linear.x = self.vx
        cmd_vel.angular.z = self.vyaw
        self.cmd_vel_pub.publish(cmd_vel)
        
        # TODO: Add distance estimation to stop at appropriate distance
        # For now, approach for a fixed time then stop
        if not hasattr(self, 'approach_start_time'):
            self.approach_start_time = time.time()
        elif time.time() - self.approach_start_time > 5.0:  # Approach for 5 seconds
            self.stop_robot()
            self.robot_mode = "idle"
            self.get_logger().info("✅ Approach complete")
            del self.approach_start_time
    
    def idle_behavior(self):
        """Idle state - keep robot still"""
        self.stop_robot()
    
    def neck_control_loop(self):
        """High-frequency neck control updates"""
        # Smooth neck movement towards target position
        # This runs at higher frequency for smooth motion
        pass
    
    def move_neck_to_position(self, pan, tilt):
        """Move neck to specified pan/tilt angles"""
        # Clamp angles to safe ranges
        pan = np.clip(pan, *self.neck_pan_range)
        tilt = np.clip(tilt, *self.neck_tilt_range)
        
        self.neck_pan = pan
        self.neck_tilt = tilt
        
        # Use robot SDK for head control if available
        if self.robot_client:
            try:
                # Convert to robot coordinate system if needed
                result = self.robot_client.RotateHead(tilt, pan)
                if result == 0:
                    self.get_logger().info(f"🎥 Head moved to pan={pan:.2f}, tilt={tilt:.2f}")
                else:
                    self.get_logger().warn(f"⚠️  Head rotation returned code: {result}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to rotate head: {e}")
        
        # Also publish ROS message for compatibility
        neck_cmd = Float64MultiArray()
        neck_cmd.data = [pan, tilt]
        self.neck_control_pub.publish(neck_cmd)
    
    def send_movement_command(self):
        """Send movement command to robot using Booster SDK"""
        if self.robot_client:
            try:
                result = self.robot_client.Move(self.vx, self.vy, self.vyaw)
                if result != 0:
                    self.get_logger().warn(f"⚠️  Movement command returned code: {result}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to send movement command: {e}")
        else:
            # Simulate movement for testing
            self.get_logger().info(f"🏃 Simulated movement: vx={self.vx:.2f}, vy={self.vy:.2f}, vyaw={self.vyaw:.2f}")
    
    def stop_robot(self):
        """Stop all robot movement"""
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        
        # Send to real robot if available
        self.send_movement_command()
        
        # Also publish ROS message for compatibility
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info("🛑 Robot stopped")
    
    def publish_status(self):
        """Publish current robot status"""
        status = {
            "mode": self.robot_mode,
            "target_object": self.target_object,
            "object_found": self.object_position is not None,
            "neck_position": [self.neck_pan, self.neck_tilt],
            "timestamp": time.time()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = GeminiRobotController()
        
        controller.get_logger().info("🚀 Gemini Robot Controller started!")
        controller.get_logger().info("📝 Usage:")
        controller.get_logger().info("  • Send object prompts: ros2 topic pub /robot/find_object std_msgs/String \"data: 'find a red apple'\"")
        controller.get_logger().info("  • Robot control: ros2 topic pub /robot/control std_msgs/String \"data: 'stop'\"")
        controller.get_logger().info("  • Monitor status: ros2 topic echo /robot/status")
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Gemini Robot Controller stopped by user")
    except Exception as e:
        controller.get_logger().error(f"❌ Controller error: {e}")
    finally:
        try:
            if 'controller' in locals():
                controller.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()