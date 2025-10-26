#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64
import json
import requests
import time
import threading
from collections import deque

# Import Booster SDK
try:
    from booster_robotics_sdk_python import (
        B1LocoClient,
        ChannelFactory,
        RobotMode,
        GetModeResponse
    )
except ImportError as e:
    print(f"Warning: Booster SDK not found. Running in simulation mode.")
    print(f"To install SDK: cd booster_robotics_sdk/build && sudo cmake .. -DBUILD_PYTHON_BINDING=on && sudo make && sudo make install")
    B1LocoClient = None


class GeminiPersonFollower(Node):
    """ROS2 node for person following using Google Gemini Vision API"""
    
    def __init__(self):
        super().__init__('gemini_person_follower')
        
        # Declare parameters
        self.declare_parameter('api_key', 'AIzaSyDnpcOTRkPfB0zagID3uhieCUTcpBG8jmg')
        self.declare_parameter('network_interface', 'enxa0cec861c322')
        self.declare_parameter('use_booster_sdk', True)
        self.declare_parameter('camera_topic', '/zed_real_robot/image_raw')
        self.declare_parameter('detection_interval', 1.0)  # seconds between detection
        self.declare_parameter('movement_speed', 0.2)  # m/s forward speed (more conservative)
        self.declare_parameter('rotation_speed', 0.3)  # rad/s rotation speed (more conservative)
        self.declare_parameter('min_person_size', 0.15)  # minimum person size in frame
        self.declare_parameter('center_threshold', 0.1)  # threshold for centering
        
        # Get parameters
        self.api_key = self.get_parameter('api_key').value
        self.network_interface = self.get_parameter('network_interface').value
        self.use_booster_sdk = self.get_parameter('use_booster_sdk').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.detection_interval = self.get_parameter('detection_interval').value
        self.movement_speed = self.get_parameter('movement_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.min_person_size = self.get_parameter('min_person_size').value
        self.center_threshold = self.get_parameter('center_threshold').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize Booster SDK if available
        self.client = None
        if self.use_booster_sdk and B1LocoClient is not None:
            try:
                self.get_logger().info(f'Initializing Booster SDK on interface {self.network_interface}')
                ChannelFactory.Instance().Init(0, self.network_interface)
                self.client = B1LocoClient()
                self.client.Init()
                self.get_logger().info('Booster SDK initialized successfully')
                
                # Set robot to WALK mode - wait a moment after SDK init
                time.sleep(1.0)
                self.set_robot_walk_mode()
                
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Booster SDK: {e}')
                self.get_logger().info('Falling back to Twist message mode')
                self.client = None
        
        # Create publishers for robot control
        if not self.client:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # State variables
        self.last_image = None
        self.last_detection_time = 0
        self.person_detected = False
        self.person_position = None  # (x, y, width, height) normalized to [0,1]
        self.distance_estimate = 'unknown'  # 'close', 'medium', 'far', 'unknown'
        self.tracking_state = 'searching'  # 'searching', 'tracking', 'approaching', 'unknown_distance_sequence'
        
        # Unknown distance movement sequence state
        self.unknown_distance_step = 0  # 0: not started, 1: moving forward 1m, 2: moving toward person 0.5m
        self.sequence_start_time = 0
        self.movement_start_time = 0
        
        # Threading for detection
        self.detection_thread = None
        self.detection_lock = threading.Lock()
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Gemini Person Follower initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        
        # Check robot status after initialization
        if self.client:
            self.check_robot_status()
        
    def set_robot_walk_mode(self):
        """Set robot to WALK mode (required for movement)"""
        if not self.client:
            return
            
        mode_names = {
            RobotMode.kDamping: "DAMP (Safe Mode)",
            RobotMode.kPrepare: "PREP (Standing)", 
            RobotMode.kWalking: "WALK (Ready to Move)"
        }
        
        try:
            # Try multiple times to get mode (sometimes takes a moment after SDK init)
            current_mode = None
            for attempt in range(3):
                response = GetModeResponse()
                result = self.client.GetMode(response)
                
                if result == 0:
                    current_mode = response.mode
                    self.get_logger().info(f'Current robot mode: {mode_names.get(current_mode, f"UNKNOWN({current_mode})")}')
                    break
                else:
                    self.get_logger().warn(f'Failed to get robot mode (attempt {attempt+1}/3), error code: {result}')
                    time.sleep(1.0)
            
            if current_mode is None:
                self.get_logger().error('Could not determine robot mode after 3 attempts')
                return
                
            # If not in WALK mode, switch to it
            if current_mode != RobotMode.kWalking:
                self.get_logger().info('Robot needs to be switched to WALK mode')
                
                # First ensure we're not in DAMP mode - go to PREP
                if current_mode != RobotMode.kPrepare:
                    self.get_logger().info('Switching to PREP mode first...')
                    result = self.client.ChangeMode(RobotMode.kPrepare)
                    if result != 0:
                        self.get_logger().error(f'Failed to switch to PREP mode, error code: {result}')
                        return
                    
                    # Wait for mode transition
                    time.sleep(4.0)
                
                # Now switch to WALK mode
                self.get_logger().info('Now switching to WALK mode...')
                result = self.client.ChangeMode(RobotMode.kWalking)
                if result == 0:
                    self.get_logger().info('Successfully switched to WALK mode')
                    time.sleep(2.0)  # Wait for mode to be fully active
                else:
                    self.get_logger().error(f'Failed to switch to WALK mode, error code: {result}')
            else:
                self.get_logger().info('Robot already in WALK mode')
                
        except Exception as e:
            self.get_logger().error(f'Failed to set robot mode: {e}')
    
    def check_robot_status(self):
        """Check and display current robot mode"""
        if not self.client:
            return True  # No SDK, assume OK
            
        try:
            # Wait a moment before checking (SDK might need time)
            time.sleep(1.0)
            
            response = GetModeResponse()
            result = self.client.GetMode(response)
            if result == 0:
                mode = response.mode
                mode_names = {
                    RobotMode.kDamping: "DAMP (Safe Mode)",
                    RobotMode.kPrepare: "PREP (Standing)",
                    RobotMode.kWalking: "WALK (Ready to Move)"
                }
                self.get_logger().info(f'Robot status check - Mode: {mode_names.get(mode, f"UNKNOWN({mode})")}')
                
                if mode != RobotMode.kWalking:
                    self.get_logger().warn('Robot is not in WALK mode. Attempting to switch...')
                    self.set_robot_walk_mode()
                    return False
                return True
            else:
                self.get_logger().warn(f'Could not get robot mode, error code: {result}. Robot may still work.')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to check robot status: {e}')
            return False
    
    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            # Handle NV12 format if necessary
            if msg.encoding == 'nv12':
                # NV12 to BGR conversion
                yuv_data = np.frombuffer(msg.data, dtype=np.uint8)
                height = msg.height
                width = msg.width
                
                # Reshape for NV12 format
                yuv = yuv_data.reshape(height * 3 // 2, width)
                bgr_image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                
            else:
                # Use cv_bridge for standard formats
                bgr_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Store latest image
            self.last_image = bgr_image
            
            # Check if it's time for detection
            current_time = time.time()
            if current_time - self.last_detection_time > self.detection_interval:
                self.last_detection_time = current_time
                
                # Run detection in separate thread to avoid blocking
                if self.detection_thread is None or not self.detection_thread.is_alive():
                    self.detection_thread = threading.Thread(
                        target=self.detect_person,
                        args=(bgr_image.copy(),)
                    )
                    self.detection_thread.start()
                    
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')
    
    def detect_person(self, image):
        """Use Gemini Vision API to detect person in image"""
        try:
            # Resize image for faster processing
            max_size = 640
            height, width = image.shape[:2]
            if width > max_size:
                scale = max_size / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Prepare Gemini API request
            url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key={self.api_key}"
            
            prompt = """Detect person in image. Respond JSON only:
            {"person_detected": true, "bounding_box": {"x": 0.5, "y": 0.5, "width": 0.3, "height": 0.7}, "position_description": "center"}
            
            Use coordinates 0-1. If no person: {"person_detected": false}"""
            
            payload = {
                "contents": [{
                    "parts": [
                        {"text": prompt},
                        {
                            "inline_data": {
                                "mime_type": "image/jpeg",
                                "data": image_base64
                            }
                        }
                    ]
                }],
                "generationConfig": {
                    "temperature": 0.1,
                    "topK": 1,
                    "topP": 0.8,
                    "maxOutputTokens": 2048
                }
            }
            
            # Make API request
            response = requests.post(url, json=payload)
            
            if response.status_code == 200:
                result = response.json()
                
                # Debug: Log the response structure
                self.get_logger().debug(f'Gemini API response structure: {result}')
                
                # Extract text from response
                if 'candidates' in result and len(result['candidates']) > 0:
                    candidate = result['candidates'][0]
                    self.get_logger().debug(f'Candidate structure: {candidate}')
                    
                    # Handle different response structures
                    text = None
                    if 'content' in candidate:
                        content = candidate['content']
                        if 'parts' in content and len(content['parts']) > 0:
                            text = content['parts'][0]['text']
                        elif 'text' in content:
                            text = content['text']
                    elif 'text' in candidate:
                        text = candidate['text']
                    
                    if not text:
                        # Check if response was truncated due to MAX_TOKENS
                        if candidate.get('finishReason') == 'MAX_TOKENS':
                            self.get_logger().error('Gemini response truncated (MAX_TOKENS). Using fallback detection.')
                            # Fallback: assume person detected if we got this far (since we have an image)
                            with self.detection_lock:
                                self.person_detected = True
                                self.person_position = (0.5, 0.5, 0.3, 0.7)  # Default center position
                                self.tracking_state = 'tracking'
                                self.get_logger().info('Fallback: Person detected (center)')
                        else:
                            self.get_logger().error(f'Could not extract text from response: {candidate}')
                        return
                    
                    # Parse JSON from response
                    try:
                        # Extract JSON from the text (handle markdown code blocks)
                        json_start = text.find('{')
                        json_end = text.rfind('}') + 1
                        if json_start >= 0 and json_end > json_start:
                            json_str = text[json_start:json_end]
                            detection_result = json.loads(json_str)
                            
                            # Update detection state
                            with self.detection_lock:
                                self.person_detected = detection_result.get('person_detected', False)
                                
                                if self.person_detected:
                                    bbox = detection_result.get('bounding_box', {})
                                    self.person_position = (
                                        bbox.get('x', 0.5),
                                        bbox.get('y', 0.5),
                                        bbox.get('width', 0),
                                        bbox.get('height', 0)
                                    )
                                    
                                    # Log detection
                                    pos_desc = detection_result.get('position_description', 'unknown')
                                    dist = detection_result.get('distance_estimate', 'unknown')
                                    self.distance_estimate = dist
                                    self.get_logger().info(f'Person detected: {pos_desc}, distance: {dist}')
                                    
                                    # Handle unknown distance with special sequence
                                    if dist == 'unknown' and self.tracking_state != 'unknown_distance_sequence':
                                        self.tracking_state = 'unknown_distance_sequence'
                                        self.unknown_distance_step = 1
                                        self.sequence_start_time = time.time()
                                        self.movement_start_time = time.time()
                                        self.get_logger().info('Starting unknown distance sequence: moving forward 1m')
                                    
                                    # Update tracking state
                                    if bbox.get('height', 0) > self.min_person_size:
                                        self.tracking_state = 'tracking'
                                    else:
                                        self.tracking_state = 'approaching'
                                else:
                                    self.person_position = None
                                    self.tracking_state = 'searching'
                                    self.get_logger().info('No person detected')
                                    
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f'Failed to parse JSON from Gemini response: {e}')
                        self.get_logger().debug(f'Response text: {text}')
                        
            else:
                self.get_logger().error(f'Gemini API request failed: {response.status_code}')
                self.get_logger().debug(f'Response: {response.text}')
                
        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
    
    def control_loop(self):
        """Main control loop for robot movement"""
        with self.detection_lock:
            person_detected = self.person_detected
            person_position = self.person_position
            state = self.tracking_state
        
        vx = 0.0  # Forward/backward velocity
        vy = 0.0  # Left/right velocity
        vyaw = 0.0  # Rotational velocity
        
        if person_detected and person_position:
            x, y, width, height = person_position
            
            # Calculate centering error
            x_error = x - 0.5  # Error from center (-0.5 to 0.5)
            
            # Rotation control to center person
            if abs(x_error) > self.center_threshold:
                # Rotate to center the person
                vyaw = -x_error * self.rotation_speed * 2.0  # Negative for correct direction
                self.get_logger().debug(f'Centering: x_error={x_error:.2f}, vyaw={vyaw:.2f}')
            
            # Forward movement control
            if state == 'tracking':
                # Person is visible and reasonably sized
                if abs(x_error) < self.center_threshold * 2:
                    # Move forward if person is centered
                    vx = self.movement_speed
                    self.get_logger().debug(f'Moving forward: vx={vx:.2f}')
            elif state == 'approaching':
                # Person is too far (small in frame)
                if abs(x_error) < self.center_threshold * 2:
                    vx = self.movement_speed * 1.5  # Move faster when far
                    self.get_logger().debug(f'Approaching: vx={vx:.2f}')
        
        elif state == 'unknown_distance_sequence':
            # Execute the unknown distance movement sequence
            current_time = time.time()
            elapsed_time = current_time - self.movement_start_time
            
            if self.unknown_distance_step == 1:
                # Step 1: Move forward 1m (estimate 5 seconds at 0.2 m/s)
                vx = 0.2  # Move forward at safer speed
                if elapsed_time >= 5.0:  # Approximate time to travel 1m at 0.2 m/s
                    self.unknown_distance_step = 2
                    self.movement_start_time = current_time
                    self.get_logger().info('Step 1 complete. Now moving toward person 0.5m')
                    
            elif self.unknown_distance_step == 2:
                # Step 2: Move toward person 0.5m
                if person_detected and person_position:
                    x, _, _, _ = person_position
                    x_error = x - 0.5  # Error from center
                    
                    # Move forward and toward the person
                    vx = 0.15  # Slower forward movement
                    vy = -x_error * 0.2  # More gentle lateral movement toward person
                    
                    if elapsed_time >= 3.5:  # Approximate time to travel 0.5m at slower speed
                        # Sequence complete, return to normal tracking
                        self.tracking_state = 'tracking'
                        self.unknown_distance_step = 0
                        self.get_logger().info('Unknown distance sequence complete, returning to tracking')
                else:
                    # Person lost during sequence, return to searching
                    self.tracking_state = 'searching'
                    self.unknown_distance_step = 0
                    self.get_logger().info('Person lost during sequence, returning to search')
                    
        elif state == 'searching':
            # No person detected - rotate to search
            vyaw = self.rotation_speed * 0.5  # Slow rotation to search
            self.get_logger().debug('Searching for person...')
        
        # Send movement command (only if we have non-zero movement or using SDK)
        if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vyaw) > 0.01) or not self.client:
            self.send_movement_command(vx, vy, vyaw)
    
    def send_movement_command(self, vx, vy, vyaw):
        """Send movement command to robot"""
        try:
            if self.client:
                # Use Booster SDK
                result = self.client.Move(vx, vy, vyaw)
                if result != 0:
                    # Error code 100 often means robot is not in correct mode
                    if result == 100:
                        if not hasattr(self, '_last_mode_error_time') or (time.time() - self._last_mode_error_time) > 5.0:
                            self.get_logger().error('Movement failed: Robot not in WALK mode (error 100). Attempting to fix...')
                            self._last_mode_error_time = time.time()
                            # Try to fix the mode in background
                            threading.Thread(target=self.set_robot_walk_mode).start()
                    else:
                        self.get_logger().warn(f'Movement command returned code: {result}')
                    
                    self.get_logger().debug(f'Failed command: vx={vx:.3f}, vy={vy:.3f}, vyaw={vyaw:.3f}')
                else:
                    # Successful movement - log occasionally for debugging
                    if not hasattr(self, '_last_move_log') or (time.time() - self._last_move_log) > 2.0:
                        self.get_logger().debug(f'Movement: vx={vx:.3f}, vy={vy:.3f}, vyaw={vyaw:.3f}')
                        self._last_move_log = time.time()
            else:
                # Use ROS Twist message
                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.angular.z = vyaw
                self.cmd_vel_pub.publish(twist)
                self.get_logger().debug(f'Published twist: vx={vx:.3f}, vy={vy:.3f}, vyaw={vyaw:.3f}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send movement command: {e}')
    
    def stop_robot(self):
        """Stop all robot movement"""
        self.send_movement_command(0.0, 0.0, 0.0)
        self.get_logger().info('Robot stopped')
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.stop_robot()
        
        # Switch robot back to safe mode if using SDK
        if self.client:
            try:
                self.client.ChangeMode(RobotMode.kDamping)
                self.get_logger().info('Robot switched to DAMP mode')
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = GeminiPersonFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()