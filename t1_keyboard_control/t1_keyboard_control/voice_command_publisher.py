#!/usr/bin/env python3
"""
Voice Command Publisher for T1 Robot
Converts voice commands to robot actions via ROS2 topics
Supports both direct movement and AI-powered object detection/tracking
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import threading
import time
import json
import re
from typing import Dict, List, Optional

class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')
        
        # Declare parameters
        self.declare_parameter('microphone_device_index', -1)  # -1 = default
        self.declare_parameter('listen_timeout', 5.0)
        self.declare_parameter('phrase_timeout', 1.0)
        self.declare_parameter('energy_threshold', 4000)
        
        # Get parameters
        self.mic_device_index = self.get_parameter('microphone_device_index').value
        self.listen_timeout = self.get_parameter('listen_timeout').value
        self.phrase_timeout = self.get_parameter('phrase_timeout').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.find_object_pub = self.create_publisher(String, '/robot/find_object', 10)
        self.robot_control_pub = self.create_publisher(String, '/robot/control', 10)
        self.voice_status_pub = self.create_publisher(String, '/voice/status', 10)
        
        # Speech recognition setup
        self.recognizer = sr.Recognizer()
        self.microphone = None
        self.is_listening = False
        self.listen_thread = None
        
        # Movement parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.movement_duration = 1.0  # seconds for each movement command
        
        # Voice command patterns
        self.setup_command_patterns()
        
        # Initialize microphone
        self.setup_microphone()
        
        self.get_logger().info("🎤 Voice Command Publisher initialized")
        self.print_usage_instructions()
        
    def setup_command_patterns(self):
        """Define voice command patterns and their mappings"""
        self.movement_commands = {
            # Basic movement
            r'\b(move|go)\s+(forward|ahead|front)\b': {'action': 'move_forward'},
            r'\b(move|go)\s+(backward|back|backwards)\b': {'action': 'move_backward'},
            r'\b(move|go)\s+(left)\b': {'action': 'move_left'},
            r'\b(move|go)\s+(right)\b': {'action': 'move_right'},
            r'\b(turn|rotate)\s+(left)\b': {'action': 'turn_left'},
            r'\b(turn|rotate)\s+(right)\b': {'action': 'turn_right'},
            r'\b(stop|halt|freeze)\b': {'action': 'stop'},
            
            # Simple directional commands
            r'\b(forward|ahead)\b': {'action': 'move_forward'},
            r'\b(backward|back|reverse)\b': {'action': 'move_backward'},
            r'\b(left)\b': {'action': 'move_left'},
            r'\b(right)\b': {'action': 'move_right'},
        }
        
        self.object_detection_commands = {
            # Find objects
            r'\b(find|look for|search for|locate)\s+(.+?)(?:\s+(please|now))?$': {'action': 'find_object'},
            r'\b(where is|show me)\s+(.+?)(?:\s+(please|now))?$': {'action': 'find_object'},
            r'\b(detect|identify)\s+(.+?)(?:\s+(please|now))?$': {'action': 'find_object'},
            
            # Track/follow objects
            r'\b(follow|track|chase)\s+(.+?)(?:\s+(please|now))?$': {'action': 'track_object'},
            r'\b(go to|approach|move to)\s+(.+?)(?:\s+(please|now))?$': {'action': 'approach_object'},
        }
        
        self.robot_control_commands = {
            # Robot control
            r'\b(stop searching|stop looking|cancel search)\b': {'action': 'stop_search'},
            r'\b(start searching|begin search|search mode)\b': {'action': 'start_search'},
            r'\b(robot stop|emergency stop|full stop)\b': {'action': 'emergency_stop'},
        }
        
        # Voice control commands
        self.voice_control_commands = {
            r'\b(start listening|voice on|listen)\b': {'action': 'start_listening'},
            r'\b(stop listening|voice off|quiet)\b': {'action': 'stop_listening'},
            r'\b(voice status|listening status)\b': {'action': 'voice_status'},
        }
    
    def setup_microphone(self):
        """Initialize microphone for speech recognition"""
        try:
            if self.mic_device_index == -1:
                self.microphone = sr.Microphone()
                self.get_logger().info("🎤 Using default microphone")
            else:
                self.microphone = sr.Microphone(device_index=self.mic_device_index)
                self.get_logger().info(f"🎤 Using microphone device {self.mic_device_index}")
                
            # Calibrate for ambient noise
            with self.microphone as source:
                self.get_logger().info("🔧 Calibrating microphone for ambient noise...")
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                
            self.recognizer.energy_threshold = self.energy_threshold
            self.get_logger().info(f"✅ Microphone ready (energy threshold: {self.energy_threshold})")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to setup microphone: {e}")
            self.get_logger().info("📋 Available audio devices:")
            for i, name in enumerate(sr.Microphone.list_microphone_names()):
                self.get_logger().info(f"  Device {i}: {name}")
    
    def print_usage_instructions(self):
        """Print voice command usage instructions"""
        instructions = """
        ==========================================
        🎤 VOICE COMMAND USAGE
        ==========================================
        
        Movement Commands:
        ------------------
        • "Move forward" / "Go ahead" / "Forward"
        • "Move backward" / "Go back" / "Backward"  
        • "Move left" / "Left"
        • "Move right" / "Right"
        • "Turn left" / "Rotate left"
        • "Turn right" / "Rotate right"
        • "Stop" / "Halt" / "Freeze"
        
        AI Object Detection:
        --------------------
        • "Find a red ball"
        • "Look for a person"
        • "Search for a chair"
        • "Where is the bottle?"
        • "Show me a book"
        • "Detect a face"
        
        Object Tracking:
        ----------------
        • "Follow the person"
        • "Track the red object"
        • "Go to the chair"
        • "Approach the table"
        
        Robot Control:
        --------------
        • "Stop searching"
        • "Start searching"  
        • "Robot stop"
        • "Emergency stop"
        
        Voice Control:
        --------------
        • "Start listening" - Begin voice recognition
        • "Stop listening" - Stop voice recognition
        • "Voice status" - Check listening status
        
        📝 Usage:
        1. Say "Start listening" to activate
        2. Speak your commands clearly
        3. Wait for confirmation
        4. Say "Stop listening" to deactivate
        
        🎯 Topics Published:
        • /cmd_vel - Direct movement commands
        • /robot/find_object - AI object detection
        • /robot/control - Robot state control
        • /voice/status - Voice system status
        ==========================================
        """
        print(instructions)
    
    def start_voice_recognition(self):
        """Start continuous voice recognition in a separate thread"""
        if self.is_listening:
            self.get_logger().warn("⚠️  Voice recognition already active")
            return
            
        if not self.microphone:
            self.get_logger().error("❌ Microphone not available")
            return
            
        self.is_listening = True
        self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listen_thread.start()
        
        self.get_logger().info("🎤 Voice recognition started - listening for commands...")
        self.publish_voice_status("listening", "Voice recognition active")
    
    def stop_voice_recognition(self):
        """Stop voice recognition"""
        self.is_listening = False
        if self.listen_thread:
            self.listen_thread.join(timeout=2.0)
        
        self.get_logger().info("🔇 Voice recognition stopped")
        self.publish_voice_status("stopped", "Voice recognition inactive")
    
    def _listen_loop(self):
        """Main listening loop for voice recognition"""
        while self.is_listening and rclpy.ok():
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    self.get_logger().debug("🎧 Listening for audio...")
                    audio = self.recognizer.listen(
                        source, 
                        timeout=self.listen_timeout,
                        phrase_time_limit=self.phrase_timeout
                    )
                
                # Recognize speech
                self.get_logger().debug("🔍 Processing audio...")
                text = self.recognizer.recognize_google(audio).lower()
                
                if text:
                    self.get_logger().info(f"🎤 Heard: '{text}'")
                    self.process_voice_command(text)
                    
            except sr.WaitTimeoutError:
                # Normal timeout, continue listening
                continue
            except sr.UnknownValueError:
                self.get_logger().debug("🤷 Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"❌ Speech recognition service error: {e}")
                time.sleep(1)  # Wait before retrying
            except Exception as e:
                self.get_logger().error(f"❌ Voice recognition error: {e}")
                time.sleep(1)
    
    def process_voice_command(self, text: str):
        """Process recognized voice command"""
        text = text.strip()
        if not text:
            return
            
        self.get_logger().info(f"🎯 Processing command: '{text}'")
        
        # Check voice control commands first
        if self._check_voice_control_commands(text):
            return
            
        # Check movement commands
        if self._check_movement_commands(text):
            return
            
        # Check object detection commands
        if self._check_object_detection_commands(text):
            return
            
        # Check robot control commands
        if self._check_robot_control_commands(text):
            return
            
        # No command matched
        self.get_logger().warn(f"❓ Unknown command: '{text}'")
        self.publish_voice_status("unknown_command", f"Command not recognized: {text}")
    
    def _check_voice_control_commands(self, text: str) -> bool:
        """Check and execute voice control commands"""
        for pattern, command in self.voice_control_commands.items():
            if re.search(pattern, text):
                action = command['action']
                
                if action == 'start_listening':
                    if not self.is_listening:
                        self.start_voice_recognition()
                    else:
                        self.get_logger().info("🎤 Already listening")
                        
                elif action == 'stop_listening':
                    self.stop_voice_recognition()
                    
                elif action == 'voice_status':
                    status = "active" if self.is_listening else "inactive"
                    self.get_logger().info(f"🎤 Voice recognition is {status}")
                    self.publish_voice_status("status_check", f"Voice recognition: {status}")
                    
                return True
        return False
    
    def _check_movement_commands(self, text: str) -> bool:
        """Check and execute movement commands"""
        for pattern, command in self.movement_commands.items():
            if re.search(pattern, text):
                self._execute_movement_command(command['action'])
                return True
        return False
    
    def _check_object_detection_commands(self, text: str) -> bool:
        """Check and execute object detection commands"""
        for pattern, command in self.object_detection_commands.items():
            match = re.search(pattern, text)
            if match:
                action = command['action']
                # Extract object name from the second capture group
                object_name = match.group(2).strip() if match.group(2) else "object"
                self._execute_object_command(action, object_name)
                return True
        return False
    
    def _check_robot_control_commands(self, text: str) -> bool:
        """Check and execute robot control commands"""
        for pattern, command in self.robot_control_commands.items():
            if re.search(pattern, text):
                self._execute_robot_control_command(command['action'])
                return True
        return False
    
    def _execute_movement_command(self, action: str):
        """Execute movement command via cmd_vel topic"""
        cmd_vel = Twist()
        
        if action == 'move_forward':
            cmd_vel.linear.x = self.linear_speed
            self.get_logger().info("➡️  Moving forward")
            
        elif action == 'move_backward':
            cmd_vel.linear.x = -self.linear_speed
            self.get_logger().info("⬅️  Moving backward")
            
        elif action == 'move_left':
            cmd_vel.linear.y = self.linear_speed
            self.get_logger().info("⬆️  Moving left")
            
        elif action == 'move_right':
            cmd_vel.linear.y = -self.linear_speed
            self.get_logger().info("⬇️  Moving right")
            
        elif action == 'turn_left':
            cmd_vel.angular.z = self.angular_speed
            self.get_logger().info("↺ Turning left")
            
        elif action == 'turn_right':
            cmd_vel.angular.z = -self.angular_speed
            self.get_logger().info("↻ Turning right")
            
        elif action == 'stop':
            # cmd_vel is already zero
            self.get_logger().info("🛑 Stopping")
        
        # Publish movement command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # For non-stop commands, schedule a stop after movement duration
        if action != 'stop':
            threading.Timer(self.movement_duration, self._auto_stop).start()
    
    def _auto_stop(self):
        """Automatically stop movement after duration"""
        stop_cmd = Twist()  # All velocities are zero
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info("⏹️  Auto-stopping movement")
    
    def _execute_object_command(self, action: str, object_name: str):
        """Execute object detection/tracking command"""
        if action in ['find_object', 'track_object', 'approach_object']:
            # Send to Gemini AI for object detection
            command_text = f"{action.replace('_', ' ')} {object_name}"
            msg = String()
            msg.data = command_text
            self.find_object_pub.publish(msg)
            
            self.get_logger().info(f"🎯 AI Command: '{command_text}'")
            self.publish_voice_status("object_command", f"Searching for: {object_name}")
        
    def _execute_robot_control_command(self, action: str):
        """Execute robot control command"""
        msg = String()
        
        if action == 'stop_search':
            msg.data = "stop"
            self.get_logger().info("🛑 Stopping search")
            
        elif action == 'start_search':
            msg.data = "search"
            self.get_logger().info("🔍 Starting search mode")
            
        elif action == 'emergency_stop':
            # Send both stop commands for safety
            msg.data = "stop"
            self.robot_control_pub.publish(msg)
            
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            self.get_logger().info("🚨 EMERGENCY STOP")
            self.publish_voice_status("emergency_stop", "Emergency stop activated")
            return
        
        self.robot_control_pub.publish(msg)
    
    def publish_voice_status(self, status_type: str, message: str):
        """Publish voice system status"""
        status = {
            "type": status_type,
            "message": message,
            "timestamp": time.time(),
            "is_listening": self.is_listening
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.voice_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Check if speech_recognition is available
        try:
            import speech_recognition as sr
        except ImportError:
            print("❌ speech_recognition not installed. Run: pip install SpeechRecognition pyaudio")
            return
            
        voice_publisher = VoiceCommandPublisher()
        
        voice_publisher.get_logger().info("🚀 Voice Command Publisher started!")
        voice_publisher.get_logger().info("💡 Say 'Start listening' to activate voice control")
        
        # Auto-start listening
        voice_publisher.start_voice_recognition()
        
        rclpy.spin(voice_publisher)
        
    except KeyboardInterrupt:
        voice_publisher.get_logger().info("🛑 Voice Command Publisher stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'voice_publisher' in locals():
                voice_publisher.stop_voice_recognition()
                voice_publisher.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()