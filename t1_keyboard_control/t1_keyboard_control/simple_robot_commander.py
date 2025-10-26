#!/usr/bin/env python3
"""
Simple Robot Commander
Executes natural language movement commands like:
- "move ahead 2 meters"
- "turn right 90 degrees"  
- "move forward 1.5m then turn left"
- "go back 3 meters and stop"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re
import time
import threading
import math
from typing import List, Dict, Optional, Tuple

class SimpleRobotCommander(Node):
    def __init__(self):
        super().__init__('simple_robot_commander')
        
        # Declare parameters
        self.declare_parameter('default_linear_speed', 0.3)  # m/s
        self.declare_parameter('default_angular_speed', 0.5)  # rad/s
        self.declare_parameter('command_topic', '/robot/simple_command')
        self.declare_parameter('status_topic', '/robot/commander_status')
        
        # Get parameters
        self.default_linear_speed = self.get_parameter('default_linear_speed').value
        self.default_angular_speed = self.get_parameter('default_angular_speed').value
        self.command_topic = self.get_parameter('command_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, self.command_topic, self.command_callback, 10
        )
        
        # Command execution state
        self.is_executing = False
        self.current_command = None
        self.execution_thread = None
        self.stop_execution = False
        
        # Movement patterns
        self.setup_command_patterns()
        
        self.get_logger().info("🤖 Simple Robot Commander initialized")
        self.print_usage_instructions()
        
    def setup_command_patterns(self):
        """Define movement command patterns"""
        
        # Distance patterns (meters, m, centimeters, cm, feet, ft)
        self.distance_patterns = [
            r'(\d+(?:\.\d+)?)\s*(?:meters?|m)\b',
            r'(\d+(?:\.\d+)?)\s*(?:centimeters?|cm)\b',
            r'(\d+(?:\.\d+)?)\s*(?:feet?|ft)\b',
            r'(\d+(?:\.\d+)?)\b(?!\s*(?:degrees?|deg))'  # Just number (assume meters)
        ]
        
        # Angle patterns (degrees, deg, radians, rad)
        self.angle_patterns = [
            r'(\d+(?:\.\d+)?)\s*(?:degrees?|deg|°)\b',
            r'(\d+(?:\.\d+)?)\s*(?:radians?|rad)\b'
        ]
        
        # Movement direction patterns
        self.movement_patterns = {
            # Forward movement
            r'\b(?:move|go)\s+(?:forward|ahead|front|straight)': 'forward',
            r'\b(?:forward|ahead|front)': 'forward',
            
            # Backward movement  
            r'\b(?:move|go)\s+(?:backward|back|backwards|reverse)': 'backward',
            r'\b(?:backward|back|reverse)': 'backward',
            
            # Left movement
            r'\b(?:move|go)\s+(?:left)': 'left',
            r'\b(?:left)(?!\s+turn)': 'left',
            
            # Right movement
            r'\b(?:move|go)\s+(?:right)': 'right', 
            r'\b(?:right)(?!\s+turn)': 'right',
            
            # Turn left
            r'\b(?:turn|rotate)\s+(?:left|counterclockwise|ccw)': 'turn_left',
            r'\b(?:left\s+turn)': 'turn_left',
            
            # Turn right
            r'\b(?:turn|rotate)\s+(?:right|clockwise|cw)': 'turn_right',
            r'\b(?:right\s+turn)': 'turn_right',
            
            # Stop
            r'\b(?:stop|halt|freeze|brake)\b': 'stop'
        }
    
    def print_usage_instructions(self):
        """Print usage instructions"""
        instructions = """
        ==========================================
        🤖 SIMPLE ROBOT COMMANDER
        ==========================================
        
        Natural Language Commands:
        --------------------------
        
        📐 MOVEMENT WITH DISTANCE:
        • "move forward 2 meters"
        • "go ahead 1.5m"  
        • "move back 3 meters"
        • "go left 0.5 meters"
        • "move right 2.3m"
        
        🔄 ROTATION WITH ANGLES:
        • "turn right 90 degrees"
        • "turn left 45 deg"
        • "rotate right 180°"
        • "turn left 1.57 radians"
        
        🔗 COMBINED COMMANDS:
        • "move forward 2m then turn right 90 degrees"
        • "go ahead 1 meter and turn left"
        • "move back 0.5m then stop"
        • "turn right 45 degrees then move forward 1m"
        
        ⏹️ CONTROL COMMANDS:
        • "stop" / "halt" / "freeze"
        
        📡 ROS Usage:
        --------------
        ros2 topic pub /robot/simple_command std_msgs/String "data: 'move forward 2 meters'"
        ros2 topic pub /robot/simple_command std_msgs/String "data: 'turn right 90 degrees then move ahead 1m'"
        
        📊 Monitor Status:
        ------------------
        ros2 topic echo /robot/commander_status
        
        ==========================================
        """
        print(instructions)
    
    def command_callback(self, msg):
        """Handle incoming movement commands"""
        command_text = msg.data.strip()
        if not command_text:
            return
            
        self.get_logger().info(f"📝 Received command: '{command_text}'")
        
        if self.is_executing:
            self.get_logger().warn("⚠️  Already executing a command. Send 'stop' to cancel.")
            return
            
        # Parse and execute command
        self.execute_command_sequence(command_text)
    
    def execute_command_sequence(self, command_text: str):
        """Parse and execute a sequence of movement commands"""
        try:
            # Parse the command into individual actions
            actions = self.parse_command(command_text)
            
            if not actions:
                self.get_logger().error(f"❌ Could not parse command: '{command_text}'")
                self.publish_status("error", f"Could not parse command: {command_text}")
                return
            
            self.get_logger().info(f"🎯 Parsed {len(actions)} action(s): {actions}")
            
            # Execute actions in a separate thread
            self.current_command = command_text
            self.is_executing = True
            self.stop_execution = False
            
            self.execution_thread = threading.Thread(
                target=self._execute_actions_thread,
                args=(actions,),
                daemon=True
            )
            self.execution_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"❌ Error executing command: {e}")
            self.publish_status("error", str(e))
    
    def parse_command(self, command_text: str) -> List[Dict]:
        """Parse natural language command into actionable steps"""
        command_text = command_text.lower().strip()
        actions = []
        
        # Split by common connectors
        segments = re.split(r'\b(?:then|and|after that|next)\b', command_text)
        
        for segment in segments:
            segment = segment.strip()
            if not segment:
                continue
                
            action = self.parse_single_command(segment)
            if action:
                actions.append(action)
        
        return actions
    
    def parse_single_command(self, segment: str) -> Optional[Dict]:
        """Parse a single movement command segment"""
        
        # Check for stop command first
        if re.search(self.movement_patterns[r'\b(?:stop|halt|freeze|brake)\b'], segment):
            return {'action': 'stop'}
        
        # Try to match movement patterns
        action_type = None
        for pattern, action in self.movement_patterns.items():
            if re.search(pattern, segment):
                action_type = action
                break
        
        if not action_type:
            return None
            
        action = {'action': action_type}
        
        # Extract distance for movement commands
        if action_type in ['forward', 'backward', 'left', 'right']:
            distance = self.extract_distance(segment)
            if distance is not None:
                action['distance'] = distance
            else:
                # Default distance if not specified
                action['distance'] = 1.0
                
        # Extract angle for turn commands
        elif action_type in ['turn_left', 'turn_right']:
            angle = self.extract_angle(segment)
            if angle is not None:
                action['angle'] = angle
            else:
                # Default angle if not specified (90 degrees)
                action['angle'] = math.pi / 2  # 90 degrees in radians
        
        return action
    
    def extract_distance(self, text: str) -> Optional[float]:
        """Extract distance value from text"""
        for pattern in self.distance_patterns:
            match = re.search(pattern, text)
            if match:
                value = float(match.group(1))
                
                # Convert based on unit
                if 'cm' in pattern or 'centimeter' in text:
                    return value / 100.0  # Convert cm to meters
                elif 'ft' in pattern or 'feet' in text or 'foot' in text:
                    return value * 0.3048  # Convert feet to meters
                else:
                    return value  # Assume meters
        return None
    
    def extract_angle(self, text: str) -> Optional[float]:
        """Extract angle value from text and convert to radians"""
        for pattern in self.angle_patterns:
            match = re.search(pattern, text)
            if match:
                value = float(match.group(1))
                
                # Convert based on unit
                if 'rad' in pattern or 'radian' in text:
                    return value  # Already in radians
                else:
                    return math.radians(value)  # Convert degrees to radians
        return None
    
    def _execute_actions_thread(self, actions: List[Dict]):
        """Execute actions in a separate thread"""
        try:
            self.publish_status("executing", f"Starting execution of {len(actions)} actions")
            
            for i, action in enumerate(actions):
                if self.stop_execution:
                    self.get_logger().info("🛑 Execution stopped by user")
                    break
                    
                self.get_logger().info(f"🎬 Executing action {i+1}/{len(actions)}: {action}")
                self.publish_status("action", f"Executing: {action}")
                
                success = self._execute_single_action(action)
                if not success:
                    self.get_logger().error(f"❌ Failed to execute action: {action}")
                    break
                    
                # Small pause between actions
                if i < len(actions) - 1 and not self.stop_execution:
                    time.sleep(0.2)
            
            # Ensure robot is stopped at the end
            self._stop_robot()
            
            if not self.stop_execution:
                self.get_logger().info("✅ Command sequence completed successfully")
                self.publish_status("completed", "All actions executed successfully")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in execution thread: {e}")
            self.publish_status("error", str(e))
        finally:
            self.is_executing = False
            self.current_command = None
    
    def _execute_single_action(self, action: Dict) -> bool:
        """Execute a single action"""
        try:
            action_type = action['action']
            
            if action_type == 'stop':
                self._stop_robot()
                return True
                
            elif action_type in ['forward', 'backward', 'left', 'right']:
                distance = action.get('distance', 1.0)
                return self._execute_movement(action_type, distance)
                
            elif action_type in ['turn_left', 'turn_right']:
                angle = action.get('angle', math.pi / 2)
                return self._execute_turn(action_type, angle)
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"❌ Error executing action {action}: {e}")
            return False
    
    def _execute_movement(self, direction: str, distance: float) -> bool:
        """Execute linear movement"""
        try:
            cmd_vel = Twist()
            
            # Set movement direction
            if direction == 'forward':
                cmd_vel.linear.x = self.default_linear_speed
                self.get_logger().info(f"➡️  Moving forward {distance:.2f}m")
            elif direction == 'backward':
                cmd_vel.linear.x = -self.default_linear_speed
                self.get_logger().info(f"⬅️  Moving backward {distance:.2f}m")
            elif direction == 'left':
                cmd_vel.linear.y = self.default_linear_speed
                self.get_logger().info(f"⬆️  Moving left {distance:.2f}m")
            elif direction == 'right':
                cmd_vel.linear.y = -self.default_linear_speed
                self.get_logger().info(f"⬇️  Moving right {distance:.2f}m")
            
            # Calculate movement duration
            duration = distance / self.default_linear_speed
            
            # Execute movement
            start_time = time.time()
            while (time.time() - start_time) < duration and not self.stop_execution:
                self.cmd_vel_pub.publish(cmd_vel)
                time.sleep(0.1)
            
            # Stop movement
            self._stop_robot()
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Movement error: {e}")
            return False
    
    def _execute_turn(self, direction: str, angle: float) -> bool:
        """Execute rotational movement"""
        try:
            cmd_vel = Twist()
            
            # Set rotation direction
            if direction == 'turn_left':
                cmd_vel.angular.z = self.default_angular_speed
                self.get_logger().info(f"↺ Turning left {math.degrees(angle):.1f}°")
            elif direction == 'turn_right':
                cmd_vel.angular.z = -self.default_angular_speed
                self.get_logger().info(f"↻ Turning right {math.degrees(angle):.1f}°")
            
            # Calculate turn duration
            duration = abs(angle) / self.default_angular_speed
            
            # Execute turn
            start_time = time.time()
            while (time.time() - start_time) < duration and not self.stop_execution:
                self.cmd_vel_pub.publish(cmd_vel)
                time.sleep(0.1)
            
            # Stop rotation
            self._stop_robot()
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Turn error: {e}")
            return False
    
    def _stop_robot(self):
        """Stop all robot movement"""
        stop_cmd = Twist()  # All velocities are zero
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Stop execution if running
        if self.is_executing:
            self.stop_execution = True
            
        self.get_logger().info("🛑 Robot stopped")
    
    def publish_status(self, status_type: str, message: str):
        """Publish commander status"""
        status = {
            "type": status_type,
            "message": message,
            "is_executing": self.is_executing,
            "current_command": self.current_command,
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        commander = SimpleRobotCommander()
        
        commander.get_logger().info("🚀 Simple Robot Commander started!")
        commander.get_logger().info("💡 Send commands to: /robot/simple_command")
        commander.get_logger().info("📊 Monitor status at: /robot/commander_status")
        commander.get_logger().info("Example: ros2 topic pub /robot/simple_command std_msgs/String \"data: 'move forward 2 meters then turn right 90 degrees'\"")
        
        rclpy.spin(commander)
        
    except KeyboardInterrupt:
        commander.get_logger().info("🛑 Simple Robot Commander stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'commander' in locals():
                if commander.is_executing:
                    commander._stop_robot()
                commander.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()