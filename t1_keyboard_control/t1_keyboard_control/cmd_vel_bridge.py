#!/usr/bin/env python3
"""
CMD_VEL Bridge Node
Subscribes to /cmd_vel topic and translates commands to Booster SDK robot movement
This allows standard ROS navigation commands to control the K1 robot hardware
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import json
import time

# Import Booster SDK
try:
    from booster_robotics_sdk_python import (
        B1LocoClient, 
        ChannelFactory,
        RobotMode,
        GetModeResponse
    )
    BOOSTER_SDK_AVAILABLE = True
except ImportError as e:
    print(f"❌ Error: Booster SDK not found. Please install the SDK first.")
    print(f"ImportError details: {e}")
    BOOSTER_SDK_AVAILABLE = False


class CmdVelBridge(Node):
    """Bridge between ROS /cmd_vel topic and Booster SDK robot control"""
    
    def __init__(self, network_interface='eno1'):
        super().__init__('cmd_vel_bridge')
        
        self.network_interface = network_interface
        self.get_logger().info(f'🌉 Starting CMD_VEL Bridge on interface {network_interface}')
        
        # Initialize Booster SDK
        self.robot_client = None
        if BOOSTER_SDK_AVAILABLE:
            try:
                ChannelFactory.Instance().Init(0, network_interface)
                self.robot_client = B1LocoClient()
                self.robot_client.Init()
                self.get_logger().info('✅ Booster SDK initialized successfully')
                
                # Check robot mode
                self.check_robot_mode()
                
            except Exception as e:
                self.get_logger().error(f'❌ Failed to initialize Booster SDK: {e}')
                self.robot_client = None
        else:
            self.get_logger().error('❌ Booster SDK not available - node cannot function')
            return
        
        # Movement parameters
        self.max_linear_speed = 0.5   # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.command_timeout = 1.0    # seconds
        
        # Last command tracking for safety
        self.last_command_time = 0.0
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publishers for status
        self.status_pub = self.create_publisher(
            String,
            '/cmd_vel_bridge/status',
            10
        )
        
        # Safety timer - stop robot if no commands received
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('🚀 CMD_VEL Bridge ready!')
        self.get_logger().info('📝 Listening on /cmd_vel topic')
        self.get_logger().info('🤖 Commands will be sent to Booster SDK robot')
        
    def check_robot_mode(self):
        """Check current robot mode and warn if not in walking mode"""
        if not self.robot_client:
            return
            
        try:
            response = GetModeResponse()
            result = self.robot_client.GetMode(response)
            if result == 0:
                mode = response.mode
                mode_names = {
                    RobotMode.kDamping: "DAMP (Safe Mode)",
                    RobotMode.kPrepare: "PREP (Standing)", 
                    RobotMode.kWalking: "WALK (Ready to Move)"
                }
                
                mode_name = mode_names.get(mode, "UNKNOWN")
                self.get_logger().info(f'🤖 Current robot mode: {mode_name}')
                
                if mode != RobotMode.kWalking:
                    self.get_logger().warn('⚠️  Robot is not in WALK mode!')
                    self.get_logger().info('🔄 Attempting to switch to WALK mode...')
                    success = self.change_to_walk_mode()
                    if success:
                        self.get_logger().info('✅ Successfully switched to WALK mode')
                        self.get_logger().info('✅ Robot is ready to receive movement commands')
                    else:
                        self.get_logger().error('❌ Failed to switch to WALK mode')
                        self.get_logger().warn('⚠️  Manual mode change required')
                else:
                    self.get_logger().info('✅ Robot is ready to receive movement commands')
                    
            else:
                self.get_logger().error(f'❌ Failed to get robot mode, error code: {result}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to check robot mode: {e}')
    
    def change_to_walk_mode(self):
        """Change robot to WALK mode (PREP -> WALK sequence)"""
        try:
            # First switch to PREP mode
            self.get_logger().info('🔄 Switching to PREP mode...')
            result = self.robot_client.ChangeMode(RobotMode.kPrepare)
            if result != 0:
                self.get_logger().error(f'❌ Failed to switch to PREP mode: {result}')
                return False
            
            # Wait a moment for mode change
            import time
            time.sleep(2)
            
            # Now switch to WALK mode
            self.get_logger().info('🔄 Switching to WALK mode...')
            result = self.robot_client.ChangeMode(RobotMode.kWalking)
            if result != 0:
                self.get_logger().error(f'❌ Failed to switch to WALK mode: {result}')
                return False
                
            # Wait for mode change to complete
            time.sleep(1)
            
            # Verify the mode change
            response = GetModeResponse()
            result = self.robot_client.GetMode(response)
            if result == 0 and response.mode == RobotMode.kWalking:
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error changing to WALK mode: {e}')
            return False
    
    def cmd_vel_callback(self, msg):
        """Handle incoming /cmd_vel commands"""
        if not self.robot_client:
            self.get_logger().warn('⚠️  No robot client available - ignoring command')
            return
            
        # Extract velocities from Twist message
        vx = msg.linear.x   # Forward/backward
        vy = msg.linear.y   # Left/right (usually 0 for differential drive)
        vyaw = msg.angular.z  # Rotation
        
        # Clamp velocities to safe limits
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
        vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))
        vyaw = max(-self.max_angular_speed, min(self.max_angular_speed, vyaw))
        
        # Log the command
        if vx != 0.0 or vy != 0.0 or vyaw != 0.0:
            self.get_logger().info(f'🏃 Movement command: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}')
        else:
            self.get_logger().info('🛑 Stop command received')
        
        # Send command to robot
        self.send_robot_command(vx, vy, vyaw)
        
        # Update tracking
        self.last_command_time = time.time()
        self.last_vx = vx
        self.last_vy = vy  
        self.last_vyaw = vyaw
    
    def send_robot_command(self, vx, vy, vyaw):
        """Send movement command to robot via Booster SDK"""
        if not self.robot_client:
            return
            
        try:
            result = self.robot_client.Move(vx, vy, vyaw)
            if result != 0:
                self.get_logger().warn(f'⚠️  Robot movement returned code: {result}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send movement command: {e}')
    
    def safety_check(self):
        """Safety check - stop robot if no commands received for timeout period"""
        if not self.robot_client:
            return
            
        current_time = time.time()
        
        # If we haven't received commands recently and robot was moving, stop it
        if (self.last_command_time > 0 and 
            current_time - self.last_command_time > self.command_timeout and
            (self.last_vx != 0.0 or self.last_vy != 0.0 or self.last_vyaw != 0.0)):
            
            self.get_logger().warn('⚠️  No recent commands - stopping robot for safety')
            self.send_robot_command(0.0, 0.0, 0.0)
            
            # Reset tracking
            self.last_vx = 0.0
            self.last_vy = 0.0
            self.last_vyaw = 0.0
    
    def publish_status(self):
        """Publish bridge status"""
        status = {
            "bridge_active": self.robot_client is not None,
            "last_command_time": self.last_command_time,
            "current_velocities": {
                "vx": self.last_vx,
                "vy": self.last_vy, 
                "vyaw": self.last_vyaw
            },
            "booster_sdk_available": BOOSTER_SDK_AVAILABLE,
            "timestamp": time.time()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Get network interface from command line argument if provided
    network_interface = 'eno1'  # Default network interface
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]
    
    try:
        bridge = CmdVelBridge(network_interface)
        
        if bridge.robot_client is not None:
            bridge.get_logger().info('🌉 CMD_VEL Bridge is running!')
            bridge.get_logger().info('🎯 Send commands to /cmd_vel topic')
            bridge.get_logger().info('📊 Monitor status: ros2 topic echo /cmd_vel_bridge/status')
            
            rclpy.spin(bridge)
        else:
            bridge.get_logger().error('❌ Bridge failed to initialize - exiting')
            
    except KeyboardInterrupt:
        print('\n🛑 CMD_VEL Bridge stopped by user')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'bridge' in locals() and bridge.robot_client:
                bridge.get_logger().info('🛑 Stopping robot before shutdown...')
                bridge.send_robot_command(0.0, 0.0, 0.0)
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()