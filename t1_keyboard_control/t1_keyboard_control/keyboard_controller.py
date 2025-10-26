#!/usr/bin/env python3


import rclpy 
from rclpy.node import Node
import sys
import termios
import tty
import select

# Import Booster SDK
try:
    from booster_robotics_sdk_python import (
        B1LocoClient, 
        ChannelFactory,
        RobotMode,
        GetModeResponse,
        B1HandAction,
        kChangeMode,
        kMove
    )
except ImportError as e:
    print(f"Error: Booster SDK not found. Please install the SDK first.")
    print(f"ImportError details: {e}")
    print("Run: cd booster_robotics_sdk/build && sudo cmake .. -DBUILD_PYTHON_BINDING=on && sudo make && sudo make install")
    sys.exit(1)


class T1KeyboardController(Node):
    """ROS2 node for controlling T1 robot with keyboard"""
    
    def __init__(self, network_interface='eno1'):
        super().__init__('t1_keyboard_controller')
        
        self.network_interface = network_interface
        self.get_logger().info(f'Initializing T1 Keyboard Controller on interface {network_interface}')
        
        # Initialize Booster SDK
        try:
            ChannelFactory.Instance().Init(0, network_interface)
            self.client = B1LocoClient()
            self.client.Init()
            self.get_logger().info('Booster SDK initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Booster SDK: {e}')
            raise
        
        # Movement parameters
        self.vx = 0.0  # Forward/backward speed
        self.vy = 0.0  # Left/right speed
        self.vyaw = 0.0  # Rotational speed
        
        # Speed settings (adjust these based on robot capabilities)
        self.linear_speed = 0.3  # m/s for forward/backward
        self.lateral_speed = 0.2  # m/s for left/right  
        self.angular_speed = 0.5  # rad/s for rotation
        
        # Head rotation settings (in radians)
        self.head_pitch = 0.0  # Up/down rotation (-0.3 to 1.0 rad)
        self.head_yaw = 0.0    # Left/right rotation (-0.785 to 0.785 rad)
        self.head_pitch_step = 0.1  # Pitch adjustment per keypress
        self.head_yaw_step = 0.1    # Yaw adjustment per keypress
        
        # Terminal settings for keyboard input
        self.settings = None
        
        # Print instructions
        self.print_instructions()
        
        # Check robot mode
        self.check_robot_status()

    def print_instructions(self):
        """Print keyboard control instructions"""
        msg = """
        ==========================================
        T1 Robot Keyboard Control
        ==========================================
        
        Movement Controls:
        ------------------
        W : Move Forward
        S : Move Backward
        A : Move Left
        D : Move Right
        Q : Rotate Left (CCW)
        E : Rotate Right (CW)
        SPACE : Stop movement
        
        Head Controls:
        --------------
        I : Tilt head up
        K : Tilt head down
        J : Turn head left
        L : Turn head right
        U : Reset head position
        
        Hand Controls:
        --------------
        H : Wave hand (open/close)
        
        Mode Controls:
        --------------
        1 : Switch to PREP mode (standing)
        2 : Switch to WALK mode (ready to walk)
        0 : Switch to DAMP mode (safe mode)
        
        Other:
        ------
        M : Check current mode
        X : Exit program
        
        ==========================================
        Press keys to control the robot...
        ==========================================
        """
        print(msg)

    def get_key(self):
        """Get keyboard input (non-blocking)"""
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1).lower()
        return None

    def check_robot_status(self):
        """Check and display current robot mode"""
        try:
            response = GetModeResponse()
            result = self.client.GetMode(response)
            if result == 0:
                mode = response.mode
                mode_names = {
                    RobotMode.kDamping: "DAMP (Safe Mode)",
                    RobotMode.kPrepare: "PREP (Standing)",
                    RobotMode.kWalking: "WALK (Ready to Move)"
                }
                self.get_logger().info(f'Current robot mode: {mode_names.get(mode, "UNKNOWN")}')
                
                if mode != RobotMode.kWalking:
                    self.get_logger().warn('Robot is not in WALK mode. Press "2" to enter WALK mode.')
            else:
                self.get_logger().error(f'Failed to get robot mode, error code: {result}')
        except Exception as e:
            self.get_logger().error(f'Failed to get robot mode: {e}')

    def change_mode(self, target_mode):
        """Change robot operating mode"""
        mode_names = {
            RobotMode.kDamping: "DAMP",
            RobotMode.kPrepare: "PREP", 
            RobotMode.kWalking: "WALK"
        }
        
        try:
            self.get_logger().info(f'Changing mode to {mode_names.get(target_mode, "UNKNOWN")}...')
            
            # If switching to WALK mode, must go through PREP first
            if target_mode == RobotMode.kWalking:
                # First switch to PREP
                self.get_logger().info('Switching to PREP mode first...')
                result = self.client.ChangeMode(RobotMode.kPrepare)
                if result != 0:
                    self.get_logger().error(f'Failed to switch to PREP mode: {result}')
                    return False
                
                # Wait a moment
                rclpy.spin_once(self, timeout_sec=1.0)
                
                # Now switch to WALK
                self.get_logger().info('Now switching to WALK mode...')
                result = self.client.ChangeMode(RobotMode.kWalking)
            else:
                result = self.client.ChangeMode(target_mode)
            
            if result == 0:
                self.get_logger().info(f'Successfully changed to {mode_names.get(target_mode)} mode')
                return True
            else:
                self.get_logger().error(f'Mode change failed with code: {result}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to change mode: {e}')
            return False

    def send_movement_command(self):
        """Send movement command to robot"""
        try:
            result = self.client.Move(self.vx, self.vy, self.vyaw)
            if result != 0:
                self.get_logger().warn(f'Movement command returned code: {result}')
        except Exception as e:
            self.get_logger().error(f'Failed to send movement command: {e}')
    
    def rotate_head(self, pitch, yaw):
        """Rotate robot head
        
        Args:
            pitch: Up/down angle in radians (-0.3 to 1.0, downward is positive)
            yaw: Left/right angle in radians (-0.785 to 0.785, leftward is positive)
        """
        # Clamp values to valid ranges
        pitch = max(-0.3, min(1.0, pitch))
        yaw = max(-0.785, min(0.785, yaw))
        
        try:
            result = self.client.RotateHead(pitch, yaw)
            if result == 0:
                self.get_logger().info(f'Head rotated to pitch={pitch:.2f}, yaw={yaw:.2f}')
            else:
                self.get_logger().warn(f'Head rotation returned code: {result}')
        except Exception as e:
            self.get_logger().error(f'Failed to rotate head: {e}')
    
    def wave_hand(self, action=B1HandAction.kHandOpen):
        """Perform wave hand action
        
        Args:
            action: Hand action (kHandOpen or kHandClose)
        """
        try:
            result = self.client.WaveHand(action)
            if result == 0:
                action_name = 'open' if action == B1HandAction.kHandOpen else 'close'
                self.get_logger().info(f'Hand wave action: {action_name}')
            else:
                self.get_logger().warn(f'Wave hand returned code: {result}')
        except Exception as e:
            self.get_logger().error(f'Failed to wave hand: {e}')

    def stop_robot(self):
        """Stop all robot movement"""
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.send_movement_command()
        self.get_logger().info('Robot stopped')

    def handle_key(self, key):
        """Handle keyboard input and control robot"""
        if key is None:
            return True
        
        # Movement keys
        if key == 'w':
            self.vx = self.linear_speed
            self.vy = 0.0
            self.vyaw = 0.0
            self.get_logger().info('Moving forward')
            self.send_movement_command()
            
        elif key == 's':
            self.vx = -self.linear_speed
            self.vy = 0.0
            self.vyaw = 0.0
            self.get_logger().info('Moving backward')
            self.send_movement_command()
            
        elif key == 'a':
            self.vx = 0.0
            self.vy = self.lateral_speed
            self.vyaw = 0.0
            self.get_logger().info('Moving left')
            self.send_movement_command()
            
        elif key == 'd':
            self.vx = 0.0
            self.vy = -self.lateral_speed
            self.vyaw = 0.0
            self.get_logger().info('Moving right')
            self.send_movement_command()
            
        elif key == 'q':
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = self.angular_speed
            self.get_logger().info('Rotating left')
            self.send_movement_command()
            
        elif key == 'e':
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = -self.angular_speed
            self.get_logger().info('Rotating right')
            self.send_movement_command()
            
        elif key == ' ':
            self.stop_robot()
        
        # Head control keys
        elif key == 'i':
            self.head_pitch -= self.head_pitch_step
            self.rotate_head(self.head_pitch, self.head_yaw)
            
        elif key == 'k':
            self.head_pitch += self.head_pitch_step
            self.rotate_head(self.head_pitch, self.head_yaw)
            
        elif key == 'j':
            self.head_yaw += self.head_yaw_step
            self.rotate_head(self.head_pitch, self.head_yaw)
            
        elif key == 'l':
            self.head_yaw -= self.head_yaw_step
            self.rotate_head(self.head_pitch, self.head_yaw)
            
        elif key == 'u':
            self.head_pitch = 0.0
            self.head_yaw = 0.0
            self.rotate_head(self.head_pitch, self.head_yaw)
            self.get_logger().info('Head position reset')
        
        # Hand control keys
        elif key == 'h':
            # Toggle between open and close
            if not hasattr(self, '_hand_open'):
                self._hand_open = True
            self._hand_open = not self._hand_open
            action = B1HandAction.kHandOpen if self._hand_open else B1HandAction.kHandClose
            self.wave_hand(action)
        
        # Mode change keys
        elif key == '0':
            self.stop_robot()
            self.change_mode(RobotMode.kDamping)
            
        elif key == '1':
            self.stop_robot()
            self.change_mode(RobotMode.kPrepare)
            
        elif key == '2':
            self.change_mode(RobotMode.kWalking)
        
        # Status check
        elif key == 'm':
            self.check_robot_status()
        
        # Exit
        elif key == 'x':
            self.get_logger().info('Exiting...')
            self.stop_robot()
            return False
        
        return True

    def run(self):
        """Main control loop"""
        # Set terminal to raw mode for immediate key capture
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            running = True
            while running and rclpy.ok():
                # Get keyboard input
                key = self.get_key()
                
                # Handle the key
                running = self.handle_key(key)
                
                # Spin ROS2 node
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f'Error in main loop: {e}')
            
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Stop robot before exiting
            self.stop_robot()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Get network interface from command line argument if provided
    network_interface = 'eno1'  # Default network interface
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]
    
    try:
        controller = T1KeyboardController(network_interface)
        controller.run()
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