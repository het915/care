#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
import sys
import termios
import tty
import select
import asyncio
import websockets
import json
import logging
from datetime import datetime
import threading

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

# ------------------------- WebSocket Server Class -------------------------
class GestureServer:
    def __init__(self, robot_controller):
        self.clients = set()
        self.event_history = []
        self.robot_controller = robot_controller
        self.logger = logging.getLogger("SpectaclesServer")

    async def register_client(self, websocket):
        """Register a new client connection."""
        self.clients.add(websocket)
        self.logger.info(f"Client connected. Total clients: {len(self.clients)}")

    async def unregister_client(self, websocket):
        """Unregister a client connection."""
        self.clients.discard(websocket)
        self.logger.info(f"Client disconnected. Total clients: {len(self.clients)}")

    async def handle_message(self, websocket, message):
        """Handle incoming gesture or button data."""
        try:
            # Clean up the message by removing any trailing characters that might cause JSON errors
            clean_message = message.rstrip('#0123456789')
            
            data = json.loads(clean_message)
            msg_type = data.get("type")

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # ---------------- Handle Button Data ----------------
            if msg_type == "button":
                button = data.get("button", "unknown")
                action = data.get("action", "unknown")
                rotate_by = data.get("rotateBy", 0)  # Get rotation value

                self.logger.info(f"[{timestamp}] Button: {button} | Action: {action} | RotateBy: {rotate_by}")

                self.event_history.append({
                    "timestamp": timestamp,
                    "type": "button",
                    "button": button,
                    "action": action
                })

                # Map button presses to keyboard commands
                await self.handle_button_command(button, action, rotate_by)

                ack = {
                    "type": "ack",
                    "message": f"Received {action} on {button}",
                    "timestamp": timestamp
                }
                await websocket.send(json.dumps(ack))

                # Broadcast to other clients (if any)
                if len(self.clients) > 1:
                    broadcast_msg = {
                        "type": "button_broadcast",
                        "data": data,
                        "timestamp": timestamp
                    }
                    await self.broadcast(json.dumps(broadcast_msg), exclude=websocket)

            # ---------------- Handle Gesture Data ----------------
            elif msg_type == "gesture":
                hand = data.get("hand", "unknown")
                direction = data.get("direction", "unknown")

                self.logger.info(f"[{timestamp}] {hand} hand: {direction}")

                self.event_history.append({
                    "timestamp": timestamp,
                    "type": "gesture",
                    "hand": hand,
                    "direction": direction,
                    "position": data.get("position", {})
                })

                # Map gestures to keyboard commands
                await self.handle_gesture_command(hand, direction)

                ack = {
                    "type": "ack",
                    "message": f"Received {direction} gesture from {hand} hand",
                    "timestamp": timestamp
                }
                await websocket.send(json.dumps(ack))

                # Broadcast to other clients
                if len(self.clients) > 1:
                    broadcast_msg = {
                        "type": "gesture_broadcast",
                        "data": data,
                        "timestamp": timestamp
                    }
                    await self.broadcast(json.dumps(broadcast_msg), exclude=websocket)

            # ---------------- Handle Unknown Data ----------------
            else:
                self.logger.info(f"Received unrecognized message: {message}")

            # Keep only the last 100 events
            if len(self.event_history) > 100:
                self.event_history = self.event_history[-100:]

        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON received: {message}")
        except Exception as e:
            self.logger.error(f"Error handling message: {e}")

    async def handle_button_command(self, button, action, rotate_by=0):
        """Map button presses to robot commands with rotation support"""
        try:
            # Button mapping as requested:
            # up button + triggerDown -> W (forward)
            # up button + triggerUp -> Space (stop)
            # down button + triggerDown -> S (backward)  
            # down button + triggerUp -> Space (stop)
            # left button + triggerDown -> A (left)
            # left button + triggerUp -> Space (stop)
            # right button + triggerDown -> D (right)
            # right button + triggerUp -> Space (stop)
            # turn_left/turn_right with rotateBy -> Q/E based on rotation direction
            
            command_key = None
            
            if button == "up":
                if action == "triggerDown":
                    command_key = 'w'  # Move forward
                elif action == "triggerUp":
                    command_key = ' '  # Stop
            elif button == "down":
                if action == "triggerDown":
                    command_key = 's'  # Move backward
                elif action == "triggerUp":
                    command_key = ' '  # Stop
            elif button == "left":
                if action == "triggerDown":
                    command_key = 'a'  # Move left
                elif action == "triggerUp":
                    command_key = ' '  # Stop
            elif button == "right":
                if action == "triggerDown":
                    command_key = 'd'  # Move right
                elif action == "triggerUp":
                    command_key = ' '  # Stop
            elif button == "turn_left" or button == "turn_right":
                if action == "triggerDown" and rotate_by != 0:
                    # Map rotation to Q/E based on rotateBy value (-180 to +180)
                    if rotate_by > 0:  # Positive rotation = turn right
                        command_key = 'e'  # Turn right
                    else:  # Negative rotation = turn left
                        command_key = 'q'  # Turn left
                    
                    # Execute multiple times for larger rotations (short intervals)
                    rotation_intensity = min(abs(rotate_by) / 30, 5)  # Scale rotation
                    repeat_count = max(1, int(rotation_intensity))
                    
                    self.logger.info(f"Executing rotation command '{command_key}' {repeat_count} times for rotation: {rotate_by}")
                    
                    # Execute the rotation command multiple times for intensity
                    for i in range(repeat_count):
                        self.robot_controller.handle_websocket_command(command_key)
                        # Small delay between commands for short intervals
                        await asyncio.sleep(0.1)
                    
                    return  # Return early since we handled the rotation commands
                elif action == "triggerUp":
                    command_key = ' '  # Stop rotation
            
            if command_key:
                self.logger.info(f"Executing robot command for key: '{command_key}'")
                # Execute the command in the robot controller
                self.robot_controller.handle_websocket_command(command_key)
                
        except Exception as e:
            self.logger.error(f"Error handling button command: {e}")

    async def handle_gesture_command(self, hand, direction):
        """Map gestures to robot commands (optional - you can extend this)"""
        try:
            # Example gesture mappings (you can customize these):
            command_key = None
            
            if direction == "swipe_up":
                command_key = 'i'  # Head up
            elif direction == "swipe_down":
                command_key = 'k'  # Head down
            elif direction == "swipe_left":
                command_key = 'j'  # Head left
            elif direction == "swipe_right":
                command_key = 'l'  # Head right
            elif direction == "wave":
                command_key = 'h'  # Wave hand
            
            if command_key:
                self.logger.info(f"Executing robot command for gesture key: '{command_key}'")
                self.robot_controller.handle_websocket_command(command_key)
                
        except Exception as e:
            self.logger.error(f"Error handling gesture command: {e}")

    async def broadcast(self, message, exclude=None):
        """Broadcast a message to all connected clients except the excluded one."""
        if not self.clients:
            return

        disconnected = set()
        for client in self.clients:
            if client != exclude:
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)

        # Clean up disconnected clients
        for client in disconnected:
            await self.unregister_client(client)

    async def handle_client(self, websocket):
        """Handle an individual client connection."""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister_client(websocket)


class T1KeyboardController(Node):
    """ROS2 node for controlling T1 robot with keyboard and WebSocket"""
    
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
        
        # WebSocket server
        self.websocket_server = None
        self.websocket_thread = None
        self.websocket_loop = None
        
        # Setup logging for WebSocket server
        logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
        
        # Print instructions
        self.print_instructions()
        
        # Check robot mode
        self.check_robot_status()

    def print_instructions(self):
        """Print keyboard control instructions"""
        msg = """
        ==========================================
        T1 Robot Control (Keyboard + WebSocket)
        ==========================================
        
        Keyboard Controls:
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
        
        WebSocket Controls (Spectacles):
        --------------------------------
        Up Button: Press = W, Release = SPACE
        Down Button: Press = S, Release = SPACE  
        Left Button: Press = A, Release = SPACE
        Right Button: Press = D, Release = SPACE
        
        WebSocket Server: ws://0.0.0.0:8765
        ==========================================
        """
        print(msg)

    def start_websocket_server(self):
        """Start WebSocket server in a separate thread"""
        def run_server():
            # Create new event loop for this thread
            self.websocket_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.websocket_loop)
            
            async def start_server_async():
                server = GestureServer(self)
                
                # Start the WebSocket server
                start_server = await websockets.serve(
                    server.handle_client,
                    "0.0.0.0",
                    8765,
                    ping_interval=20,
                    ping_timeout=10
                )
                
                self.get_logger().info("WebSocket server started on ws://0.0.0.0:8765")
                self.get_logger().info("Spectacles can now connect to control the robot")
                
                # Keep the server running
                await start_server.wait_closed()
            
            # Run the async server
            try:
                self.websocket_loop.run_until_complete(start_server_async())
            except Exception as e:
                self.get_logger().error(f"WebSocket server error: {e}")
        
        self.websocket_thread = threading.Thread(target=run_server, daemon=True)
        self.websocket_thread.start()

    def stop_websocket_server(self):
        """Stop WebSocket server"""
        if self.websocket_loop:
            self.websocket_loop.call_soon_threadsafe(self.websocket_loop.stop)

    def handle_websocket_command(self, key):
        """Handle commands from WebSocket (same as keyboard commands)"""
        self.get_logger().info(f"WebSocket command received: '{key}'")
        self.handle_key(key, from_websocket=True)

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
        """Rotate robot head"""
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
        """Perform wave hand action"""
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

    def handle_key(self, key, from_websocket=False):
        """Handle keyboard input and control robot"""
        if key is None:
            return True
        
        source = "WebSocket" if from_websocket else "Keyboard"
        
        # Movement keys
        if key == 'w':
            self.vx = self.linear_speed
            self.vy = 0.0
            self.vyaw = 0.0
            self.get_logger().info(f'[{source}] Moving forward')
            self.send_movement_command()
            
        elif key == 's':
            self.vx = -self.linear_speed
            self.vy = 0.0
            self.vyaw = 0.0
            self.get_logger().info(f'[{source}] Moving backward')
            self.send_movement_command()
            
        elif key == 'a':
            self.vx = 0.0
            self.vy = self.lateral_speed
            self.vyaw = 0.0
            self.get_logger().info(f'[{source}] Moving left')
            self.send_movement_command()
            
        elif key == 'd':
            self.vx = 0.0
            self.vy = -self.lateral_speed
            self.vyaw = 0.0
            self.get_logger().info(f'[{source}] Moving right')
            self.send_movement_command()
            
        elif key == 'q':
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = self.angular_speed
            self.get_logger().info(f'[{source}] Rotating left')
            self.send_movement_command()
            
        elif key == 'e':
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = -self.angular_speed
            self.get_logger().info(f'[{source}] Rotating right')
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
        
        # Mode change keys (only from keyboard for safety)
        elif key == '0' and not from_websocket:
            self.stop_robot()
            self.change_mode(RobotMode.kDamping)
            
        elif key == '1' and not from_websocket:
            self.stop_robot()
            self.change_mode(RobotMode.kPrepare)
            
        elif key == '2' and not from_websocket:
            self.change_mode(RobotMode.kWalking)
        
        # Status check (only from keyboard)
        elif key == 'm' and not from_websocket:
            self.check_robot_status()
        
        # Exit (only from keyboard)
        elif key == 'x' and not from_websocket:
            self.get_logger().info('Exiting...')
            self.stop_robot()
            return False
        
        return True

    def run(self):
        """Main control loop"""
        # Start WebSocket server
        self.start_websocket_server()
        
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
            # Stop WebSocket server
            self.stop_websocket_server()
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