#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.3',
            description='Default linear speed in m/s'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.5', 
            description='Default angular speed in rad/s'
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eno1',
            description='Network interface for robot communication (e.g., eno1, enx607d09c27a48)'
        ),
        
        # Startup message
        LogInfo(
            msg="🚀 Starting Simple Robot Command System..."
        ),
        
        # CMD_VEL Bridge - Translates /cmd_vel to Booster SDK commands
        Node(
            package='t1_keyboard_control',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface')
            }],
            prefix='stdbuf -o L',
        ),
        
        # Simple Robot Commander - Natural language movement commands  
        Node(
            package='t1_keyboard_control',
            executable='simple_robot_commander',
            name='simple_robot_commander',
            output='screen',
            parameters=[{
                'default_linear_speed': LaunchConfiguration('linear_speed'),
                'default_angular_speed': LaunchConfiguration('angular_speed'),
                'command_topic': '/robot/simple_command',
                'status_topic': '/robot/commander_status'
            }],
            prefix='stdbuf -o L',
        ),
        
        # Completion message with usage examples
        LogInfo(
            msg=[
                "\n",
                "==========================================\n",
                "🤖 SIMPLE ROBOT COMMANDER READY!\n",
                "==========================================\n",
                "\n",
                "📝 NATURAL LANGUAGE COMMANDS:\n",
                "  • 'move forward 2 meters'\n",
                "  • 'go back 1.5m'\n",
                "  • 'turn right 90 degrees'\n",
                "  • 'move left 0.5 meters'\n",
                "  • 'move ahead 2m then turn right 45 degrees'\n",
                "  • 'go forward 1 meter and turn left'\n",
                "  • 'stop'\n",
                "\n",
                "💻 COMMAND EXAMPLES:\n",
                "  # Move forward 2 meters:\n",
                "  ros2 topic pub --once /robot/simple_command std_msgs/String \"data: 'move forward 2 meters'\"\n",
                "\n",
                "  # Turn right 90 degrees:\n",  
                "  ros2 topic pub --once /robot/simple_command std_msgs/String \"data: 'turn right 90 degrees'\"\n",
                "\n",
                "  # Complex command:\n",
                "  ros2 topic pub --once /robot/simple_command std_msgs/String \"data: 'move ahead 2m then turn right 90 degrees'\"\n",
                "\n", 
                "  # Stop robot:\n",
                "  ros2 topic pub --once /robot/simple_command std_msgs/String \"data: 'stop'\"\n",
                "\n",
                "📊 MONITOR STATUS:\n",
                "  ros2 topic echo /robot/commander_status\n",
                "\n",
                "🌐 NETWORK INTERFACE:\n",
                "  # Use custom network interface:\n",
                "  ros2 launch t1_keyboard_control simple_robot_launch.py network_interface:=enx607d09c27a48\n",
                "\n",
                "📡 TOPICS:\n",
                "  • /robot/simple_command - Send movement commands\n",
                "  • /robot/commander_status - Monitor execution status\n", 
                "  • /cmd_vel - Low-level movement commands\n",
                "\n",
                "==========================================\n"
            ]
        )
    ])