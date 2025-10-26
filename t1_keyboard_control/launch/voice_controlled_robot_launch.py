#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'gemini_api_key',
            default_value='',
            description='Google Gemini API key for AI vision'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/k1_camera/image_raw',
            description='Camera topic for AI vision'
        ),
        DeclareLaunchArgument(
            'microphone_device',
            default_value='-1',
            description='Microphone device index (-1 for default)'
        ),
        
        # Startup message
        LogInfo(
            msg="🚀 Starting Voice-Controlled AI Robot System..."
        ),
        
        # CMD_VEL Bridge - Translates /cmd_vel to Booster SDK commands
        Node(
            package='t1_keyboard_control',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'network_interface': 'eno1'
            }],
            prefix='stdbuf -o L',  # Line buffered output
        ),
        
        # Gemini AI Robot Controller - Object detection and tracking
        Node(
            package='t1_keyboard_control',
            executable='gemini_robot_controller',
            name='gemini_robot_controller',
            output='screen',
            parameters=[{
                'gemini_api_key': LaunchConfiguration('gemini_api_key'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'cmd_vel_topic': '/cmd_vel',
                'search_mode': True,
                'detection_confidence': 0.7,
                'approach_distance': 1.0
            }],
            prefix='stdbuf -o L',
        ),
        
        # Voice Command Publisher - Speech recognition to robot commands
        Node(
            package='t1_keyboard_control',
            executable='voice_command_publisher',
            name='voice_command_publisher',
            output='screen',
            parameters=[{
                'microphone_device_index': LaunchConfiguration('microphone_device'),
                'listen_timeout': 5.0,
                'phrase_timeout': 1.0,
                'energy_threshold': 4000
            }],
            prefix='stdbuf -o L',
        ),
        
        # Startup completion message
        LogInfo(
            msg=[
                "\n",
                "==========================================\n",
                "🤖 VOICE-CONTROLLED AI ROBOT READY!\n", 
                "==========================================\n",
                "\n",
                "🎤 VOICE COMMANDS:\n",
                "  • 'Move forward/backward/left/right'\n",
                "  • 'Turn left/right'\n", 
                "  • 'Stop' / 'Robot stop'\n",
                "  • 'Find a [object]' / 'Look for [object]'\n",
                "  • 'Follow the [object]' / 'Go to [object]'\n",
                "  • 'Stop searching' / 'Start searching'\n",
                "\n",
                "📡 ROS TOPICS:\n",
                "  • /cmd_vel - Direct movement commands\n",
                "  • /robot/find_object - AI object detection\n",
                "  • /robot/control - Robot state control\n",
                "  • /voice/status - Voice system status\n",
                "\n",
                "🔧 MANUAL CONTROL:\n",
                "  • ros2 topic pub /cmd_vel geometry_msgs/Twist ...\n",
                "  • ros2 topic pub /robot/find_object std_msgs/String \"data: 'find a person'\"\n",
                "  • ros2 topic pub /robot/control std_msgs/String \"data: 'stop'\"\n",
                "\n",
                "📊 MONITORING:\n",
                "  • ros2 topic echo /robot/status\n",
                "  • ros2 topic echo /voice/status\n",
                "  • ros2 topic echo /robot/detection\n",
                "\n",
                "==========================================\n"
            ]
        )
    ])