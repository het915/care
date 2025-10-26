#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'network_interface',
            default_value='eno1',
            description='Network interface for robot connection'
        ),
        
        DeclareLaunchArgument(
            'gemini_api_key',
            default_value='',
            description='Google Gemini API key for vision processing'
        ),
        
        # Log info
        LogInfo(
            msg='🚀 Starting Gemini Robot Controller with CMD_VEL Bridge'
        ),
        
        # CMD_VEL Bridge Node - bridges /cmd_vel to Booster SDK
        Node(
            package='t1_keyboard_control',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=[LaunchConfiguration('network_interface')],
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface')
            }]
        ),
        
        # Gemini Robot Controller - AI vision and decision making
        Node(
            package='t1_keyboard_control',
            executable='gemini_robot_controller',
            name='gemini_robot_controller',
            output='screen',
            parameters=[{
                'gemini_api_key': LaunchConfiguration('gemini_api_key'),
                'camera_topic': '/k1_camera/image_raw',
                'neck_control_topic': '/robot/neck_position',
                'cmd_vel_topic': '/cmd_vel',
                'search_mode': True,
                'detection_confidence': 0.7,
                'approach_distance': 1.0
            }]
        ),
        
        LogInfo(
            msg='✅ Both nodes started!'
        ),
        
        LogInfo(
            msg='📝 Usage:'
        ),
        
        LogInfo(
            msg='  • Send detection commands: ros2 topic pub /robot/find_object std_msgs/String "data: \'find a person\'"'
        ),
        
        LogInfo(
            msg='  • Monitor robot status: ros2 topic echo /robot/status'
        ),
        
        LogInfo(
            msg='  • Monitor bridge status: ros2 topic echo /cmd_vel_bridge/status'
        ),
    ])