#!/usr/bin/env python3
"""
Gemini Robot Controller Launch File
Launches the complete robot control system with vision-based object detection
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare launch arguments
    gemini_api_key_arg = DeclareLaunchArgument(
        'gemini_api_key',
        default_value='',
        description='Google Gemini API key for vision AI (set via environment or parameter)'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/k1_camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    neck_control_topic_arg = DeclareLaunchArgument(
        'neck_control_topic',
        default_value='/robot/neck_position',
        description='Topic for controlling robot neck movement'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic for robot movement commands'
    )
    
    # Get Gemini API key from environment if not provided
    gemini_api_key = os.getenv('GEMINI_API_KEY', LaunchConfiguration('gemini_api_key'))
    
    # Main Gemini Robot Controller Node
    gemini_controller_node = Node(
        package='t1_keyboard_control',
        executable='gemini_robot_controller',
        name='gemini_robot_controller',
        output='screen',
        parameters=[{
            'gemini_api_key': gemini_api_key,
            'camera_topic': LaunchConfiguration('camera_topic'),
            'neck_control_topic': LaunchConfiguration('neck_control_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'search_mode': True,
            'detection_confidence': 0.7,
            'approach_distance': 1.0
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    # K1 Camera Agent (for camera streaming)
    k1_camera_agent_node = Node(
        package='t1_keyboard_control',
        executable='k1_camera_agent',
        name='k1_camera_agent',
        output='screen',
        parameters=[{
            'log_level': 'info'
        }]
    )
    
    # Image Conversion Node (to handle encoding issues)
    image_converter_node = Node(
        package='t1_keyboard_control',
        executable='image_converter',
        name='image_converter',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('camera_topic'),
            'output_topic': '/k1_camera/image_raw/converted',
            'output_encoding': 'bgr8'
        }]
    )
    
    # RViz2 with robot visualization (delayed start)
    rviz_node = TimerAction(
        period=5.0,  # Wait 5 seconds for other nodes to start
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/home/het/ros2_ws/src/t1_keyboard_control/config/gemini_robot.rviz'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gemini_api_key_arg,
        camera_topic_arg,
        neck_control_topic_arg,
        cmd_vel_topic_arg,
        gemini_controller_node,
        k1_camera_agent_node,
        image_converter_node,
        rviz_node,
    ])