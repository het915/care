#!/usr/bin/env python3
"""
K1 Camera Agent Launch File
Launches the intelligent K1 camera discovery and streaming agent
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    stream_port_arg = DeclareLaunchArgument(
        'stream_port',
        default_value='5000',
        description='HTTP streaming port for web interface'
    )
    
    enable_web_stream_arg = DeclareLaunchArgument(
        'enable_web_stream',
        default_value='true',
        description='Enable web streaming interface'
    )
    
    # K1 Camera Agent Node
    k1_camera_agent_node = Node(
        package='t1_keyboard_control',
        executable='k1_camera_agent',
        name='k1_camera_agent',
        output='screen',
        parameters=[{
            'log_level': 'info'
        }]
    )
    
    # K1 Camera Streamer Node (delayed start to allow agent to discover camera)
    k1_camera_streamer_node = TimerAction(
        period=10.0,  # Wait 10 seconds for agent to discover camera
        actions=[
            Node(
                package='t1_keyboard_control',
                executable='k1_camera_streamer',
                name='k1_camera_streamer',
                output='screen',
                parameters=[{
                    'camera_topic': '/k1_camera/image_raw',
                    'stream_port': LaunchConfiguration('stream_port'),
                    'stream_host': '0.0.0.0',
                    'jpeg_quality': 85,
                }]
            )
        ]
    )
    
    return LaunchDescription([
        stream_port_arg,
        enable_web_stream_arg,
        k1_camera_agent_node,
        k1_camera_streamer_node,
    ])