#!/usr/bin/env python3
"""
Launch file for K1 Camera Streamer
Starts the camera streaming service with configurable parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    stream_port_arg = DeclareLaunchArgument(
        'stream_port',
        default_value='5000',
        description='HTTP streaming port'
    )
    
    stream_host_arg = DeclareLaunchArgument(
        'stream_host',
        default_value='0.0.0.0',
        description='HTTP streaming host (0.0.0.0 for all interfaces)'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG compression quality (1-100)'
    )
    
    # K1 Camera Streamer Node
    k1_camera_streamer_node = Node(
        package='t1_keyboard_control',
        executable='k1_camera_streamer',
        name='k1_camera_streamer',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'stream_port': LaunchConfiguration('stream_port'),
            'stream_host': LaunchConfiguration('stream_host'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
        }],
        remappings=[
            # You can add topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        stream_port_arg,
        stream_host_arg,
        jpeg_quality_arg,
        k1_camera_streamer_node,
    ])