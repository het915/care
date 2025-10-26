#!/usr/bin/env python3
"""
Launch file for Camera Stream Demo
Starts both the test camera publisher and the K1 camera streamer
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/test_camera/image',
        description='Camera topic to use'
    )
    
    stream_port_arg = DeclareLaunchArgument(
        'stream_port',
        default_value='5000',
        description='HTTP streaming port'
    )
    
    # Test Camera Publisher Node
    test_camera_node = Node(
        package='t1_keyboard_control',
        executable='test_camera_publisher',
        name='test_camera_publisher',
        output='screen',
        parameters=[{
            'publish_topic': LaunchConfiguration('camera_topic'),
            'frame_rate': 10.0,
            'image_width': 640,
            'image_height': 480,
        }]
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
            'stream_host': '0.0.0.0',
            'jpeg_quality': 80,
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        stream_port_arg,
        test_camera_node,
        k1_camera_streamer_node,
    ])