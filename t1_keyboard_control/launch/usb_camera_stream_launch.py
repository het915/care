#!/usr/bin/env python3
"""
Launch file for USB Camera Stream
Starts both the USB camera publisher and the K1 camera streamer
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='USB camera device ID (/dev/video0 = 0, /dev/video1 = 1, etc.)'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic to publish/subscribe'
    )
    
    stream_port_arg = DeclareLaunchArgument(
        'stream_port',
        default_value='5000',
        description='HTTP streaming port'
    )
    
    resolution_width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera width resolution'
    )
    
    resolution_height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Camera height resolution'
    )
    
    flip_horizontal_arg = DeclareLaunchArgument(
        'flip_horizontal',
        default_value='false',
        description='Flip camera horizontally'
    )
    
    flip_vertical_arg = DeclareLaunchArgument(
        'flip_vertical',
        default_value='false',
        description='Flip camera vertically'
    )
    
    # USB Camera Publisher Node
    usb_camera_node = Node(
        package='t1_keyboard_control',
        executable='usb_camera_publisher',
        name='usb_camera_publisher',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'publish_topic': LaunchConfiguration('camera_topic'),
            'frame_rate': 30.0,
            'image_width': LaunchConfiguration('width'),
            'image_height': LaunchConfiguration('height'),
            'flip_horizontal': LaunchConfiguration('flip_horizontal'),
            'flip_vertical': LaunchConfiguration('flip_vertical'),
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
        device_id_arg,
        camera_topic_arg,
        stream_port_arg,
        resolution_width_arg,
        resolution_height_arg,
        flip_horizontal_arg,
        flip_vertical_arg,
        usb_camera_node,
        k1_camera_streamer_node,
    ])