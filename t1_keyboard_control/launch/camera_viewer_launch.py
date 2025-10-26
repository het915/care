#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    """Launch file to start camera viewer and optionally RViz"""
    
    # Launch arguments
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start RViz automatically'
    )
    
    # Camera viewer node
    camera_viewer_node = Node(
        package='t1_keyboard_control',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    # RViz node (conditional)
    rviz_config_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'camera_viewer.rviz'
    )
    
    rviz_node = ExecuteProcess(
        condition=LaunchConfiguration('start_rviz'),
        cmd=[
            'rviz2',
            '-d', rviz_config_path if os.path.exists(rviz_config_path) else ''
        ],
        output='screen'
    )
    
    return LaunchDescription([
        start_rviz_arg,
        camera_viewer_node,
        rviz_node,
    ])