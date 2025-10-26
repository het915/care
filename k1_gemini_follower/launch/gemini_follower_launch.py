#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for K1 Gemini Person Follower"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('k1_gemini_follower')
    config_file = os.path.join(pkg_dir, 'config', 'gemini_follower_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/zed_real_robot/image_raw',
        description='Camera topic to subscribe to'
    )
    
    use_booster_sdk_arg = DeclareLaunchArgument(
        'use_booster_sdk',
        default_value='true',
        description='Use Booster SDK for robot control (false uses cmd_vel)'
    )
    
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enxa0cec861c322',
        description='Network interface for Booster SDK'
    )
    
    # Create the node
    gemini_follower_node = Node(
        package='k1_gemini_follower',
        executable='gemini_person_follower',
        name='gemini_person_follower',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'use_booster_sdk': LaunchConfiguration('use_booster_sdk'),
                'network_interface': LaunchConfiguration('network_interface'),
            }
        ],
        remappings=[
            ('/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    # Log startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'K1 Gemini Person Follower Launch\n',
            '=' * 60, '\n',
            'Configuration:\n',
            '  - Camera Topic: ', LaunchConfiguration('camera_topic'), '\n',
            '  - Use Booster SDK: ', LaunchConfiguration('use_booster_sdk'), '\n',
            '  - Network Interface: ', LaunchConfiguration('network_interface'), '\n',
            '  - Config File: ', LaunchConfiguration('config_file'), '\n',
            '=' * 60, '\n',
            '\n',
            'SAFETY NOTES:\n',
            '1. Ensure K1 robot is in a safe area before starting\n',
            '2. Robot will switch to WALK mode automatically\n',
            '3. Press Ctrl+C to stop following and switch to DAMP mode\n',
            '4. Keep clear path for robot movement\n',
            '\n',
            'CONTROLS:\n',
            '- Robot will automatically detect and follow a person\n',
            '- Detection runs every 1 second using Gemini Vision API\n',
            '- Robot will rotate to search if no person is found\n',
            '- Robot will approach and maintain distance from detected person\n',
            '=' * 60, '\n'
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        config_file_arg,
        camera_topic_arg,
        use_booster_sdk_arg,
        network_interface_arg,
        
        # Startup message
        startup_msg,
        
        # Node
        gemini_follower_node,
    ])