#!/bin/bash
# Gemini Robot Controller Test Commands

# 1. Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/het/ros2_ws/install/setup.bash

# 2. Set Gemini API key (replace with your key)
export GEMINI_API_KEY="your_gemini_api_key_here"

# 3. Launch the complete system
echo "🚀 Launching Gemini Robot Controller..."
ros2 launch t1_keyboard_control gemini_robot_controller_launch.py

# Alternative: Launch individual nodes for debugging
# ros2 run t1_keyboard_control k1_camera_agent
# ros2 run t1_keyboard_control gemini_robot_controller
# ros2 run t1_keyboard_control image_converter

# Test voice commands (in separate terminal):
# ros2 topic pub /voice_command std_msgs/String "data: 'find the red ball'"
# ros2 topic pub /voice_command std_msgs/String "data: 'look for a person'"
# ros2 topic pub /voice_command std_msgs/String "data: 'approach the object'"
