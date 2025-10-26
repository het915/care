#!/bin/bash
echo "Checking for available camera topics..."
echo "======================================"
echo
echo "Available Image topics:"
ros2 topic list | grep -i image
echo
echo "Available camera-related topics:"  
ros2 topic list | grep -i camera
echo
echo "Available ZED topics:"
ros2 topic list | grep -i zed
echo
echo "To see topic info, use:"
echo "ros2 topic info [topic_name]"
echo "ros2 topic echo [topic_name] --once"