# Gemini Robot Controller 🤖🧠

An intelligent ROS2 node that integrates Google Gemini Vision API with K1 robot control for autonomous object detection, tracking, and movement.

## Features ✨

- **AI Vision Processing**: Uses Google Gemini Vision API for object detection and scene understanding
- **Neck Rotation Control**: Autonomously rotates robot neck to track detected objects
- **Movement Control**: Commands robot to move forward, backward, and rotate based on AI analysis
- **Voice Commands**: Accepts natural language instructions like "find the red ball" or "approach the person"
- **Real-time Streaming**: Live camera feed with object detection overlays
- **RViz Integration**: Visualizes detected objects, robot pose, and movement paths

## System Architecture 🏗️

```
┌─────────────────────┐    ┌──────────────────────┐    ┌─────────────────────┐
│   K1 Robot Camera   │────│  Gemini Robot        │────│   Robot Movement    │
│   - Video Stream    │    │  Controller          │    │   - Neck Control    │
│   - Image Capture   │    │  - AI Processing     │    │   - Base Movement   │
└─────────────────────┘    │  - Object Detection  │    └─────────────────────┘
                          │  - Decision Making   │              
┌─────────────────────┐    │  - Coordinate Calc   │    ┌─────────────────────┐
│   Voice Commands    │────│                      │────│   RViz Display      │
│   - Natural Lang    │    └──────────────────────┘    │   - Object Markers  │
│   - Text Prompts    │                               │   - Robot State     │
└─────────────────────┘                               └─────────────────────┘
```

## Prerequisites 📋

### 1. ROS2 Humble
```bash
sudo apt install ros-humble-desktop
```

### 2. Google Gemini API Key
1. Visit [Google AI Studio](https://aistudio.google.com/app/apikey)
2. Create an API key
3. Set environment variable:
```bash
export GEMINI_API_KEY="your_api_key_here"
# Add to ~/.bashrc for permanent setup
echo 'export GEMINI_API_KEY="your_api_key_here"' >> ~/.bashrc
```

### 3. K1 Robot Setup
- Ensure K1 robot is powered on
- Connected to same network
- Camera service running
- Default IP: `192.168.10.102`

### 4. Python Dependencies
```bash
pip install google-generativeai opencv-python pillow numpy requests
```

## Installation 🛠️

### 1. Build Workspace
```bash
cd /home/het/ros2_ws
colcon build --packages-select t1_keyboard_control
source install/setup.bash
```

### 2. Verify Setup
```bash
python3 scripts/setup_gemini_controller.py
```

## Usage 🚀

### Quick Start
```bash
# 1. Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/het/ros2_ws/install/setup.bash

# 2. Set Gemini API key
export GEMINI_API_KEY="your_api_key_here"

# 3. Launch complete system
ros2 launch t1_keyboard_control gemini_robot_controller_launch.py
```

### Individual Node Testing
```bash
# Camera agent only
ros2 run t1_keyboard_control k1_camera_agent

# Gemini controller only  
ros2 run t1_keyboard_control gemini_robot_controller

# Image converter for RViz
ros2 run t1_keyboard_control image_converter
```

## Voice Commands 🗣️

Send commands via ROS topic:
```bash
# Find specific objects
ros2 topic pub /voice_command std_msgs/String "data: 'find the red ball'"
ros2 topic pub /voice_command std_msgs/String "data: 'look for a person'"
ros2 topic pub /voice_command std_msgs/String "data: 'detect the blue cup'"

# Movement commands
ros2 topic pub /voice_command std_msgs/String "data: 'approach the object'"
ros2 topic pub /voice_command std_msgs/String "data: 'move closer to the person'"
ros2 topic pub /voice_command std_msgs/String "data: 'turn left and find the chair'"

# Search modes
ros2 topic pub /voice_command std_msgs/String "data: 'start searching'"
ros2 topic pub /voice_command std_msgs/String "data: 'stop and look around'"
```

## Topics & Services 📡

### Subscribed Topics
- `/k1_camera/image_raw` - Camera feed from K1 robot
- `/voice_command` - Natural language commands

### Published Topics
- `/robot/neck_position` - Neck rotation commands (pitch/yaw)
- `/cmd_vel` - Robot base movement commands
- `/detected_objects` - Object detection markers for RViz
- `/robot_pose` - Current robot pose
- `/robot_status` - System status and mode

### Services
- `/set_search_mode` - Enable/disable autonomous searching
- `/get_detection_info` - Get current detection status

## Configuration ⚙️

### Launch Parameters
```bash
ros2 launch t1_keyboard_control gemini_robot_controller_launch.py \
    gemini_api_key:="your_key" \
    camera_topic:="/k1_camera/image_raw" \
    search_mode:=true \
    detection_confidence:=0.7 \
    approach_distance:=1.0
```

### Node Parameters
```yaml
gemini_robot_controller:
  ros__parameters:
    gemini_api_key: "your_gemini_api_key"
    camera_topic: "/k1_camera/image_raw"
    neck_control_topic: "/robot/neck_position"
    cmd_vel_topic: "/cmd_vel"
    search_mode: true
    detection_confidence: 0.7
    approach_distance: 1.0
    max_neck_pitch: 30.0  # degrees
    max_neck_yaw: 45.0    # degrees
    movement_speed: 0.3   # m/s
    rotation_speed: 0.5   # rad/s
```

## Troubleshooting 🔧

### Common Issues

1. **"Unsupported image encoding [nv12]" in RViz**
   - Solution: The image_converter node handles this automatically
   - Check: `ros2 topic echo /k1_camera/image_raw/converted`

2. **K1 Robot Not Found**
   ```bash
   # Manual IP check
   ping 192.168.10.102
   curl http://192.168.10.102:8080
   
   # Network discovery
   ros2 run t1_keyboard_control k1_camera_agent
   ```

3. **Gemini API Errors**
   ```bash
   # Test API connection
   python3 -c "
   import google.generativeai as genai
   import os
   genai.configure(api_key=os.getenv('GEMINI_API_KEY'))
   model = genai.GenerativeModel('gemini-1.5-flash')
   print(model.generate_content('Hello').text)
   "
   ```

4. **No Object Detection**
   - Check camera topic: `ros2 topic hz /k1_camera/image_raw`
   - Verify lighting conditions
   - Adjust detection_confidence parameter

### Debug Commands
```bash
# Check all topics
ros2 topic list

# Monitor detection status
ros2 topic echo /robot_status

# View camera feed
ros2 run image_view image_view image:=/k1_camera/image_raw/converted

# Check node status
ros2 node list
ros2 node info /gemini_robot_controller
```

## Development 👨‍💻

### Adding Custom Objects
Edit `gemini_robot_controller.py`:
```python
# Add to object detection prompt
def create_detection_prompt(self, user_command="general detection"):
    return f"""
    Analyze this image and find: {user_command}
    
    Custom objects to detect:
    - Your custom object here
    - Another custom object
    
    Return JSON with object locations...
    """
```

### Custom Movement Patterns
```python
def custom_movement_pattern(self, object_info):
    """Implement custom robot movement logic"""
    if object_info['type'] == 'custom_object':
        # Your custom movement code
        self.rotate_neck_to_object(object_info)
        self.move_towards_object(object_info)
```

## Performance Tips ⚡

1. **Reduce API Calls**: Adjust processing frequency
2. **Optimize Prompts**: Use specific detection prompts
3. **Cache Results**: Store recent detections
4. **Network Optimization**: Use local processing when possible

## Safety Notes ⚠️

- Always supervise robot movement
- Set appropriate movement speed limits
- Test in safe environment first
- Emergency stop: `Ctrl+C` or `ros2 topic pub /cmd_vel geometry_msgs/Twist`

## License 📄

This project is part of the t1_keyboard_control ROS2 package.

---

**Happy Robot AI Development!** 🎉

For issues and contributions, please check the main package documentation.