# K1 Robot Camera Agent 🤖📹

An intelligent agent that automatically discovers, connects to, and streams video from K1 robot cameras over Ethernet connections.

## Features

### 🧠 **Intelligent Discovery**
- **Network Topology Analysis**: Automatically detects Ethernet interfaces and subnets
- **Robot Candidate Scanning**: Tests common robot IP addresses and network ranges  
- **Service Detection**: Probes for camera services on discovered devices
- **Confidence Scoring**: Ranks potential robots based on service responses

### 🔗 **Automatic Connection**  
- **Multi-Protocol Support**: Tests HTTP, HTTPS, and RTSP camera streams
- **Fallback Discovery**: Multiple discovery methods if primary fails
- **Connection Persistence**: Automatically saves and reuses working configurations
- **Reconnection Logic**: Handles network interruptions gracefully

### 📡 **Streaming Capabilities**
- **ROS2 Integration**: Publishes camera feed as ROS2 Image messages
- **Web Interface**: Optional HTTP streaming for browser viewing
- **Real-time Status**: Continuous monitoring and reporting
- **Frame Statistics**: Tracks streaming performance

## Quick Start

### 🚀 **Launch the Agent**
```bash
# Start the intelligent K1 camera agent
ros2 launch t1_keyboard_control k1_camera_agent_launch.py
```

### 📱 **Access Camera Stream**
Once the agent discovers and connects to your K1 robot:
- **ROS Topic**: `/k1_camera/image_raw`  
- **Web Interface**: http://your-laptop-ip:5000
- **Direct Stream**: http://your-laptop-ip:5000/video_feed

## How It Works

### 1. **Network Analysis** 🌐
```
🔍 Analyzing network topology...
🔗 Found Ethernet interface: enxa0cec861c322 (192.168.10.10)
🌐 Scanning subnet: 192.168.10.0/24
```

### 2. **Robot Discovery** 🎯  
```
🔍 Scanning for K1 robot candidates...
🎯 Found robot candidate: 192.168.10.100
📡 Testing services on candidate...
✅ Detected K1 robot services
```

### 3. **Camera Connection** 📹
```
📹 Detecting camera services...
🔍 Checking camera services on 192.168.10.100
📹 Found camera service: http://192.168.10.100:8080/video
🎯 Connecting to: http://192.168.10.100:8080/video
✅ Successfully connected to K1 camera!
```

### 4. **Streaming** 🎬
```
🎬 Starting camera streaming...
📊 Agent Status: Connected | Frames: 1250 | Robot: 192.168.10.100
```

## Configuration

### **Launch Parameters**
```bash
# Custom streaming port
ros2 launch t1_keyboard_control k1_camera_agent_launch.py stream_port:=8080

# Disable web streaming (ROS only)
ros2 launch t1_keyboard_control k1_camera_agent_launch.py enable_web_stream:=false
```

### **Saved Configuration**
The agent automatically saves working configurations to:
```
~/.ros/k1_robot_config.yaml
```

## Network Discovery Details

### **Tested IP Ranges**
The agent automatically tests these common robot IP addresses:
- `192.168.1.100`, `192.168.1.10`, `192.168.1.1`
- `192.168.0.100`, `192.168.0.10`, `192.168.0.1`  
- `192.168.10.100`, `192.168.10.10`, `192.168.10.1`
- `10.0.0.100`, `10.0.0.10`, `10.0.0.1`
- `172.16.0.100`, `172.16.0.10`, `172.16.0.1`

### **Camera Service Paths**
Tests these common camera endpoints:
- `/video`, `/camera`, `/stream`, `/video_feed`
- `/mjpeg`, `/webcam`, `/cam`, `/live`

### **Service Ports**  
Scans these typical robot service ports:
- `8080` (most common for cameras)
- `8000`, `5000` (alternative HTTP ports)
- `80`, `443` (standard web ports)
- `9090` (ROS bridge)

## Troubleshooting

### **Connection Issues**
```bash
# Check agent logs for discovery details
ros2 topic echo /rosout

# Verify network connectivity
ping 192.168.10.100

# Test camera URL manually
curl http://192.168.10.100:8080/video
```

### **Common Solutions**
1. **Robot Not Found**: Ensure robot is powered and network-connected
2. **Camera Not Accessible**: Check if camera service is running on robot
3. **Network Issues**: Verify Ethernet connection between laptop and robot
4. **Firewall**: Ensure ports 8080, 8000, 5000 are accessible

### **Manual Override**
If automatic discovery fails, you can manually specify robot details:
```bash
# Edit saved config
nano ~/.ros/k1_robot_config.yaml

# Set specific robot IP
robot_ip: "192.168.10.100"
camera_url: "http://192.168.10.100:8080/video"
```

## Advanced Features

### **Multi-Robot Support** (Future)
The agent architecture supports connecting to multiple K1 robots simultaneously.

### **Camera Quality Control**
- Automatic resolution detection
- Adaptive quality based on network conditions
- Frame rate monitoring and adjustment

### **Security Features**
- Authentication support for protected camera feeds
- SSL/TLS encryption for secure connections
- Access logging and monitoring

## Integration Examples

### **With Other ROS Nodes**
```bash
# Use camera feed with image processing
ros2 run image_view image_view image:=/k1_camera/image_raw

# Record camera feed
ros2 bag record /k1_camera/image_raw

# Process with custom node
ros2 run my_package image_processor --ros-args -r image:=/k1_camera/image_raw
```

### **With Computer Vision**
The agent publishes standard ROS2 Image messages compatible with:
- OpenCV
- cv_bridge  
- image_transport
- All ROS2 vision packages

## Status Monitoring

### **Real-time Status**
```bash
# Monitor agent status
ros2 topic echo /rosout | grep k1_camera_agent

# Check camera topic
ros2 topic hz /k1_camera/image_raw

# View camera info
ros2 topic info /k1_camera/image_raw
```

### **Web Dashboard**
Access real-time status and controls at: http://your-laptop-ip:5000

---

## 🎉 **Getting Started**

1. **Connect** your laptop to K1 robot via Ethernet
2. **Launch** the agent: `ros2 launch t1_keyboard_control k1_camera_agent_launch.py`
3. **Wait** for automatic discovery (usually 10-30 seconds)
4. **Access** camera stream at http://your-laptop-ip:5000

The agent handles everything else automatically! 🚀

---

## Legacy Components (T1 Robot)

The package also includes components for the T1 robot system:

### Camera Viewer Node (`camera_viewer.py`)
- Automatically discovers camera topics in ROS2
- Republishes feeds for RViz visualization
- Handles both raw and compressed image formats

### Camera Demo Node (`camera_demo.py`)  
- Publishes synthetic camera feeds for testing
- Creates multiple camera topics with colorful patterns

### Quick Start with T1 Demo
```bash
cd /home/het/ros2_ws
source install/setup.bash

# Terminal 1 - Start demo cameras
ros2 run t1_keyboard_control camera_demo

# Terminal 2 - Start camera viewer
ros2 run t1_keyboard_control camera_viewer
   
   # Terminal 3 - Start RViz
   rviz2 -d src/t1_keyboard_control/config/camera_viewer.rviz
   ```

2. **Or use the launch file:**
   ```bash
   cd /home/het/ros2_ws
   source install/setup.bash
   ros2 launch t1_keyboard_control camera_viewer_launch.py
   ```

### Using with Real T1 Robot Cameras

1. **Start your T1 robot** and ensure camera topics are being published

2. **Check available camera topics:**
   ```bash
   ros2 topic list | grep -E "(camera|image)"
   ```

3. **Start the camera viewer:**
   ```bash
   ros2 run t1_keyboard_control camera_viewer
   ```

4. **View in RViz:**
   ```bash
   rviz2 -d src/t1_keyboard_control/config/camera_viewer.rviz
   ```

### Manual RViz Setup

If you prefer to set up RViz manually:

1. **Start RViz:** `rviz2`
2. **Set Fixed Frame** to `camera_frame` (or appropriate frame)
3. **Add Image display:**
   - Click "Add" → "By topic" 
   - Find `/camera_viewer/*` topics
   - Select "Image" under your desired camera topic
4. **Configure image display** as needed

## Features

### Automatic Discovery
- The camera viewer automatically detects new camera topics every 5 seconds
- Supports both raw `sensor_msgs/Image` and `sensor_msgs/CompressedImage`
- Looks for common camera topic patterns: `camera`, `image`, `rgb`, `depth`, `stereo`, etc.

### Multi-format Support
- **Raw Images**: Direct pass-through with timestamp updates
- **Compressed Images**: Automatic decompression to raw format for RViz

### Real-time Monitoring
- Status updates every 10 seconds showing active cameras
- Frame rate and resolution information
- Error handling and logging

### Thread Safety
- Multi-threaded execution for smooth performance
- Thread-safe image storage and processing

## Topic Mapping

The camera viewer republishes discovered topics with the prefix `/camera_viewer`:

```
Original Topic                    → Republished Topic
/t1/camera/front/image_raw       → /camera_viewer/t1/camera/front/image_raw
/camera/rgb/image_raw            → /camera_viewer/camera/rgb/image_raw
/stereo/left/image_raw/compressed → /camera_viewer/stereo/left/image_raw/compressed
```

## Configuration

### Camera Viewer Parameters
Currently, the camera viewer uses built-in parameters. Future versions may support:
- Custom topic discovery patterns
- Frame rate limiting
- Image resizing
- Custom republish topic prefixes

### RViz Configuration
The included RViz config (`camera_viewer.rviz`) includes:
- Grid display for reference
- Pre-configured Image display
- Optimal window layout for camera viewing
- Standard RViz tools (move, zoom, etc.)

## Troubleshooting

### No Camera Topics Found
```bash
# Check if any camera topics exist
ros2 topic list

# Check topic types
ros2 topic info /your/camera/topic

# Test with demo
ros2 run t1_keyboard_control camera_demo
```

### RViz Shows Black Image
1. Check that the topic is publishing: `ros2 topic hz /camera_viewer/your/topic`
2. Verify image encoding is supported
3. Check RViz Fixed Frame matches image frame_id
4. Try different image transport settings

### Performance Issues
1. Check system resources: `htop`
2. Reduce camera resolution if possible
3. Limit number of active camera feeds
4. Check network bandwidth for remote cameras

## Integration with T1 Robot

This camera viewer integrates seamlessly with the existing T1 robot control system:

- **Use alongside keyboard controller:**
  ```bash
  # Terminal 1 - Robot control
  ros2 run t1_keyboard_control keyboard_controller <network_interface>
  
  # Terminal 2 - Camera viewing  
  ros2 run t1_keyboard_control camera_viewer
  
  # Terminal 3 - Visualization
  rviz2 -d src/t1_keyboard_control/config/camera_viewer.rviz
  ```

- **Monitor robot movement** while viewing camera feeds
- **Visual feedback** for navigation and control
- **Record camera data** using ROS2 bag while controlling robot

## Dependencies

- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Standard sensor message types  
- `cv_bridge` - OpenCV-ROS bridge
- `opencv2` - Computer vision library
- `std_msgs` - Standard message types

## Future Enhancements

- [ ] Camera calibration display
- [ ] Multi-camera synchronization
- [ ] Image annotation tools
- [ ] Recording and playback features
- [ ] Custom image processing filters
- [ ] Web-based camera viewer
- [ ] Camera parameter adjustment interface