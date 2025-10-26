# K1 Camera Streaming Setup

This setup provides HTTP video streaming from ROS2 camera topics using Flask.

## Components

### 1. K1 Camera Streamer (`k1_camera_streamer`)
- Subscribes to ROS2 camera topics (Image messages)
- Converts images to MJPEG format
- Streams video over HTTP using Flask
- Provides a web interface for viewing

### 2. Test Camera Publisher (`test_camera_publisher`)
- Publishes synthetic test pattern images
- Useful for testing the streaming setup
- Generates moving patterns with timestamp and frame counter

## Quick Start

### Option 1: Launch Everything Together
```bash
# Start both test camera and streamer
ros2 launch t1_keyboard_control camera_stream_demo_launch.py
```

### Option 2: Manual Setup
```bash
# Terminal 1: Start test camera
ros2 run t1_keyboard_control test_camera_publisher

# Terminal 2: Start camera streamer
ros2 run t1_keyboard_control k1_camera_streamer --ros-args -p camera_topic:=/test_camera/image
```

### Option 3: Use with Real USB Camera
```bash
# Start USB camera and streamer together
ros2 launch t1_keyboard_control usb_camera_stream_launch.py

# Or manually:
# Terminal 1: Start USB camera
ros2 run t1_keyboard_control usb_camera_publisher

# Terminal 2: Start streamer with USB camera topic
ros2 run t1_keyboard_control k1_camera_streamer --ros-args -p camera_topic:=/camera/image_raw
```

## Accessing the Stream

### Web Interface
- **URL**: http://192.168.10.10:5000
- **Description**: Full web page with embedded video player
- **Features**: Stream information, responsive design

### Direct MJPEG Stream
- **URL**: http://192.168.10.10:5000/video_feed
- **Description**: Raw MJPEG stream for embedding in other applications
- **Usage**: Can be used in VLC, OBS, or custom applications

## Configuration Parameters

### K1 Camera Streamer Parameters
- `camera_topic`: ROS2 image topic to subscribe to (default: `/zed/zed_node/left/image_rect_color`)
- `stream_host`: HTTP server host (default: `0.0.0.0`)
- `stream_port`: HTTP server port (default: `5000`)
- `jpeg_quality`: JPEG compression quality 1-100 (default: `80`)

### Test Camera Parameters
- `publish_topic`: ROS2 topic to publish images to (default: `/test_camera/image`)
- `frame_rate`: Publishing frame rate in Hz (default: `10.0`)
- `image_width`: Image width in pixels (default: `640`)
- `image_height`: Image height in pixels (default: `480`)

## Example Usage

### Custom Port and Topic
```bash
ros2 run t1_keyboard_control k1_camera_streamer --ros-args \
  -p camera_topic:=/my_camera/image \
  -p stream_port:=8080 \
  -p jpeg_quality:=90
```

### High Quality Stream
```bash
ros2 run t1_keyboard_control k1_camera_streamer --ros-args \
  -p jpeg_quality:=95 \
  -p stream_port:=5001
```

## Network Access

### Local Network Access
The stream is accessible from any device on the same network:
- **Robot IP**: 192.168.10.10
- **Stream URL**: http://192.168.10.10:5000
- **Direct Feed**: http://192.168.10.10:5000/video_feed

### Port Forwarding (if needed)
If accessing from outside the local network, set up port forwarding on your router:
- **External Port**: Choose an available port (e.g., 8080)
- **Internal Port**: 5000
- **Internal IP**: 192.168.10.10

## Troubleshooting

### No Video Stream
1. Check if camera topic is publishing:
   ```bash
   ros2 topic list | grep image
   ros2 topic echo /test_camera/image --once
   ```

2. Verify streamer is subscribed:
   ```bash
   ros2 node info /k1_camera_streamer
   ```

### Network Issues
1. Check firewall settings
2. Verify IP address:
   ```bash
   ip addr show enxa0cec861c322
   ```

3. Test local access:
   ```bash
   curl http://localhost:5000
   ```

### Performance Issues
1. Reduce JPEG quality (lower number = smaller file)
2. Reduce frame rate of source camera
3. Check network bandwidth

## Integration with External Applications

### VLC Media Player
1. Open VLC
2. Go to Media → Open Network Stream
3. Enter: `http://192.168.10.10:5000/video_feed`

### OBS Studio
1. Add Source → Browser Source
2. URL: `http://192.168.10.10:5000`
3. Set appropriate width/height

### Custom Applications
Use the MJPEG stream URL directly in your applications:
- **OpenCV**: `cv2.VideoCapture('http://192.168.10.10:5000/video_feed')`
- **FFmpeg**: `ffmpeg -i http://192.168.10.10:5000/video_feed output.mp4`

## Security Considerations

⚠️ **Warning**: This is a development server without authentication. For production use:
1. Implement authentication
2. Use HTTPS
3. Restrict access by IP
4. Use a production WSGI server (gunicorn, uWSGI)

## Files Structure

```
t1_keyboard_control/
├── t1_keyboard_control/
│   ├── k1_camera_streamer.py          # Main camera streaming node
│   └── test_camera_publisher.py       # Test pattern generator
├── launch/
│   ├── k1_camera_streamer_launch.py   # Streamer launch file
│   └── camera_stream_demo_launch.py   # Demo with test camera
├── config/
│   └── k1_camera_streamer.yaml        # Configuration file
└── scripts/
    └── check_camera_topics.sh         # Helper script to find topics
```