# Spectacles Gesture HUD Controller

A simple HUD system for Snap Spectacles AR glasses that detects pinch gestures and sends directional data to a WebSocket server.

## Features

- **Pinch Gesture Detection**: Detects pinch gestures from both left and right hands
- **Directional Movement**: Tracks movement during pinch to determine direction (up, down, left, right, forward, backward)
- **WebSocket Communication**: Sends gesture data in real-time to a WebSocket server
- **Visual HUD**: Displays status and instructions on the AR display
- **Dual Hand Support**: Works with both left and right hand gestures

## Gesture Types

The system detects the following gesture types:
- **Tap**: Pinch without significant movement
- **Up**: Pinch and move upward
- **Down**: Pinch and move downward  
- **Left**: Pinch and move left
- **Right**: Pinch and move right
- **Forward**: Pinch and move forward
- **Backward**: Pinch and move backward

## Setup Instructions

### 1. Lens Studio Setup

1. **Add the Script**: The `GestureHUDController` script is already created in `Assets/Untitled TypeScript.ts`

2. **Create Scene Objects**:
   - Create a Scene Object in your scene
   - Add the `GestureHUDController` script component to it
   - Add two Text components for status and instruction display

3. **Configure Inputs**:
   - Assign the `InternetModule` asset to the `internetModule` input
   - Assign the status Text component to the `statusText` input
   - Assign the instruction Text component to the `instructionText` input

4. **Update WebSocket URL**:
   - In the script, change `WEBSOCKET_URL` to point to your WebSocket server
   - Default is set to `"wss://echo.websocket.org"` for testing

### 2. WebSocket Server Setup

#### Option A: Use the Provided Python Server

1. **Install Dependencies**:
   ```bash
   pip install websockets
   ```

2. **Run the Server**:
   ```bash
   python websocket_server.py
   ```

3. **Update WebSocket URL**:
   - Change the `WEBSOCKET_URL` in the TypeScript file to: `"ws://YOUR_SERVER_IP:8765"`
   - Replace `YOUR_SERVER_IP` with your computer's IP address

#### Option B: Use a WebSocket Testing Service

For quick testing, you can use online WebSocket echo services:
- `wss://echo.websocket.org`
- `wss://ws.postman-echo.com/raw`

### 3. Spectacles Setup

1. **Enable Hand Tracking**: Ensure hand tracking is enabled in your Spectacles settings
2. **Test Gestures**: 
   - Pinch with either hand
   - Move your hand in different directions while pinching
   - Release the pinch to send the gesture data

## Data Format

The system sends JSON data in the following format:

```json
{
  "type": "gesture",
  "hand": "right",
  "direction": "up",
  "timestamp": 1703123456789,
  "position": {
    "x": 0.123,
    "y": 0.456,
    "z": 0.789
  }
}
```

## Configuration

### Adjustable Parameters

In the `GestureHUDController` script, you can modify:

- `WEBSOCKET_URL`: WebSocket server URL
- `MIN_GESTURE_DISTANCE`: Minimum movement distance to register a directional gesture (default: 0.1)

### WebSocket Server Configuration

In `websocket_server.py`, you can modify:

- Server IP and port (default: localhost:8765)
- Gesture history limit (default: 100 gestures)
- Logging level

## Troubleshooting

### Common Issues

1. **"SIK API not available"**: 
   - Ensure Spectacles Interaction Kit is properly installed
   - Check that the project is configured for Spectacles platform

2. **"InternetModule not assigned"**:
   - Add an InternetModule asset to your project
   - Assign it to the script's internetModule input

3. **WebSocket Connection Failed**:
   - Check your network connection
   - Verify the WebSocket server is running
   - Ensure the URL is correct (ws:// for local, wss:// for secure)

4. **No Gesture Detection**:
   - Ensure hand tracking is enabled
   - Check that hands are visible to the camera
   - Verify the Spectacles Interaction Kit is properly configured

### Debug Information

The system provides debug output in the Lens Studio console:
- WebSocket connection status
- Gesture detection events
- Data transmission confirmations

## Example Usage

1. **Start the WebSocket server**:
   ```bash
   python websocket_server.py
   ```

2. **Deploy to Spectacles** and test gestures

3. **Monitor the server console** for incoming gesture data

4. **Use the gesture data** in your application logic

## Extending the System

### Adding New Gesture Types

To add new gesture types, modify the `calculateGestureDirection()` method in the TypeScript file.

### Custom WebSocket Protocol

Modify the `sendGestureData()` method to send data in your preferred format.

### Additional UI Elements

Add more Text components or UI elements to display additional information in the HUD.

## Requirements

- Snap Lens Studio 5.15+
- Spectacles Interaction Kit
- Spectacles AR glasses
- Python 3.7+ (for the WebSocket server)
- websockets library (for the Python server)

## License

This project is provided as-is for educational and development purposes.
