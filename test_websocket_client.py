#!/usr/bin/env python3
"""
Simple WebSocket client to test the gesture server.
This can be used to verify that the WebSocket server is working correctly.
"""

import asyncio
import websockets
import json
import time

async def test_gesture_server():
    """Test the gesture WebSocket server."""
    uri = "ws://localhost:8765"
    
    try:
        print("Connecting to WebSocket server...")
        async with websockets.connect(uri) as websocket:
            print("Connected! Sending test gesture data...")
            
            # Send test gesture data
            test_gestures = [
                {
                    "type": "gesture",
                    "hand": "right",
                    "direction": "up",
                    "timestamp": int(time.time() * 1000),
                    "position": {"x": 0.1, "y": 0.2, "z": 0.3}
                },
                {
                    "type": "gesture",
                    "hand": "left",
                    "direction": "down",
                    "timestamp": int(time.time() * 1000),
                    "position": {"x": -0.1, "y": -0.2, "z": 0.3}
                },
                {
                    "type": "gesture",
                    "hand": "right",
                    "direction": "left",
                    "timestamp": int(time.time() * 1000),
                    "position": {"x": -0.2, "y": 0.1, "z": 0.3}
                }
            ]
            
            for gesture in test_gestures:
                await websocket.send(json.dumps(gesture))
                print(f"Sent: {gesture['hand']} hand - {gesture['direction']}")
                
                # Wait for response
                response = await websocket.recv()
                print(f"Received: {response}")
                await asyncio.sleep(1)
            
            print("Test completed successfully!")
            
    except websockets.exceptions.ConnectionRefused:
        print("Error: Could not connect to WebSocket server.")
        print("Make sure the server is running on ws://localhost:8765")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    print("WebSocket Gesture Server Test Client")
    print("====================================")
    print("This will test the gesture WebSocket server with sample data.")
    print()
    
    asyncio.run(test_gesture_server())
