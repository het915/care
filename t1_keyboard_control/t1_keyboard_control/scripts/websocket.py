#!/usr/bin/env python3
"""
Spectacles WebSocket Server
---------------------------
Receives both gesture and button data from Spectacles AR glasses.

✅ Supports:
  - Multiple simultaneous clients
  - Gesture and button message types
  - Broadcasting to all connected clients
  - JSON-structured acknowledgments
  - Logs and stores the last 100 messages

💡 Usage:
  1. Run this server on your laptop.
  2. Find your local IP (use `ipconfig` or `ifconfig`).
  3. In your Spectacles TypeScript file, set:
       private readonly WEBSOCKET_URL = "ws://<your-ip>:8765";
  4. Ensure both devices are on the same Wi-Fi.
"""

import asyncio
import websockets
import json
import logging
from datetime import datetime

# ------------------------- Logging Setup -------------------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SpectaclesServer")


# ------------------------- Server Class --------------------------
class GestureServer:
    def __init__(self):
        self.clients = set()
        self.event_history = []

    async def register_client(self, websocket):
        """Register a new client connection."""
        self.clients.add(websocket)
        logger.info(f"Client connected. Total clients: {len(self.clients)}")

    async def unregister_client(self, websocket):
        """Unregister a client connection."""
        self.clients.discard(websocket)
        logger.info(f"Client disconnected. Total clients: {len(self.clients)}")

    async def handle_message(self, websocket, message):
        """Handle incoming gesture or button data."""
        try:
            data = json.loads(message)
            msg_type = data.get("type")

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # ---------------- Handle Button Data ----------------
            if msg_type == "button":
                button = data.get("button", "unknown")
                action = data.get("action", "unknown")

                logger.info(f"[{timestamp}] Button: {button} | Action: {action}")

                self.event_history.append({
                    "timestamp": timestamp,
                    "type": "button",
                    "button": button,
                    "action": action
                })

                ack = {
                    "type": "ack",
                    "message": f"Received {action} on {button}",
                    "timestamp": timestamp
                }
                await websocket.send(json.dumps(ack))

                # Broadcast to other clients (if any)
                if len(self.clients) > 1:
                    broadcast_msg = {
                        "type": "button_broadcast",
                        "data": data,
                        "timestamp": timestamp
                    }
                    await self.broadcast(json.dumps(broadcast_msg), exclude=websocket)

            # ---------------- Handle Gesture Data ----------------
            elif msg_type == "gesture":
                hand = data.get("hand", "unknown")
                direction = data.get("direction", "unknown")

                logger.info(f"[{timestamp}] {hand} hand: {direction}")

                self.event_history.append({
                    "timestamp": timestamp,
                    "type": "gesture",
                    "hand": hand,
                    "direction": direction,
                    "position": data.get("position", {})
                })

                ack = {
                    "type": "ack",
                    "message": f"Received {direction} gesture from {hand} hand",
                    "timestamp": timestamp
                }
                await websocket.send(json.dumps(ack))

                # Broadcast to other clients
                if len(self.clients) > 1:
                    broadcast_msg = {
                        "type": "gesture_broadcast",
                        "data": data,
                        "timestamp": timestamp
                    }
                    await self.broadcast(json.dumps(broadcast_msg), exclude=websocket)

            # ---------------- Handle Unknown Data ----------------
            else:
                logger.info(f"Received unrecognized message: {message}")

            # Keep only the last 100 events
            if len(self.event_history) > 100:
                self.event_history = self.event_history[-100:]

        except json.JSONDecodeError:
            logger.error(f"Invalid JSON received: {message}")
        except Exception as e:
            logger.error(f"Error handling message: {e}")

    async def broadcast(self, message, exclude=None):
        """Broadcast a message to all connected clients except the excluded one."""
        if not self.clients:
            return

        disconnected = set()
        for client in self.clients:
            if client != exclude:
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)

        # Clean up disconnected clients
        for client in disconnected:
            await self.unregister_client(client)

    async def handle_client(self, websocket):
        """Handle an individual client connection."""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister_client(websocket)


# ------------------------- Server Entrypoint -------------------------
async def main():
    server = GestureServer()

    # Listen on all interfaces so Spectacles or other laptops can connect
    start_server = websockets.serve(
        server.handle_client,
        "0.0.0.0",  # Allow connections from other devices
        8765,
        ping_interval=20,
        ping_timeout=10
    )

    logger.info("Starting WebSocket server on ws://0.0.0.0:8765")
    logger.info("Update WEBSOCKET_URL in Spectacles script to ws://<your-ip>:8765")

    await start_server

    try:
        await asyncio.Future()  # Run forever
    except KeyboardInterrupt:
        logger.info("Server stopped by user")


if __name__ == "__main__":
    print("Spectacles WebSocket Server")
    print("===========================")
    print("Receives button/gesture data from Spectacles AR glasses.")
    print("Ensure both devices are on the same Wi-Fi network.")
    print()

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")