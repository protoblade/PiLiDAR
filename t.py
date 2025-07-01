import threading
import asyncio
import websockets
import json # Import json for parsing client messages
from lib.config import Config
from lib.lidar_driver import Lidar
from lib.pointcloud import save_raw_scan, get_scan_dict # Ensure these are imported if used for saving

clients = set()

target_resolution = 0.5

config = Config()
config.init()
config.update_target_res(target_resolution)

lidar = Lidar(config)

async def bcast(message):
    """Broadcasts a message to all connected WebSocket clients."""
    for ws in list(clients):
        try:
            await ws.send(message)
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed, removing client.")
            clients.remove(ws)
        except Exception as e:
            print(f"Error sending data to client: {e}")
            pass


def start_lidar_loop():
    """Starts the LiDAR reading loop in a separate thread."""
    try:
        print(f"Max packages: {config.max_packages}")
        # The read_loop_websocket now manages its own scanning state and sends "done"
        lidar.read_loop_websocket(send_fn=bcast, max_packages=config.max_packages)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping read loop.")
        lidar.close()
        print("Exiting program.")
    except Exception as e:
        print(f"Error in start_lidar_loop: {e}")
        lidar.close()


async def handler(websocket, path=None):
    """Handles WebSocket connections and incoming messages."""
    clients.add(websocket)
    # Initialize a flag for this specific WebSocket connection to track scanning state
    websocket.is_scanning_active = False
    try:
        async for message in websocket:
            print(f"Received message: {message}")
            try:
                parts = message.split()
                command = parts[0].lower()

                if command == "start":
                    if len(parts) > 1:
                        await websocket.send("Error: 'start' command does not take any arguments. Just send 'start'.")
                    else:
                        lidar.start_scan()
                        websocket.is_scanning_active = True
                        await websocket.send("Scan started. Send whole numbers (0-359) for heading updates.")
                elif command == "stop":
                    lidar.stop_scan()
                    websocket.is_scanning_active = False
                    await websocket.send("Scan stopped and data saved.")
                else:
                    # If not 'start' or 'stop', try to interpret as a heading
                    if websocket.is_scanning_active:
                        try:
                            heading = int(message) # Attempt to convert the whole message to an integer
                            if 0 <= heading <= 359:
                                lidar.heading = heading
                                await websocket.send(f"Heading updated to: {heading}")
                            else:
                                await websocket.send("Error: Heading must be between 0 and 359.")
                        except ValueError:
                            await websocket.send("Unknown command or invalid heading format. Please send 'start', 'stop', or a whole number (0-359) for heading.")
                    else:
                        await websocket.send("Unknown command. Please send 'start' to begin scanning.")

            except Exception as e:
                print(f"Error processing message: {e}")
                await websocket.send(f"Error processing message: {e}")
    finally:
        clients.remove(websocket)


async def main():
    """Main function to start the LiDAR thread and WebSocket server."""
    # Start lidar read_loop in a background thread
    thread = threading.Thread(target=start_lidar_loop, daemon=True)
    thread.start()

    async with websockets.serve(handler, '', 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
