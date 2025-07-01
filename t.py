import threading
import asyncio
import websockets
import json
from lib.config import Config
from lib.lidar_driver import Lidar
from lib.pointcloud import save_raw_scan, get_scan_dict

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
        # The read_loop_websocket continuously reads data from the LiDAR.
        # Data accumulation (saving) is controlled by lidar.is_data_collection_active.
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
                        # Prepare lidar for scanning, but don't start data collection yet
                        lidar.prepare_for_scan()
                        await websocket.send("Lidar prepared. Send first heading to start data collection.")
                elif command == "stop":
                    lidar.stop_overall_scan() # Stop data accumulation and save
                    await websocket.send("Overall scan stopped and data saved.")
                else:
                    # If not 'start' or 'stop', try to interpret as a heading
                    try:
                        heading = int(message) # Attempt to convert the whole message to an integer
                        if 0 <= heading <= 359:
                            lidar.heading = heading # Update the lidar's current heading
                            # If lidar is prepared and this is the first heading, start data collection
                            if lidar.is_overall_scanning and not lidar.is_data_collection_active:
                                lidar.start_data_collection()
                                await websocket.send(f"Heading updated to: {heading}. Data collection started.")
                            else:
                                await websocket.send(f"Heading updated to: {heading}")
                        else:
                            await websocket.send("Error: Heading must be between 0 and 359.")
                    except ValueError:
                        await websocket.send("Unknown command or invalid heading format. Please send 'start', 'stop', or a whole number (0-359) for heading.")

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
