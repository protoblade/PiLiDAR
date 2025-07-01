import asyncio
import websockets
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

# --- Configuration ---
WEBSOCKET_URL = "ws://192.168.4.1:8765"  # Your WebSocket server address
MAX_POINTS_TO_DISPLAY = 50000  # Adjust based on performance and desired history
REFRESH_INTERVAL_MS = 100 # Milliseconds between plot updates

# --- Global variables for data and plot ---
points_data = deque(maxlen=MAX_POINTS_TO_DISPLAY)
fig, ax = None, None
line_plot = None
anim_running = False

def update_plot():
    global line_plot, points_data, anim_running

    if not anim_running:
        return

    if not points_data:
        return

    # Convert deque of points to a NumPy array for plotting
    current_points = np.array(list(points_data))

    if line_plot is None:
        # Initialize plot on first data arrival
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("LiDAR Scan (XY Plane)")
        ax.set_aspect('equal', adjustable='box') # Keep aspect ratio for accurate representation
        ax.grid(True)

        # Set initial limits (you might want to make these dynamic or configurable)
        ax.set_xlim(-5000, 5000) # Example: -5m to 5m
        ax.set_ylim(-5000, 5000)

        # Plot the data as individual points
        line_plot, = ax.plot(current_points[:, 0], current_points[:, 1], 'o', markersize=2, alpha=0.7)
    else:
        # Update existing plot data
        line_plot.set_data(current_points[:, 0], current_points[:, 1])

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


async def receive_data():
    global points_data, anim_running
    print(f"Connecting to WebSocket at {WEBSOCKET_URL}...")
    while True:
        try:
            async with websockets.connect(WEBSOCKET_URL) as websocket:
                print("Connected to WebSocket. Receiving data...")
                anim_running = True
                while True:
                    try:
                        message = await websocket.recv()
                        # Parse the comma-separated string into a NumPy array
                        # Each point is [x, y, luminance].
                        # The lidar.points_2d is (N * dlength, 3) where N is out_len
                        # So, message will be a flattened 1D array of x1,y1,l1,x2,y2,l2,...
                        data_flat = np.fromstring(message, sep=',', dtype=np.float32)

                        # Reshape back to (N, 3) where N is number of points
                        # Ensure data_flat has a length divisible by 3
                        if data_flat.size > 0 and data_flat.size % 3 == 0:
                            new_points = data_flat.reshape(-1, 3)
                            # Add new points to the deque
                            for p in new_points:
                                points_data.append(p)
                        else:
                            print(f"Received malformed data (length {data_flat.size}): {message[:100]}...") # Print first 100 chars
                    except websockets.exceptions.ConnectionClosedOK:
                        print("WebSocket connection closed normally.")
                        break
                    except websockets.exceptions.ConnectionClosedError as e:
                        print(f"WebSocket connection closed with error: {e}")
                        break
                    except Exception as e:
                        print(f"Error receiving data: {e}")
                        break
        except ConnectionRefusedError:
            print(f"Connection refused. Retrying in 5 seconds...")
            anim_running = False
            await asyncio.sleep(5)
        except Exception as e:
            print(f"Error connecting to WebSocket: {e}")
            anim_running = False
            await asyncio.sleep(5)

def run_asyncio_loop():
    asyncio.run(receive_data())

def run_matplotlib_animation():
    global fig, ax

    fig, ax = plt.subplots(figsize=(8, 8))
    plt.ion() # Turn on interactive mode
    plt.show()

    while True:
        if anim_running:
            update_plot()
        plt.pause(REFRESH_INTERVAL_MS / 1000.0) # Pause for specified interval
        if not plt.fignum_exists(fig.number): # Check if the window is closed
            print("Plot window closed. Exiting visualizer.")
            break

if __name__ == "__main__":
    # Start the WebSocket client in a separate thread
    websocket_thread = threading.Thread(target=run_asyncio_loop, daemon=True)
    websocket_thread.start()

    # Start the Matplotlib animation in the main thread
    # This needs to be in the main thread for plt.show() to work correctly
    run_matplotlib_animation()
