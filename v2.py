import asyncio
import websockets
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading
import time
import sys # For sys.exit()

# --- Configuration ---
WEBSOCKET_URL = "ws://192.168.4.1:8765"  # Your WebSocket server address
MAX_POINTS_TO_DISPLAY = 100000 # Increased buffer for more history if needed
REFRESH_INTERVAL_MS = 50 # Faster refresh for smoother animation (20 FPS)

# --- Global variables for data and plot ---
points_data = deque(maxlen=MAX_POINTS_TO0DISPLAY)
fig, ax = None, None
line_plot = None
anim_running = False # Initialized as global here
data_received_count = 0 # To track if any data is being received

def update_plot():
    global line_plot, points_data, anim_running # Declare anim_running as global

    if not anim_running:
        # print("Animation not running, skipping plot update.") # Debug: Too verbose
        return

    if not points_data:
        # print("No points in deque, skipping plot update.") # Debug: Too verbose
        return

    # Convert deque of points to a NumPy array for plotting
    # Make a copy to avoid issues if deque changes during conversion
    current_points = np.array(list(points_data))

    if line_plot is None:
        print("Initializing plot...")
        fig.patch.set_facecolor('black') # Set background to black for better visibility of white points
        ax.set_facecolor('black')
        ax.set_xlabel("X (mm)", color='white')
        ax.set_ylabel("Y (mm)", color='white')
        ax.set_title("LiDAR Scan (XY Plane)", color='white')
        ax.set_aspect('equal', adjustable='box') # Keep aspect ratio for accurate representation
        ax.grid(True, color='gray', linestyle='--') # Grid for reference

        ax.tick_params(axis='x', colors='white') # White tick labels
        ax.tick_params(axis='y', colors='white')

        # Set initial limits - CRITICAL FOR VISIBILITY!
        # These are sensible defaults for LiDAR scans in millimeters
        # Adjust these based on your expected LiDAR range
        ax.set_xlim(-5000, 5000) # Example: -5m to 5m
        ax.set_ylim(-5000, 5000)

        # Plot the data as individual points (white for visibility on black background)
        line_plot, = ax.plot(current_points[:, 0], current_points[:, 1], 'o',
                             markersize=1.5, # Smaller points
                             color='cyan',   # A bright color that stands out
                             alpha=0.7,      # Slightly transparent
                             animated=True)  # Optimize for animation
    else:
        # Update existing plot data
        line_plot.set_data(current_points[:, 0], current_points[:, 1])

    # Redraw the canvas
    # This is crucial for interactive mode animation.
    fig.canvas.restore_region(ax.patch)
    ax.draw_artist(line_plot)
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()


async def receive_data():
    global points_data, anim_running, data_received_count # Declare anim_running as global
    print(f"Connecting to WebSocket at {WEBSOCKET_URL}...")
    while True:
        try:
            async with websockets.connect(WEBSOCKET_URL, ping_interval=None) as websocket: # ping_interval=None for potential dev server issues
                print("Connected to WebSocket. Receiving data...")
                anim_running = True # This assignment now refers to the global variable
                while True:
                    try:
                        message = await websocket.recv()
                        # print(f"Received message length: {len(message)}") # Debug: See raw message length

                        data_flat = np.fromstring(message, sep=',', dtype=np.float32)

                        if data_flat.size > 0 and data_flat.size % 3 == 0:
                            new_points = data_flat.reshape(-1, 3)
                            # Add new points to the deque
                            for p in new_points:
                                points_data.append(p)
                            data_received_count += new_points.shape[0] # Increment count
                            # print(f"Total points in buffer: {len(points_data)} (Last received: {new_points.shape[0]})") # Debug
                        else:
                            print(f"WARNING: Received malformed data (flat size: {data_flat.size}). Sample: '{message[:100]}...'")
                    except websockets.exceptions.ConnectionClosedOK:
                        print("WebSocket connection closed normally.")
                        anim_running = False # This assignment now refers to the global variable
                        break
                    except websockets.exceptions.ConnectionClosedError as e:
                        print(f"ERROR: WebSocket connection closed with error: {e}")
                        anim_running = False # This assignment now refers to the global variable
                        break
                    except Exception as e:
                        print(f"ERROR: Error receiving data: {e}")
                        anim_running = False # This assignment now refers to the global variable
                        break
        except ConnectionRefusedError:
            print(f"Connection refused. Ensure your LiDAR server is running at {WEBSOCKET_URL}. Retrying in 3 seconds...")
            anim_running = False # This assignment now refers to the global variable
            await asyncio.sleep(3)
        except Exception as e:
            print(f"ERROR: Error connecting to WebSocket: {e}")
            anim_running = False # This assignment now refers to the global variable
            await asyncio.sleep(3)

def check_data_flow_and_exit():
    """Checks if data is being received after some initial time."""
    global data_received_count # Declare as global to access it
    time.sleep(5) # Wait for 5 seconds for initial data
    if data_received_count == 0:
        print("\n--- DIAGNOSTIC HELP ---")
        print("No LiDAR data points were received from the WebSocket server after 5 seconds.")
        print("Possible reasons:")
        print("1. Your LiDAR server (Python script with Lidar and websockets) is NOT running.")
        print(f"2. The WebSocket URL in visualizer.py ({WEBSOCKET_URL}) is incorrect.")
        print("3. The LiDAR device itself is not connected or not sending data.")
        print("4. The Lidar.read_loop_websocket is not actually calling send_fn (bcast).")
        print("5. Firewall blocking port 8765.")
        print("Please check your server script's output for any errors or 'Connected' messages.")
        print("Ensure 'asyncio.run(main())' is at the end of your server script.")
        print("-----------------------\n")
        sys.exit(1) # Exit the script

def run_asyncio_loop():
    asyncio.run(receive_data())

def run_matplotlib_animation():
    global fig, ax, anim_running # Declare anim_running as global here as well

    fig, ax = plt.subplots(figsize=(8, 8))
    plt.ion() # Turn on interactive mode
    plt.show(block=False) # Non-blocking show

    # Start a separate thread to check if data is flowing
    # This prevents the script from hanging indefinitely if no data arrives
    check_thread = threading.Thread(target=check_data_flow_and_exit, daemon=True)
    check_thread.start()

    while plt.fignum_exists(fig.number): # Loop as long as the plot window is open
        if anim_running: # Now this correctly refers to the global variable
            try:
                update_plot()
            except Exception as e:
                print(f"ERROR: Error during plot update: {e}")
                anim_running = False # Stop trying to update plot if it fails
        plt.pause(REFRESH_INTERVAL_MS / 1000.0) # Pause for specified interval

    print("Plot window closed. Exiting visualizer.")
    # You might want to signal the receive_data thread to stop here
    # (e.g., by setting a global flag that breaks its loop)
    # For now, it will likely continue in the background until the main process exits.


if __name__ == "__main__":
    # Start the WebSocket client in a separate thread
    websocket_thread = threading.Thread(target=run_asyncio_loop, daemon=True)
    websocket_thread.start()

    # Start the Matplotlib animation in the main thread
    run_matplotlib_animation()
