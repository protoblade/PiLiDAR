'''
LiDAR driver for LDRobot LD06 and STL27L (Waveshare)

Sample Rates:
LD06:    4500 samples/s  (375 packages/s x 12 samples/package)
STL27L: 21600 samples/s (1800 packages/s x 12 samples/package)

Speed Control on Raspberry Pi
- RPi hardware PWM: https://pypi.org/project/rpi-hardware-pwm
- curve fitting using scipy.optimize.curve_fit
'''

import numpy as np
import serial
import asyncio
import time
import os
import pickle
import json # Import json for sending statistics

# running from project root

from lib.config import Config
from lib.pointcloud import save_raw_scan, get_scan_dict # Import save_raw_scan and get_scan_dict
from lib.platform_utils import init_serial  # init_serial_MCU, init_pwm_MCU


class Lidar:
    def __init__(self, config, send_stats_callback=None): # Added send_stats_callback

        self.verbose            = False
        self.sampling_rate      = config.get("LIDAR", config.DEVICE, "SAMPLING_RATE")
        self.raw_path           = config.raw_path

        self.z_angle            = None  # gets updated externally by A4988 driver

        # constants
        self.start_byte         = bytes([0x54])
        self.dlength_byte       = bytes([0x2c])
        self.dlength            = 12  # 12 samples per package
        self.package_len        = 47  # start_byte + dlength_byte + 44 byte payload, 1 byte CRC
        self.deg2rad            = np.pi / 180
        self.offset             = config.get("LIDAR", config.DEVICE, "OFFSET")
        self.crc_table          = config.crc_table

        # SERIAL
        # dmesg | grep "tty"
        self.port               = config.PORT

        # if self.platform in ['Pico', 'Pico W', 'Metro M7']:
        #     self.serial_connection  = init_serial_MCU(pin=self.port, baudrate=baudrate)
        # else:  # self.platform in ['Windows', 'Linux', 'RaspberryPi']:
        self.serial_connection  = init_serial(port=self.port, baudrate=config.get("LIDAR", config.DEVICE, "BAUDRATE"))


        self.byte_array         = bytearray()
        self.dtype              = np.float32

        self.out_len            = config.get("LIDAR", config.DEVICE, "OUT_LEN")

        # preallocate package:
        self.timestamp          = 0
        self.speed              = 0
        self.angle_package        = np.zeros(self.dlength)
        self.distance_package     = np.zeros(self.dlength)
        self.luminance_package    = np.zeros(self.dlength)
        # preallocate intermediate outputs:
        self.out_i              = 0
        self.speeds             = np.empty(self.out_len, dtype=self.dtype)
        self.timestamps         = np.empty(self.out_len, dtype=self.dtype)
        # Changed from points_2d to points_3d, now storing [x, y, z, luminance]
        self.points_3d          = np.empty((self.out_len * self.dlength, 4), dtype=self.dtype)  # [[x, y, z, l],[..

        self.z_angles           = []
        self.cartesian_list     = []

        # New attributes for scan control and data collection
        self.is_overall_scanning = False # Controls if the lidar reading loop is active (always true after start_lidar_loop)
        self.is_data_collection_active = False # Controls if data is actually appended to full_scan_data
        self.heading            = 0 # Current heading received from client
        self.full_scan_data     = [] # To store all collected data for one overall scan (until 'stop')

        # Statistics counters
        self.packets_gathered = 0
        self.packets_processed = 0
        self.points_in_buffer = 0
        self.last_stats_send_time = time.time() # For periodic statistics sending
        self.send_stats_callback = send_stats_callback # Callback to send stats to client

    def close(self):
        if hasattr(self, 'pwm') and self.pwm is not None: # Check if pwm attribute exists
            self.pwm.stop()
            print("PWM stopped.\n")

        self.serial_connection.close()
        print("Serial connection closed.\n")

    def prepare_for_scan(self):
        """Prepares the lidar for scanning (serial reading active), but doesn't start data collection."""
        print("Lidar: Preparing for scan (clearing previous data)...")
        self.is_overall_scanning = True # This means the read loop will process data
        self.is_data_collection_active = False # But data won't be collected yet
        self.full_scan_data = [] # Clear previous data for a new full scan
        # Reset statistics
        self.packets_gathered = 0
        self.packets_processed = 0
        self.points_in_buffer = 0
        self.last_stats_send_time = time.time() # Reset stats timer

    def start_data_collection(self):
        """Starts actual data collection (appending points to full_scan_data)."""
        print("Lidar: Starting data collection.")
        self.is_data_collection_active = True

    def stop_overall_scan(self):
        """Stops the overall data accumulation and saves all collected data."""
        print("Lidar: Stopping overall scan and saving data...")
        self.is_overall_scanning = False
        self.is_data_collection_active = False
        if self.full_scan_data:
            timestamp = int(time.time())
            # Ensure the raw_path directory exists
            output_dir = os.path.dirname(self.raw_path)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Create a scan dictionary and save it
            scan_dict = get_scan_dict(z_angles=[], cartesian_list=self.full_scan_data, scan_id=f"scan_{timestamp}", sensor="STL27L")
            # Save data to E57 format
            save_raw_scan(os.path.join(output_dir, f"scan_{timestamp}.e57"), scan_dict)
            print(f"Lidar: Data saved to scan_{timestamp}.e57")
        else:
            print("Lidar: No data collected to save.")


    def read_loop_websocket(self, send_fn, max_packages=None): # max_packages is now unused in loop condition
        loop_count = 0

        # Loop indefinitely as long as serial connection is open
        while self.serial_connection.is_open:
            try:
                # Reset self.out_i if it has reached the end of the preallocated buffer
                if self.out_i >= self.out_len:
                    self.out_i = 0

                # Always read to keep the serial buffer clear, regardless of data collection state
                self.read()

                # Only accumulate data if data collection is active
                if self.is_data_collection_active:
                    # Append current lidar data (now 3D points)
                    if self.points_3d.size > 0:
                        # Create a copy to avoid issues with points_3d being overwritten in next read
                        current_points_batch = np.copy(self.points_3d[self.out_i*self.dlength:(self.out_i+1)*self.dlength])
                        self.full_scan_data.extend(current_points_batch.tolist())
                        self.points_in_buffer = len(self.full_scan_data) # Update points in buffer count

                # Periodically send statistics
                if time.time() - self.last_stats_send_time >= 1.0: # Send stats every 1 second
                    stats = {
                        "type": "stats",
                        "packets_gathered": self.packets_gathered,
                        "packets_processed": self.packets_processed,
                        "points_in_buffer": self.points_in_buffer,
                        "is_collecting": self.is_data_collection_active
                    }
                    if self.send_stats_callback:
                        self.send_stats_callback(stats)
                    self.last_stats_send_time = time.time() # Reset stats timer

            except serial.SerialException:
                print("Lidar: SerialException")
                break
            except Exception as e:
                print(f"Lidar: Error in read_loop_websocket: {e}")
                break

            self.out_i += 1 # Prepare for the next iteration
            loop_count += 1


    def read(self):
        # iterate through serial stream until start package is found
        while self.serial_connection.is_open:
            data_byte = self.serial_connection.read()

            if data_byte == self.start_byte:
                # Check if the next byte is the second byte of the start sequence
                next_byte = self.serial_connection.read()
                if next_byte == self.dlength_byte:
                    # If it is, read the entire package
                    self.byte_array = self.serial_connection.read(self.package_len - 2)
                    self.byte_array = self.start_byte + self.dlength_byte + self.byte_array
                    self.packets_gathered += 1 # Increment gathered packets
                    break
                else:
                    # If it's not, discard the current byte and continue
                    continue

        # Error handling
        if len(self.byte_array) != self.package_len:
            if self.verbose:
                print("[WARNING] Incomplete package:", self.byte_array)
            self.byte_array = bytearray()
            return

        # Check if the package is valid using check_CRC8
        if not self.check_CRC8(self.byte_array):
            if self.verbose:
                print("[WARNING] Invalid package:", self.byte_array)
            # If the package is not valid, reset byte_array and continue with the next iteration
            self.byte_array = bytearray()
            return

        self.packets_processed += 1 # Increment processed packets

        # decoding updates speed, timestamp, angle_package, distance_package, luminance_package
        self.decode(self.byte_array)

        # Convert polar to local 2D cartesian (X_local, Z_local)
        # x_local_package: Represents the horizontal distance from the LiDAR in its own scanning plane.
        # z_local_package: Represents the vertical distance (height/depth) from the LiDAR's scanning plane.
        x_local_package, z_local_package = self.polar2cartesian(self.angle_package, self.distance_package, self.offset)

        # Convert local 2D (X_local, Z_local) to global 3D (X_global, Y_global, Z_global) using heading.
        # The 'heading' is interpreted as a yaw rotation around the global Z-axis.
        heading_rad = np.deg2rad(self.heading)

        # Global X-coordinate: Rotates the local horizontal distance (x_local_package) into the global X-axis.
        x_global_package = x_local_package * np.cos(heading_rad)
        # Global Y-coordinate: Rotates the local horizontal distance (x_local_package) into the global Y-axis.
        y_global_package = x_local_package * np.sin(heading_rad)
        # Global Z-coordinate: The vertical component remains unchanged by a yaw (heading) rotation.
        z_global_package = z_local_package

        # Combine into a 4-column package: [X_global, Y_global, Z_global, Luminance]
        points_package = np.column_stack((x_global_package, y_global_package, z_global_package, self.luminance_package)).astype(self.dtype)

        # write into preallocated output arrays at current index
        self.speeds[self.out_i] = self.speed
        self.timestamps[self.out_i] = self.timestamp
        # Assign to points_3d
        self.points_3d[self.out_i*self.dlength:(self.out_i+1)*self.dlength] = points_package

        # reset byte_array
        self.byte_array = bytearray()


    def decode(self, byte_array):
        # dlength = 12  # byte_array[46] & 0x1F
        self.speed = int.from_bytes(byte_array[2:4][::-1], 'big') / 360         # rotational frequency in rps
        FSA = float(int.from_bytes(byte_array[4:6][::-1], 'big')) / 100         # start angle in degrees
        LSA = float(int.from_bytes(byte_array[42:44][::-1], 'big')) / 100       # end angle in degrees


        self.timestamp = int.from_bytes(byte_array[44:46][::-1], 'big')         # timestamp in milliseconds < 30000
        # CS = int.from_bytes(byte_array[46:47][::-1], 'big')                   # CRC Checksum, checked even before decoding

        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (self.dlength-1)

        # 3 bytes per sample x 12 samples
        for counter, i in enumerate(range(0, 3 * self.dlength, 3)):
            self.angle_package[counter] = ((angleStep * counter + FSA) % 360) * self.deg2rad
            self.distance_package[counter] = int.from_bytes(byte_array[6 + i:8 + i][::-1], 'big')  # mm units
            self.luminance_package[counter] = byte_array[8 + i]


    @staticmethod
    def polar2cartesian(angles, distances, offset):

        angles = list(np.array(angles) + offset)
        x_list = distances * -np.cos(angles) # This will be our X (local)
        z_list = distances * np.sin(angles) # This will be our Z (local)
        return x_list, z_list

    def split_last_byte(self, data): # Added self as first argument
            return data[:-1], data[-1]

    def check_CRC8(self, data, crc=None):
        '''CRC check: length is 1 Byte, obtained from the verification of all the previous data except itself'''

        if crc is None:
            data, crc = self.split_last_byte(data)

        calculated_crc = 0
        for byte in data:
            if not 0 <= byte <= 255:
                raise ValueError(f"Invalid byte value: {byte}")
            calculated_crc = self.crc_table[(calculated_crc ^ byte) & 0xff]

        return calculated_crc == crc
