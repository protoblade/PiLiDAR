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

# running from project root

from lib.config import Config
from lib.pointcloud import save_raw_scan, get_scan_dict # Import save_raw_scan and get_scan_dict
from lib.platform_utils import init_serial  # init_serial_MCU, init_pwm_MCU


class Lidar:
    def __init__(self, config):

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
        self.is_scanning        = False
        self.heading            = 0 # Current heading received from client
        self.scan_start_time    = 0
        self.full_scan_data     = [] # To store all collected data for one rotation

    def close(self):
        if hasattr(self, 'pwm') and self.pwm is not None: # Check if pwm attribute exists
            self.pwm.stop()
            print("PWM stopped.\n")

        self.serial_connection.close()
        print("Serial connection closed.\n")

    def start_scan(self):
        """Starts the data collection for a new rotation."""
        print("Starting scan...")
        self.is_scanning = True
        self.full_scan_data = [] # Clear previous data
        self.scan_start_time = time.time()

    def stop_scan(self):
        """Stops the data collection and saves the collected data."""
        print("Stopping scan and saving data...")
        self.is_scanning = False
        if self.full_scan_data:
            timestamp = int(time.time())
            # Ensure the raw_path directory exists
            output_dir = os.path.dirname(self.raw_path)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Create a scan dictionary and save it
            # cartesian_list now contains [x, y, z, luminance] directly
            scan_dict = get_scan_dict(z_angles=[], cartesian_list=self.full_scan_data, scan_id=f"scan_{timestamp}", sensor="STL27L")
            save_raw_scan(os.path.join(output_dir, f"scan_{timestamp}.pkl"), scan_dict)
            print(f"Data saved to scan_{timestamp}.pkl")
        else:
            print("No data collected to save.")


    def read_loop_websocket(self, send_fn, max_packages=None):
        loop_count = 0

        while self.serial_connection.is_open and (max_packages is None or loop_count <= max_packages):
            try:
                # Reset self.out_i if it has reached the end of the preallocated buffer
                if self.out_i >= self.out_len:
                    self.out_i = 0

                self.read() # Always read to keep the serial buffer clear

                if self.is_scanning:
                    # Append current lidar data (now 3D points) with heading
                    if self.points_3d.size > 0:
                        # Create a copy to avoid issues with points_3d being overwritten in next read
                        current_points_batch = np.copy(self.points_3d[self.out_i*self.dlength:(self.out_i+1)*self.dlength])

                        # full_scan_data now directly stores the [x, y, z, luminance] points
                        self.full_scan_data.extend(current_points_batch.tolist())

                    # Check if 1 second of data has been collected
                    if (time.time() - self.scan_start_time) >= 1.0:
                        print(f"1 second of data collected for heading {self.heading}. Sending 'done'.")
                        asyncio.run(send_fn(f"{self.heading} done")) # Send heading + "done"
                        self.scan_start_time = time.time() # RESET START TIME FOR NEXT SEGMENT

            except serial.SerialException:
                print("SerialException")
                break
            except Exception as e:
                print(f"Error in read_loop_websocket: {e}")
                break

            self.out_i += 1 # Prepare for the next iteration
            loop_count += 1


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
