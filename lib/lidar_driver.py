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
import json
import threading
import websockets

# running from project root

from lib.config import Config
# from lib.pointcloud import save_raw_scan, get_scan_dict
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
        self.points_2d          = np.empty((self.out_len * self.dlength, 3), dtype=self.dtype)  # [[x, y, l],[..


        # self.data_dir           = config.lidar_dir  # TODO remove -> npy files replaced by single pkl file

        # raw output
        self.z_angles           = []
        self.cartesian_list     = []

    def close(self):
        if self.pwm is not None:
            self.pwm.stop()
            print("PWM stopped.\n")

        self.serial_connection.close()
        print("Serial connection closed.\n")


    def read_loop(self, callback=None, max_packages=None):
        loop_count = 0

        while self.serial_connection.is_open and (max_packages is None or loop_count <= max_packages):
            try:
                if self.out_i == self.out_len:
                    if callback is not None:
                        callback()

                    # save the z_angle to list
                    self.z_angles.append(self.z_angle)

                    if self.verbose:
                        print("speed:", round(self.speed, 2))
                        if self.z_angle is not None:
                            print("z_angle:", round(self.z_angle, 2))

                    # Append 2D plane to cartesian list. copying avoids identical pointers
                    self.cartesian_list.append(np.copy(self.points_2d))


                    self.out_i = 0

                self.read()


            except serial.SerialException:
                print("SerialException")
                break

            self.out_i += 1
            loop_count += 1


    def read_loop_websocket(self, send_fn, max_packages=None):
        loop_count = 0

        while self.serial_connection.is_open and (max_packages is None or loop_count <= max_packages):
            try:
                if self.out_i == self.out_len:
                    # send current scan via websocket
                    if self.cartesian_list:
                        points = self.cartesian_list[-1].tolist()
                        z_angle = self.z_angles[-1] if self.z_angles else 0
                        asyncio.run(send_fn(json.dumps({
                            'z_angle': z_angle,
                            'points': points
                        })))

                    self.z_angles.append(self.z_angle)

                    if self.verbose:
                        print("speed:", round(self.speed, 2))
                        if self.z_angle is not None:
                            print("z_angle:", round(self.z_angle, 2))

                    self.cartesian_list.append(np.copy(self.points_2d))
                    self.out_i = 0

                self.read()

            except serial.SerialException:
                print("SerialException")
                break

            self.out_i += 1
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

        # decoding updates speed, timestamp, angle_package, distance_package, luminance_package
        self.decode(self.byte_array)
        # convert polar to cartesian
        x_package, y_package = self.polar2cartesian(self.angle_package, self.distance_package, self.offset)
        points_package = np.column_stack((x_package, y_package, self.luminance_package)).astype(self.dtype)

        # write into preallocated output arrays at current index
        self.speeds[self.out_i] = self.speed
        self.timestamps[self.out_i] = self.timestamp
        self.points_2d[self.out_i*self.dlength:(self.out_i+1)*self.dlength] = points_package

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
        x_list = distances * -np.cos(angles)
        y_list = distances * np.sin(angles)
        return x_list, y_list



    def check_CRC8(self, data, crc=None):
        '''CRC check: length is 1 Byte, obtained from the verification of all the previous data except itself'''

        def split_last_byte(data):
            return data[:-1], data[-1]

        if crc is None:
            data, crc = split_last_byte(data)

        calculated_crc = 0
        for byte in data:
            if not 0 <= byte <= 255:
                raise ValueError(f"Invalid byte value: {byte}")
            calculated_crc = self.crc_table[(calculated_crc ^ byte) & 0xff]

        return calculated_crc == crc


async def lidar_websocket_handler(websocket, path):
    config = Config()
    config.set_device("STL27L")
    config.init(scan_id="_")

    lidar = Lidar(config)

    async def send_data(message):
        await websocket.send(message)

    thread = threading.Thread(target=lidar.read_loop_websocket, args=(send_data,))
    thread.start()

    try:
        async for _ in websocket:
            pass
    finally:
        lidar.close()

if __name__ == "__main__":
    start_server = websockets.serve(lidar_websocket_handler, "0.0.0.0", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    print("WebSocket server started on ws://0.0.0.0:8765")
    asyncio.get_event_loop().run_forever()
