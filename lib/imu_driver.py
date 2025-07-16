import time
import math
import numpy as np
import threading
from mpu6050 import MPU6050
import threading
import smbus

class BNO055Wrapper:
    def __init__(self, i2c_bus=3, device_address=0x29, freq=50):
        self.i2c_bus = i2c_bus
        self.device_address = device_address
        self.freq = freq  # Sampling frequency in Hz
        self.sleep_interval = 1.0 / freq  # Time between readings

        # BNO055 register addresses
        self.BNO055_CHIP_ID_ADDR = 0x00
        self.BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20
        self.BNO055_CALIB_STAT_ADDR = 0x35
        self.BNO055_OPR_MODE_ADDR = 0x3D
        self.BNO055_PWR_MODE_ADDR = 0x3E
        self.BNO055_SYS_TRIGGER_ADDR = 0x3F

        # Operation and power modes
        self.CONFIG_MODE = 0x00
        self.NDOF_MODE = 0x0C  # 9-DOF fusion mode
        self.POWER_MODE_NORMAL = 0x00

        # Initialize I2C bus
        self.bus = smbus.SMBus(self.i2c_bus)

        # Initialize sensor
        self._initialize_bno055()

        # Quaternion storage
        self.quat = None  # Stores w, x, y, z as floats
        self.calib_status = (0, 0, 0, 0)  # System, gyro, accel, mag calibration

        # Start background thread
        self.running = True
        self.thread = threading.Thread(target=self._read_bno055)
        self.thread.daemon = True
        self.thread.start()

    def _read_register(self, reg, length=1):
        """Read one or more bytes from a register."""
        try:
            return self.bus.read_i2c_block_data(self.device_address, reg, length)
        except OSError as e:
            print(f"I2C read error: {e}")
            return [0] * length

    def _write_register(self, reg, value):
        """Write a byte to a register."""
        try:
            self.bus.write_byte_data(self.device_address, reg, value)
        except OSError as e:
            print(f"I2C write error: {e}")

    def _initialize_bno055(self):
        """Initialize the BNO055 sensor."""
        # Check chip ID (should be 0xA0 for BNO055)
        chip_id = self._read_register(self.BNO055_CHIP_ID_ADDR)[0]
        if chip_id != 0xA0:
            raise RuntimeError(f"Expected BNO055 chip ID 0xA0, got 0x{chip_id:02X}")

        # Set power mode to normal
        self._write_register(self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)
        time.sleep(0.01)

        # Set to config mode
        self._write_register(self.BNO055_OPR_MODE_ADDR, self.CONFIG_MODE)
        time.sleep(0.02)

        # Set to NDOF mode (9-DOF fusion)
        self._write_register(self.BNO055_OPR_MODE_ADDR, self.NDOF_MODE)
        time.sleep(0.02)

        print("BNO055 initialized successfully")

    def _read_calibration_status(self):
        """Read calibration status for system, gyro, accel, and mag."""
        calib_stat = self._read_register(self.BNO055_CALIB_STAT_ADDR)[0]
        sys_calib = (calib_stat >> 6) & 0x03  # Bits 7-6
        gyro_calib = (calib_stat >> 4) & 0x03  # Bits 5-4
        accel_calib = (calib_stat >> 2) & 0x03  # Bits 3-2
        mag_calib = calib_stat & 0x03  # Bits 1-0
        return sys_calib, gyro_calib, accel_calib, mag_calib

    def _read_quaternion(self):
        """Read quaternion data (w, x, y, z)."""
        # Read 8 bytes (w, x, y, z; each is 2 bytes)
        data = self._read_register(self.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)

        # Combine LSB and MSB for each component (16-bit signed integers)
        quat_w = (data[1] << 8) | data[0]
        quat_x = (data[3] << 8) | data[2]
        quat_y = (data[5] << 8) | data[4]
        quat_z = (data[7] << 8) | data[6]

        # Convert to signed integers
        if quat_w > 32767:
            quat_w -= 65536
        if quat_x > 32767:
            quat_x -= 65536
        if quat_y > 32767:
            quat_y -= 65536
        if quat_z > 32767:
            quat_z -= 65536

        # Convert to float (1 LSB = 1/(2^14) in quaternion mode)
        scale = 1.0 / (1 << 14)  # 1/16384
        return np.array([quat_w * scale, quat_x * scale, quat_y * scale, quat_z * scale])

    def _read_bno055(self):
        """Background thread to continuously read quaternion and calibration data."""
        while self.running:
            try:
                # Read calibration status
                self.calib_status = self._read_calibration_status()

                # Read quaternion data
                quat = self._read_quaternion()

                # Check for invalid data
                if not any(math.isnan(value) for value in quat):
                    self.quat = quat
                time.sleep(self.sleep_interval)
            except Exception as e:
                print(f"Error in read thread: {e}")
                time.sleep(0.01)

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
        if self.quat is None:
            return None

        w, x, y, z = self.quat

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Handle 90 degrees
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return type('Euler', (), {'x': roll, 'y': pitch, 'z': yaw})()

    def get_quat_values(self):
        """Return quaternion values as a numpy array [w, x, y, z]."""
        return self.quat if self.quat is not None else np.array([0.0, 0.0, 0.0, 0.0])

    def get_calibration_status(self):
        """Return calibration status (system, gyro, accel, mag)."""
        return self.calib_status

    def close(self):
        """Stop the background thread and clean up."""
        self.running = False
        self.thread.join()
        # Set sensor to config mode
        self._write_register(self.BNO055_OPR_MODE_ADDR, self.CONFIG_MODE)
        self.bus.close()
        print("BNO055 closed")

if __name__ == '__main__':
    from config import Config  # Assuming similar config module

    config = Config()
    config.init()

    imu = BNO055Wrapper(
        i2c_bus=config.get("IMU", "i2c_bus"),
        device_address=config.get("IMU", "device_address"),
        freq=config.get("IMU", "frequency")
    )

    try:
        time.sleep(0.5)  # Wait for initial readings
        print("Calibrating BNO055... Move in figure-8 for mag, hold still for gyro/accel.")
        while True:
            sys_calib, gyro_calib, accel_calib, mag_calib = imu.get_calibration_status()
            print(f"\rCalibration: Sys={sys_calib}/3 Gyro={gyro_calib}/3 Accel={accel_calib}/3 Mag={mag_calib}/3", end='')

            quat = imu.get_quat_values()
            print(f" Quaternion (wxyz): {quat}", end='')

            euler = imu.get_euler_angles()
            if euler:
                print(f" Euler (rad): x={euler.x:.2f} y={euler.y:.2f} z={euler.z:.2f}", end='')

            time.sleep(0.01)
    except KeyboardInterrupt:
        imu.close()
        print("\nStopped.")


class MPU6050Wrapper:
    def __init__(self, i2c_bus=3, device_address=0x68, freq=50):
        self.i2c_bus = i2c_bus
        self.device_address = device_address
        self.freq_divider = int(200 / freq)  # 0x04

        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.freq_divider)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_buffer = [0]*64

        self.quat = None  # current Quaternion object

        self.running = True
        self.thread = threading.Thread(target=self._read_mpu6050)
        self.thread.daemon = True
        self.thread.start()

    def _read_mpu6050(self):
        while self.running:
            try:
                if self.mpu.isreadyFIFO(self.packet_size):
                    self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                    quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)

                    # Check if any value is NaN
                    if not any(math.isnan(value) for value in [quat.w, quat.x, quat.y, quat.z]):
                        self.quat = quat
                        time.sleep(0.02)
                    else:
                        time.sleep(0.01)
            except OSError as e:
                time.sleep(0.01)

    def get_euler_angles(self):
        return self.mpu.DMP_get_euler_roll_pitch_yaw(self.quat)

    def get_quat_values(self):
        return np.array([self.quat.w, self.quat.x, self.quat.y, self.quat.z])

    def close(self):
        self.running = False


if __name__ == '__main__':
    from config import Config # , format_value

    config = Config()
    config.init()

    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"), config.get("IMU", "frequency"))

    try:
        time.sleep(0.5)  # wait for FIFO buffer to fill
        while True:
            quat_float = imu.get_quat_values() / 16384
            print(f'\r Quaternion (wxyz): { quat_float}', end='')

            # euler = imu.get_euler_angles()
            # print(f'\r Euler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}', end='')
            time.sleep(0.01)

    except KeyboardInterrupt:
        imu.close()
        print("\nStopped.")
