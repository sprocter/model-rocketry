from machine import I2C
from struct import unpack
import time

# Accelerometer Registers #
_ACCEL_ADDR = const(0x68)  # Default ICM20649 I2C Addr: 104
_REG_BANK_SEL = const(0x7F)  # All banks
_USER_CTRL = const(0x03)  # Bank 0, pg 38
_LP_CONFIG = const(0x05)  # Bank 0, pg 39
_PWR_MGMT_1 = const(0x06)  # Bank 0, pg 39
_PWR_MGMT_2 = const(0x07)  # Bank 0, pg 40
_FIFO_EN_2 = const(0x67)  # Bank 0, pg 54
_FIFO_RST = const(0x68)  # Bank 0, pg 55
_FIFO_MODE = const(0x69)  # Bank 0, pg 55
_FIFO_COUNTH = const(0x70)  # Bank 0, pg 55
_FIFO_R_W = const(0x72)  # Bank 0, pg 56
_ODR_ALIGN_EN = const(0x09)  # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_1 = const(0x10)  # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_2 = const(0x11)  # Bank 2, pg 65
_ACCEL_CONFIG = const(0x14)  # Bank 2, pg 66
_ACCEL_CONFIG_2 = const(0x15)  # Bank 2, pg 67
_MOD_CTRL_USR = const(0x54)  # Bank 2, pg 70

_X_ERR = const(0.029050755)
_Y_ERR = -0.008103238  # Micropython constants can't be negative???
_Z_ERR = const(0.044206185)


class ICM20649:

    def __init__(self, resolution: int, i2c: I2C):
        # Resolution #
        if resolution == 1:
            # rate = 10, rate_hz = 1125/(1 + rate) = 1125/11 = 102.27Hz
            self._ACCEL_SAMPLERATE = b"\x0a"
            self._LOW_POWER = True
        elif resolution == 2:
            # rate = 2, rate_hz = 1125/(1 + rate) = 1125/3 = 375
            self._ACCEL_SAMPLERATE = b"\x02"
            self._LOW_POWER = False
        elif resolution == 3:
            # rate = 1, rate_hz = 1125/(1 + rate) = 1125/2 = 562.5
            self._ACCEL_SAMPLERATE = b"\x01"
            self._LOW_POWER = False

        # Initialize buffer to the size of the sensor's FIFO
        self.mv = memoryview(bytearray(4096))

        self.i2c = i2c

    # Call to get the device-specific calibration values needed to correct
    # accelerometer data
    def print_calib_values(self) -> None:
        xs, ys, zs = [], [], []
        reading = bytearray(6)
        print("Set the device face-up on a flat surface and hold it still.")
        print("Calibration begins in five seconds.")
        time.sleep(5)
        print("Calibration beginning now, it will take 10 seconds...")
        for _ in range(1000):
            self.i2c.readfrom_mem_into(_ACCEL_ADDR, 0x2D, reading)
            raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
            xs.append(raw_accel_x / 1024)
            ys.append(raw_accel_y / 1024)
            zs.append(raw_accel_z / 1024)
            time.sleep_ms(10)

        print("_X_ERR = ", sum(xs) / len(xs))
        print("_Y_ERR = ", sum(ys) / len(ys))
        print("_Z_ERR = ", sum(zs) / len(zs) - 1)

    def initialize_device(self) -> None:
        i2c = self.i2c
        low_power = self._LOW_POWER

        # Begin with Bank 0 configuration
        i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b"\x00")

        # Reset the device to default settings
        i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b"\x80")  # 0b10000000
        time.sleep_ms(5)
        while i2c.readfrom_mem(_ACCEL_ADDR, _PWR_MGMT_1, 1) == 128:
            time.sleep_ms(5)

        # Turn low power off, temperature sensor off, use best available clock
        # source
        i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b"\x09")  # 0b00001001
        time.sleep_ms(30)  # Let the accelerometer wake up out of sleep

        # Turn off the gyroscope, turn on the accelerometer
        i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_2, b"\x07")  # 0b00000111

        # Turn DMP Off, FIFO on, i2c master i/f module off, don't reset anything
        i2c.writeto_mem(_ACCEL_ADDR, _USER_CTRL, b"\x40")  # 0b01000000

        # Now on to Bank 2 configuration
        i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b"\x20")  # 0b00100000

        if low_power:
            # Enable ODR start-time alignment
            i2c.writeto_mem(_ACCEL_ADDR, _ODR_ALIGN_EN, b"\x01")

        # Sample rate: See Resolution section of Constants
        # Split across two bytes, 12 bits
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_1, b"\x00")
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_2, self._ACCEL_SAMPLERATE)

        # Low pass filter = 3, Accelerometer Full Scale 30g, DLPF Enabled
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG, b"\x3f")  # 0b00111111

        if low_power:
            # Average 4 samples (would be 1, but we're using the DLPF)
            i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG_2, b"\x00")  # 0b0000000

            # Disable DMP in LP Accelerometer mode
            i2c.writeto_mem(_ACCEL_ADDR, _MOD_CTRL_USR, b"\x02")

        # Back to Bank 0
        i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b"\x00")

        if low_power:
            # Operate accelerometer in duty cycled mode
            i2c.writeto_mem(_ACCEL_ADDR, _LP_CONFIG, b"\x20")  # 0b00100000
        else:
            # Operate accelerometer (and everything else) in low-noise mode
            i2c.writeto_mem(_ACCEL_ADDR, _LP_CONFIG, b"\x00")  # 0b00000000

        # Turn FIFO on for accelerometer data
        i2c.writeto_mem(_ACCEL_ADDR, _FIFO_EN_2, b"\x10")  # 0b00010000

        # Set FIFO to replace old data when full. Inshallah this won't happen
        i2c.writeto_mem(_ACCEL_ADDR, _FIFO_MODE, b"\x00")

        if low_power:
            # Reset FIFO, step 1: Assert (Set FIFO size to zero)
            i2c.writeto_mem(_ACCEL_ADDR, _FIFO_RST, b"\x01")
            # Reset FIFO, step 2: De-assert
            i2c.writeto_mem(_ACCEL_ADDR, _FIFO_RST, b"\x00")

            # Finally, set the LP_EN flag -- this prevents writing to most
            # registers, so disable for further configuration.
            i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b"\x29")  # 0b00101001

    @staticmethod
    def decode_reading(accel_reading: bytes) -> tuple[float, float, float]:
        raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", accel_reading)
        accel_x = (raw_accel_x / 1024) - _X_ERR
        accel_y = (raw_accel_y / 1024) - _Y_ERR
        accel_z = (raw_accel_z / 1024) - _Z_ERR
        return (accel_x, accel_y, accel_z)

    @micropython.native
    def read_fifo(self) -> int:
        fifo_count_bytes = bytearray(2)
        self.i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_COUNTH, fifo_count_bytes)
        fifo_count = (fifo_count_bytes[0] & 15) << 8 | fifo_count_bytes[1]
        # print("Accel FIFO: ", fifo_count)
        if fifo_count > 0:
            self.i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_R_W, self.mv[0:fifo_count])
        return fifo_count

    def shutdown(self) -> None:
        # Turn low power on and temperature sensor off, stop clock
        self.i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b"\x6f")  # 0b01101111
        # Turn off the gyroscope and accelerometer
        self.i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_2, b"\x3f")  # 0b00111111
