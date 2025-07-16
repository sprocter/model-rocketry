"""A driver for a model rocket accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------

This script contains initialization and usage functions for the InvenSense  ICM20649: https://www.adafruit.com/product/4464

To get the device-specific calibration values, call print_calib_values() after the driver has been initialized.

Note that none of this code is designed to be modified by the user.
"""

from machine import I2C
from struct import unpack
import time

MIN_RESOLUTION = const(1)
MAX_RESOLUTION = const(3)
_MAX_FIFO_COUNT = 1692

# Accelerometer Registers #
_ACCEL_ADDR = const(0x68)  # Default ICM20649 I2C Addr: 104
_REG_BANK_SEL = const(0x7F)  # All banks
_USER_CTRL = const(0x03)  # Bank 0, pg 38
_LP_CONFIG = const(0x05)  # Bank 0, pg 39
_PWR_MGMT_1 = const(0x06)  # Bank 0, pg 39
_PWR_MGMT_2 = const(0x07)  # Bank 0, pg 40
_INT_PIN_CFG = const(0x0F)  # Bank 0, pg 40
_INT_ENABLE = const(0x10)  # Bank 0, pg 41
_FIFO_EN_2 = const(0x67)  # Bank 0, pg 54
_FIFO_RST = const(0x68)  # Bank 0, pg 55
_FIFO_MODE = const(0x69)  # Bank 0, pg 55
_FIFO_COUNTH = const(0x70)  # Bank 0, pg 55
_FIFO_R_W = const(0x72)  # Bank 0, pg 56
_ODR_ALIGN_EN = const(0x09)  # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_1 = const(0x10)  # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_2 = const(0x11)  # Bank 2, pg 65
_ACCEL_INTEL_CTRL = const(0x12)  # Bank 2, pg 65
_ACCEL_WOM_THR = const(0x13)  # Bank 2, pg 66
_ACCEL_CONFIG = const(0x14)  # Bank 2, pg 66
_ACCEL_CONFIG_2 = const(0x15)  # Bank 2, pg 67
_MOD_CTRL_USR = const(0x54)  # Bank 2, pg 70

_X_ERR = const(0.029050755)
_Y_ERR = -0.008103238  # Micropython constants can't be negative???
_Z_ERR = const(0.044206185)


class ICM20649:

    def __init__(self, resolution: int, i2c: I2C):
        """Initializes the driver.

        This method initializes globals according to the user's desired resolution: higher values mean more data readings -- thus more power consumption and more disk usage.

        :param int resolution: The amount of data to record, should be in the range [ICM20649.MIN_RESOLUTION .. ICM20649.MAX_RESOLUTION] (inclusive). Values less than ICM20649.MIN_RESOLUTION will be interpreted as ICM20649.MIN_RESOLUTION, values greater than ICM20649.MAX_RESOLUTION will be interpreted as ICM20649.MAX_RESOLUTION.
        :param I2C i2c: An initialized / connected I2C object allowing communication with the device.
        """

        if resolution < MIN_RESOLUTION:
            resolution = MIN_RESOLUTION
        elif resolution > MAX_RESOLUTION:
            resolution = MAX_RESOLUTION

        # Resolution #
        if resolution == 1:
            # rate = 10, rate_hz = 1125/(1 + rate) = 1125/11 = 102.27Hz
            self._LOW_POWER = True
        elif (resolution == 2) or (resolution == 3):
            self._LOW_POWER = False

        # Initialize buffer to the size of the sensor's FIFO
        self.mv = memoryview(bytearray(4096))
        self.i2c = i2c

    # Call to get the device-specific calibration values needed to correct
    # accelerometer data
    def print_calib_values(self) -> None:
        """Prints the device-specific calibration values needed to decode the data

        This will print the device's calibration values to the terminal. These values will be three floats (values for the X, Y, and Z axes) and associated assignment statements. All three statements should be copied and pasted into the sibling python script (which should be located in ../tera).

        This only needs to be done once when you have a new device.
        """
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
        """This initializes the device when power is initially connected.

        This should be called when the device first receives power -- it will leave the device running and recording data at a rate determined by the user-selected resolution value.
        """

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

        # Set interrupt pin 1 logic level high, push-pull, clear after 50µs,
        # clear INT_STATUS on any read operation
        i2c.writeto_mem(_ACCEL_ADDR, _INT_PIN_CFG, b"\x10")  # 0b00010000

        # Enable interrupt for wake on motion to propagate to interrupt pin 1
        i2c.writeto_mem(_ACCEL_ADDR, _INT_ENABLE, b"\x08")  # 0b00001000

        # Turn DMP Off, FIFO on, i2c master i/f module off, don't reset anything
        i2c.writeto_mem(_ACCEL_ADDR, _USER_CTRL, b"\x40")  # 0b01000000

        # Now on to Bank 2 configuration
        i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b"\x20")  # 0b00100000

        if low_power:
            # Enable ODR start-time alignment
            i2c.writeto_mem(_ACCEL_ADDR, _ODR_ALIGN_EN, b"\x01")

        # Sample rate: rate = 10, rate_hz = 1125/(1 + rate) = 1125/11 = 102.27Hz
        # Split across two bytes, 12 bits
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_1, b"\x00")
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_2, b"\x0a")

        # Enable wake on motion logic, compare samples with original sample
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_INTEL_CTRL, b"\x02")  # 0b00000010

        # Wake on motion threshold. Threshold is value * 4 mg
        # But I'm pretty sure mg is milligravity?
        # Threshold is 1020mg, or an additional gravity. # 0b11111111
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_WOM_THR, b"\xff")

        # Low pass filter = 3, Accelerometer Full Scale 30g, DLPF Enabled
        i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG, b"\x3f")  # 0b00111111

        if low_power:
            # Average 4 samples (would be 1, but we're using the DLPF)
            i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG_2, b"\x00")  # 0b0000000

            # Disable DMP in LP Accelerometer mode
            i2c.writeto_mem(_ACCEL_ADDR, _MOD_CTRL_USR, b"\x02")
        else:
            # Average 32 samples
            i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG_2, b"\x03")  # 0b0000011

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

        # Set FIFO to replace old data when full.
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
        """This converts a raw reading into a triple of acceleration values

        This method unpacks the reading, scales each axis, reduces each axis by the associated error, then returns all three values

        :param bytes accel_reading: A single reading from the accelerometer. It should be a bytes object of length six (two bytes per axis)
        :return: A triple of X acceleration, y acceleration, and z acceleration
        :rtype: tuple[float, float, float]
        """
        raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", accel_reading)
        accel_x = (raw_accel_x / 1024) - _X_ERR
        accel_y = (raw_accel_y / 1024) - _Y_ERR
        accel_z = (raw_accel_z / 1024) - _Z_ERR
        return (accel_x, accel_y, accel_z)

    @micropython.native
    def read_fifo(self) -> int:
        """Read from the device's FIFO buffer.

        This checks how many bytes are available to read from the device,  reads those bytes into memory, then returns the number of bytes read so they can be written to disk.

        :return: The number of bytes read from the FIFO
        :rtype: int
        """
        fifo_count_bytes = bytearray(2)
        self.i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_COUNTH, fifo_count_bytes)

        # There seems to be an off-by-one error in the ICM20649: When the
        # buffer is full, FIFO_COUNTH is 0b00010000 but the datasheet says it
        # should be 0b00001111. So instead of &ing it with 15, we & it with 31
        # fifo_count = (fifo_count_bytes[0] & 15) << 8 | fifo_count_bytes[1]
        fifo_count = (fifo_count_bytes[0] & 31) << 8 | fifo_count_bytes[1]
        # print("Accel FIFO: ", fifo_count)
        if fifo_count > 0:
            self.i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_R_W, self.mv[0:fifo_count])
            # The ICM20649 FIFO holds a longer period of readings than the
            # BMP390, so we need to discard the excess. We read all of the
            # data (to clear the FIFO) but change this to the correct number of
            # readings to write
            if fifo_count > self._MAX_FIFO_COUNT:
                fifo_count = self._MAX_FIFO_COUNT
        return fifo_count

    def shutdown(self) -> None:
        """Shuts the device off. Call to save power."""
        # Turn low power on and temperature sensor off, stop clock
        self.i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b"\x6f")  # 0b01101111
        # Turn off the gyroscope and accelerometer
        self.i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_2, b"\x3f")  # 0b00111111
