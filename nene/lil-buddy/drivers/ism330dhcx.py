"""A very simple driver for the ISM330DHCX Accelerometer and Gyroscope

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C
from struct import unpack
import time

_REG_WHO_AM_I = const(0x0F)  # Datasheet pg 48
_REG_CTRL1_XL = const(0x10)  # Datasheet pg 49
_REG_CTRL2_G = const(0x11)  # Datasheet pg 50
_REG_CTRL3_C = const(0x12)  # Datasheet pg 51
_REG_CTRL9_XL = const(0x18)  # Datasheet pg 58
_REG_OUT_TEMP_L = const(0x20)  # Datasheet pg 64

_EXPECTED_DEVICE_ID = const(0x6B)  # Datasheet pg 48

_ACC_X_ERR = const(-0.058289064)
_ACC_Y_ERR = const(0.06798006)
_ACC_Z_ERR = const(0.1574564)
_GYRO_X_ERR = const(-0.3548993)
_GYRO_Y_ERR = const(-0.61809904)
_GYRO_Z_ERR = const(0.13999982)

_G_TO_MS2 = const(9.80665)  # https://en.wikipedia.org/wiki/Standard_gravity
_TEMP_ADJUST = const(256)  # Datasheet page 13
_TEMP_OFFSET = const(25)  # Datasheet page 13
_GYRO_ADJUST = const(0.140)  # Datasheet pg 10
_ACCEL_ADJUST = const(0.000488 * _G_TO_MS2)  # Datasheet pg 10 #


class ISM330DHCX:

    ADDR = const(0x6B)

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        self.buffer = bytearray(14)

    def initialize(self) -> None:
        if ADDR not in self.i2c.scan():
            raise OSError(f"ISM330DHCX not found at {ADDR}")
        actual_device_id = unpack(
            "<B", self.i2c.readfrom_mem(ADDR, _REG_WHO_AM_I, 1)
        )[0]
        if actual_device_id != _EXPECTED_DEVICE_ID:
            raise OSError(f"ISM330DHCX has incorrect device id {actual_device_id}")

        # Reset the device using CTRL3_C
        # 0 0 0 0 0 0 0 1
        self.i2c.writeto_mem(ADDR, _REG_CTRL3_C, b"\x01")
        time.sleep_ms(100)  # Give the device time to do its reset

        # Enable "proper device configuration" 🤨 using CTRL9_XL
        # 1 1 1 0 0 0 1 0
        self.i2c.writeto_mem(ADDR, _REG_CTRL9_XL, b"\xe2")

        # Enable "Block Data Update" so we don't read partially-updated data
        # 0 0 0 0 0 1 0 0
        self.i2c.writeto_mem(ADDR, _REG_CTRL3_C, b"\x04")

        # Configure accelerometer using CTRL1_XL:
        #   1. ODR to 52Hz 0011
        #   2. Full-scale to 16g 01
        # 0 0
        self.i2c.writeto_mem(ADDR, _REG_CTRL1_XL, b"\x34")

        # Configure gyroscope using CTRL2_G:
        #   1. ODR to 52 Hz 0011
        #   2. Fullscale to 4000dps 0001
        self.i2c.writeto_mem(ADDR, _REG_CTRL2_G, b"\x31")
        time.sleep_ms(100)  # Allow everything to wake up

    def read_raw(self) -> None:
        self.i2c.readfrom_mem_into(ADDR, _REG_OUT_TEMP_L, self.buffer)

    @micropython.native
    def decode_accel(self, reading: bytearray) -> tuple[float, float, float]:
        unpacked_reading = unpack("<hhhhhhh", reading)
        return (
            unpacked_reading[4] * _ACCEL_ADJUST - _ACC_X_ERR,
            unpacked_reading[5] * _ACCEL_ADJUST - _ACC_Y_ERR,
            unpacked_reading[6] * _ACCEL_ADJUST - _ACC_Z_ERR,
        )

    @micropython.native
    def decode_gyro(self, reading: bytearray) -> tuple[float, float, float]:
        unpacked_reading = unpack("<hhhhhhh", reading)
        return (
            unpacked_reading[1] * _GYRO_ADJUST - _GYRO_X_ERR,
            unpacked_reading[2] * _GYRO_ADJUST - _GYRO_Y_ERR,
            unpacked_reading[3] * _GYRO_ADJUST - _GYRO_Z_ERR,
        )

    @micropython.native
    def decode_temp(self, reading: bytearray) -> float:
        unpacked_reading = unpack("<hhhhhhh", reading)
        return unpacked_reading[0] / _TEMP_ADJUST + _TEMP_OFFSET

    @property
    def error(self) -> float:
        """
        Return the standard deviation of the sensor's error in meters per second per second

        :returns: The standard deviation of the decoded readings' error
        :rtype: float
        """
        return 1.05