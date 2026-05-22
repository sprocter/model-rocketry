"""A very simple driver for the ADXL375 High-G Accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025-2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C
from struct import unpack
from micropython import const
import time

G_TO_MS2 = const(9.80665)  # https://en.wikipedia.org/wiki/Standard_gravity

_ADXL375_DEVID = const(0x00)  # Datasheet pg 21
_ADXL375_BW_RATE = const(0x2C)  # Datasheet pg 22
_ADXL375_POWER_CTL = const(0x2D)  # Datasheet pg 22
_ADXL375_DATAX0 = const(0x32)  # Datasheet pg 24

_SCALE_FACTOR = const(0.049)  # Datasheet pg 3, Tbl 1
_ACCEL_ADJUST = const(G_TO_MS2 * _SCALE_FACTOR)

# _X_ERR = const(0.0793798)  # From print_accel_offsets() in utilities.py
# _Y_ERR = const(-0.2010956)  # From print_accel_offsets() in utilities.py
# _Z_ERR = const(-0.5096079)  # From print_accel_offsets() in utilities.py
_EXPECTED_DEVICE_ID = const(0xE5)  # Datasheet pg 21


class ADXL375:

    ADDR = const(0x53)

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        self.x_err = 0.0
        self.y_err = 0.0
        self.z_err = 0.0
        self.buffer = bytearray(6)

    def initialize(self, offsets: dict) -> None:
        if ADDR not in self.i2c.scan():
            raise OSError(f"ADXL375 not found at {ADDR}")
        actual_device_id = unpack("<B", self.i2c.readfrom_mem(ADDR, _ADXL375_DEVID, 1))[
            0
        ]
        if actual_device_id != _EXPECTED_DEVICE_ID:
            raise OSError(f"ADXL375 has incorrect device id {actual_device_id}")

        self.i2c.writeto_mem(ADDR, _ADXL375_POWER_CTL, b"\x08")
        self.i2c.writeto_mem(ADDR, _ADXL375_BW_RATE, b"\x0a")
        time.sleep_ms(10)

        self.x_err = offsets["ACC_X_ERR"]
        self.y_err = offsets["ACC_Y_ERR"]
        self.z_err = offsets["ACC_Z_ERR"]

    def read_raw(self) -> None:
        self.i2c.readfrom_mem_into(ADDR, _ADXL375_DATAX0, self.buffer)

    @micropython.native
    def decode_accel(self, reading: bytearray) -> tuple[float, float, float]:
        unpacked_reading = unpack("<hhh", reading)
        return (
            unpacked_reading[0] * _ACCEL_ADJUST - self.x_err,
            unpacked_reading[1] * _ACCEL_ADJUST - self.y_err,
            unpacked_reading[2] * _ACCEL_ADJUST - self.z_err,
        )

    @property
    def error(self) -> float:
        """
        Return the standard deviation of the sensor's error in meters per second per second

        :returns: The standard deviation of the decoded readings' error
        :rtype: float
        """
        return 5.0
