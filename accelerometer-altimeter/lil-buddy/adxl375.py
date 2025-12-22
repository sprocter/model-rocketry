"""A very simple driver for the ADXL375 High-G Accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C
from struct import unpack
from micropython import const
import time

ADXL375_ADDR = const(0x53)

_ADXL375_DEVID = const(0x00)
_ADXL375_BW_RATE = const(0x2C)
_ADXL375_POWER_CTL = const(0x2D)
_ADXL375_DATAX0 = const(0x32)

_SCALE_FACTOR = const(0.049)  # Datasheet pg 3, Tbl 1
_X_ERR = const(0.0793798)  # From print_accel_offsets() in utilities.py
_Y_ERR = const(-0.2010956)  # From print_accel_offsets() in utilities.py
_Z_ERR = const(-0.5096079)  # From print_accel_offsets() in utilities.py
_EXPECTED_DEVICE_ID = const(0xE5)  # Datasheet pg 21


class ADXL375:

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        self.buffer = bytearray(6)
        self.addr = ADXL375_ADDR

    def initialize(self) -> None:
        if ADXL375_ADDR not in self.i2c.scan():
            raise OSError(f"ADXL375 not found at {ADXL375_ADDR}")
        actual_device_id = unpack(
            "<B", self.i2c.readfrom_mem(ADXL375_ADDR, _ADXL375_DEVID, 1)
        )[0]
        if actual_device_id != _EXPECTED_DEVICE_ID:
            raise OSError(f"ADXL375 has incorrect device id {actual_device_id}")

        self.i2c.writeto_mem(ADXL375_ADDR, _ADXL375_POWER_CTL, b"\x08")
        self.i2c.writeto_mem(ADXL375_ADDR, _ADXL375_BW_RATE, b"\x08")
        time.sleep_ms(10)

    def read_raw(self) -> None:
        self.i2c.readfrom_mem_into(ADXL375_ADDR, _ADXL375_DATAX0, self.buffer)

    def decode_reading(self, reading: bytearray) -> tuple[float, float, float]:
        unpacked_reading = unpack("<hhh", reading)
        return (
            unpacked_reading[0] * _SCALE_FACTOR - _X_ERR,
            unpacked_reading[1] * _SCALE_FACTOR - _Y_ERR,
            unpacked_reading[2] * _SCALE_FACTOR - _Z_ERR,
        )
