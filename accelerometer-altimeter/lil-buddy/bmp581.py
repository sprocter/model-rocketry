"""A very simple driver for the BMP581 Barometric Pressure Sensor

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

_BMP581_ADDR = const(0x47)

# Page and section numbers refer to the datasheet

_BMP581_CHIPID = const(0x01)  # pg 51, 7.1
_BMP581_PRESS_XLSB = const(0x20)  # pg 56, 7.14
_BMP581_DSP_IIR_CFG = const(0x31)  # pg 62, 7.28
_BMP581_OSR_CONFIG = const(0x36)  # pg 64, 7.32
_BMP581_ODR_CONFIG = const(0x37)  # pg 64-65, 7.33

_BAROMETRIC_PRESSURE = const(1013.25) # https://en.wikipedia.org/wiki/Standard_atmosphere_(unit)
_EXPECTED_DEVICE_ID = const(80)  # pg 51, 7.1


class BMP581:

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        self.buffer = bytearray(3)
        self.addr = _BMP581_ADDR

    def initialize(self) -> None:
        if _BMP581_ADDR not in self.i2c.scan():
            raise OSError(f"BMP581 not found at {_BMP581_ADDR}")
        actual_device_id = unpack(
            "<B", self.i2c.readfrom_mem(_BMP581_ADDR, _BMP581_CHIPID, 1)
        )[0]
        if actual_device_id != _EXPECTED_DEVICE_ID:
            raise OSError(f"ADXL375 has incorrect device id {actual_device_id}")

        # is "non-stop" the same as continuous? I think so?
        # disable deep standby, ODR = 25.005hz, "non-stop" mode.
        # 1 10100 11
        self.i2c.writeto_mem(_BMP581_ADDR, _BMP581_ODR_CONFIG, b"\xd3")
        time.sleep_ms(10)

        # reserved, pressure iir = 2 / coeff 3, do not use temp iir filter
        # 00 010 000
        self.i2c.writeto_mem(_BMP581_ADDR, _BMP581_DSP_IIR_CFG, b"\x10")

        # Reserved, enable pressure readings, pressure osr = 32x, temp osr = 2x
        # 0 1 101 001
        self.i2c.writeto_mem(_BMP581_ADDR, _BMP581_OSR_CONFIG, b"\x69")
        time.sleep_ms(10)

    def read_raw(self) -> None:
        self.i2c.readfrom_mem_into(_BMP581_ADDR, _BMP581_PRESS_XLSB, self.buffer)

    def decode_reading(self, reading: bytearray) -> float:
        pressure_pa = (reading[2] << 16 | reading[1] << 8 | reading[0]) >> 6
        alti_m = (
            1 - (((pressure_pa / 100) / _BAROMETRIC_PRESSURE) ** 0.190284)
        ) * 44307.69396 # formula from https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
        return alti_m

    @property
    def error(self) -> float:
        """
        Return the standard deviation of the sensor's error in meters

        :returns: The standard deviation of the decoded readings' error
        :rtype: float
        """
        return 1.0
