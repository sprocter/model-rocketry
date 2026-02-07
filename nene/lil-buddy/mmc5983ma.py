"""A very simple driver for the MMC5983MA Magnetometer

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

_MMC5983MA_ADDR = const(0x30)

# Page and section numbers refer to the datasheet

_MMC5983MA_CHIPID = const(0x2F)  # pg 16, Product ID1
_MMC5983MA_XOUT0 = const(0x00)  # pg 13, Register Map
_MMC5983MA_CTRL0 = const(0x09)  # pg 14
_MMC5983MA_CTRL1 = const(0x0A)  # pg 15
_MMC5983MA_CTRL2 = const(0x0B)  # pg 15

_EXPECTED_DEVICE_ID = const(48)  # pg 16, Product ID1


class MMC5983MA:

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        # Mag values are three 18 bit floats split across 7 bytes 😑
        self.buffer = bytearray(7)  
        self.addr = _MMC5983MA_ADDR

        # "Local" offsets, set using the chip's SET and RESET functionality
        self._x_offset = 0
        self._y_offset = 0
        self._z_offset = 0

        # "Hard Iron" Offsets, set using the calibration function
        self._x_hi_offset = 0
        self._y_hi_offset = 0
        self._z_hi_offset = 0

        # "Soft Iron" Offsets, set using the calibration function
        self._x_si_offset = 1
        self._y_si_offset = 1
        self._z_si_offset = 1

    def _calculate_offsets(self) -> None:
        """Calculate the device's local offsets and store them

        This implements the procedure (described on pg 18 of the datasheet) for calculating the three axes' offsets. It modifies the _x|y|z_offset variables, which are used in the decode_mag function.
        """
        # "Set" operation
        # 0 0 0 0 1 0 0 0
        self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL0, b"\x08")
        time.sleep_us(1)  # Datasheet says 500ns, we'll give a little extra

        # Trigger reading
        self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL0, b"\x01")
        time.sleep_ms(10)
        self.read_raw()
        (x1, y1, z1) = self.decode_mag(self.buffer)

        # "Reset" operation
        # 0 0 0 1 0 0 0 0
        self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL0, b"\x10")
        time.sleep_us(1)  # Datasheet says 500ns, we'll give a little extra

        # Trigger reading
        self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL0, b"\x01")
        time.sleep_ms(10)
        self.read_raw()
        (x2, y2, z2) = self.decode_mag(self.buffer)

        self._x_offset = (x1 + x2) / 2
        self._y_offset = (y1 + y2) / 2
        self._z_offset = (z1 + z2) / 2

    def initialize(self) -> None:
        if _MMC5983MA_ADDR not in self.i2c.scan():
            raise OSError(f"MMC5983MA not found at {_MMC5983MA_ADDR}")
        actual_device_id = unpack(
            "<B", self.i2c.readfrom_mem(_MMC5983MA_ADDR, _MMC5983MA_CHIPID, 1)
        )[0]
        if actual_device_id != _EXPECTED_DEVICE_ID:
            raise OSError(f"MMC5983MA has incorrect device id {actual_device_id}")

        # Reset the chip, take measurements for 8ms (100Hz B/W)
        # 1 00 00 0 0 0
        self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL1, b"\x80")
        # Datasheet says power on time is 10ms, we'll give a little extra
        time.sleep_ms(20)

        self._calculate_offsets()

        # Enable Auto Set/Reset
        # 0 0 1 0 0 0 0 0
        # This is a neat feature but it 1) flips the signs from what I get from
        # a reference implementation and 2) requires re-establishing the
        # offsets. So I don't know how to use it in my time-constrained setup
        # self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL0, b"\x20")

        # Enable periodic settings, set every time*, enable continuous mode, Continuous frequency = 50hz
        # * Since we're not using auto set / reset, I don't believe this affects anything.
        # 1 000 1 100
        self.i2c.writeto_mem(_MMC5983MA_ADDR, _MMC5983MA_CTRL2, b"\x8c")

        time.sleep_ms(10)  # Block until first reading can be taken, they take 8ms

    def calibrate(self, duration=10):
        """Attempt to calculate hard-iron and soft-iron distortion in order to compensate for it.

        This takes samples over the user-specified number of seconds (default 10) and then does some math to figure out hard and soft iron distortion. Offsets / scale factors to compensate for that distortion are then stored in class variables and used to adjust future readings.

        Note that the soft iron distortion is a "good enough" implementation that does not do an actual ellipsoid fit.

        :param int duration: How long to run the calibration for in seconds

        """
        start_ts = time.ticks_ms()

        x_min = y_min = z_min = 999999999
        x_max = y_max = z_max = -999999999

        while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
            self.read_raw()
            (x, y, z) = self.decode_mag(self.buffer)
            if x < x_min:
                x_min = x
            elif x > x_max:
                x_max = x
            if y < y_min:
                y_min = y
            elif y > y_max:
                y_max = y
            if z < z_min:
                z_min = z
            elif z > z_max:
                z_max = z
            time.sleep_ms(20)

        # "Hard iron" distortion
        self._x_hi_offset = (x_min + x_max) / 2
        self._y_hi_offset = (y_min + y_max) / 2
        self._z_hi_offset = (z_min + z_max) / 2

        # "Soft iron" distortion -- uses scale biases, would be more accurate with an ellipsoid fit. See https://www.appelsiini.net/2018/calibrate-magnetometer/
        avg_hi_offset = (self._x_hi_offset + self._y_hi_offset + self._z_hi_offset) / 3
        self._x_si_offset = avg_hi_offset / self._x_hi_offset
        self._y_si_offset = avg_hi_offset / self._y_hi_offset
        self._z_si_offset = avg_hi_offset / self._z_hi_offset

    def read_raw(self) -> None:
        self.i2c.readfrom_mem_into(_MMC5983MA_ADDR, _MMC5983MA_XOUT0, self.buffer)

    @micropython.native
    def decode_mag(self, reading: bytearray) -> tuple[float, float, float]:
        x_raw = reading[0] << 10 | reading[1] << 2 | reading[6] >> 6
        y_raw = reading[2] << 10 | reading[3] << 2 | ((reading[6] >> 4) & 0x03)
        z_raw = reading[4] << 10 | reading[5] << 2 | ((reading[6] >> 2) & 0x03)
        x_adj = (x_raw - self._x_offset - self._x_hi_offset) * self._x_si_offset
        y_adj = (y_raw - self._y_offset - self._y_hi_offset) * self._y_si_offset
        z_adj = (z_raw - self._z_offset - self._z_hi_offset) * self._z_si_offset
        return (x_adj, y_adj, z_adj)
