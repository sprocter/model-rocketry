"""A module for decoding data from a model rocket accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------

This script contains methods to read data from the InvenSense ICM20649: https://www.adafruit.com/product/4464

To get the device-specific calibration values, call print_calib_values() on the device after the driver has been initialized.

Note that none of this code is designed to be modified by the user.
"""

from decimal import Decimal
from typing import Generator, Dict
from struct import unpack
from math import sqrt

from more_itertools import peekable

_X_ERR = 0.029050755
"""Accelerometer X error. 

To get this value, call `ICM20649.print_accel_calib_values()` on the device  when it's connected to the ICM 20649
"""

_Y_ERR = -0.008103238
"""Accelerometer Y error. 

To get this value, call `ICM20649.print_accel_calib_values()` on the device  when it's connected to the ICM 20649
"""

_Z_ERR = 0.044206185
"""Accelerometer Z error. 

To get this value, call `ICM20649.print_accel_calib_values()` on the device  when it's connected to the ICM 20649
"""


class ICM20649:
    """A class for decoding data from a model rocket accelerometer

    After instantiation, this class has three globals -- `xs`, `ys`, and `zs` -- which are dictionaries that map timestamps to decoded accelerations in the x, y, and z axes. The timestamps' units are seconds; they start at 0.0 and increase by the amount of time between samples as specified by the samplerate provided when the class is initialized. All accelerations are in g.
    """

    def __init__(self, samplerate_num: int):
        """Iniitalizes the class.

        This method simply initializes the global variables.

        :param int samplerate_num: This number is used to determine the samplerate (in hz) via this formula: 1125/(1+samplerate_num).
        """

        self.xs: Dict[float, float] = {}
        self.ys: Dict[float, float] = {}
        self.zs: Dict[float, float] = {}

        self.timestamp_increment = Decimal(1) / (
            Decimal(1125) / Decimal(1 + samplerate_num)
        )
        self.timestamp = peekable(self._timestamps())

    def _timestamps(self) -> Generator[float]:
        """Generator for timestamps

        This generator provides timestamps as parts of a second, derived from the sample rate specified when the class was instantiated.
        :return: The generator
        :rtype: Generator[float]
        """
        cur_timestamp = Decimal(0)
        while True:
            yield float(cur_timestamp.quantize(Decimal("0.0001")))
            cur_timestamp += self.timestamp_increment

    def store_reading(self, reading: bytes) -> None:
        """Decodes a single reading and stores it in memory

        This method decodes the single provided reading and updates the lists of x, y, and z accelerations.

        :param reading bytes: A six-byte sensor reading. The first pair of bytes are the x acceleration, the second pair are the y acceleration, and the third pair is the z acceleration.
        """
        raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
        accel_x = (raw_accel_x / 1024) - _X_ERR
        accel_y = (raw_accel_y / 1024) - _Y_ERR
        accel_z = (raw_accel_z / 1024) - _Z_ERR
        cur_idx = next(self.timestamp)
        self.xs[cur_idx] = accel_x
        self.ys[cur_idx] = accel_y
        self.zs[cur_idx] = accel_z

    @property
    def accel(self) -> Dict[float, float]:
        """Returns a map from timestamps to total acceleration

        This returns a map derived from / structurally identical to the accelerations in the individual axes but instead holding the overall acceleration.

        :return: A mappring from timestamps to acceleration (in g)
        :rtype: Dict[float, float]
        """
        return {
            k: sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            for k, v in zip(
                self.xs.keys(),
                zip(self.xs.values(), self.ys.values(), self.zs.values()),
            )
        }
