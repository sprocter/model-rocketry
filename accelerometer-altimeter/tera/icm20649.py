from decimal import Decimal
from typing import Generator, Dict
from struct import unpack
from math import sqrt

from more_itertools import peekable


_X_ERR = 0.029050755
"""Accelerometer X error. 

To get this values, call `ICM20649.print_accel_calib_values()` on the device  when it's connected to the ICM 20649
"""

_Y_ERR = -0.008103238
"""Accelerometer Y error. 

To get this values, call `ICM20649.print_accel_calib_values()` on the device  when it's connected to the ICM 20649
"""

_Z_ERR = 0.044206185
"""Accelerometer Z error. 

To get this values, call `ICM20649.print_accel_calib_values()` on the device  when it's connected to the ICM 20649
"""


class ICM20649:

    def __init__(self, samplerate_num: int):

        self.samplerate_num = samplerate_num

        self.xs: Dict[float, float] = {}
        self.ys: Dict[float, float] = {}
        self.zs: Dict[float, float] = {}
        self.accel: Dict[float, float] = {}

        self.timestamp = peekable(self._timestamps())

    def _timestamps(self) -> Generator[float]:
        cur_timestamp = Decimal(0)
        while True:
            yield float(cur_timestamp.quantize(Decimal("0.0001")))
            cur_timestamp += Decimal(1) / (
                Decimal(1125) / Decimal(1 + self.samplerate_num)
            )

    def store_reading(self, reading: bytes) -> None:
        raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
        accel_x = (raw_accel_x / 1024) - _X_ERR
        accel_y = (raw_accel_y / 1024) - _Y_ERR
        accel_z = (raw_accel_z / 1024) - _Z_ERR
        cur_idx = next(self.timestamp)
        self.xs[cur_idx] = accel_x
        self.ys[cur_idx] = accel_y
        self.zs[cur_idx] = accel_z
        self.accel[cur_idx] = sqrt(accel_x**2 + accel_y**2 + accel_z**2)
