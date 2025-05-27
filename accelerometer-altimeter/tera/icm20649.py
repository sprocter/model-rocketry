from decimal import Decimal
from typing import Generator, Dict
from struct import unpack
from math import sqrt

from more_itertools import peekable

class ICM20649:

    def __init__(
            self, 
            x_err : float,
            y_err : float,
            z_err : float,
            samplerate_num : int
        ):
        self._X_ERR = x_err
        self._Y_ERR = y_err
        self._Z_ERR = z_err

        self.samplerate_num = samplerate_num

        self.xs : Dict[float, float] = {}
        self.ys : Dict[float, float] = {}
        self.zs : Dict[float, float] = {}
        self.accel : Dict[float, float] = {}

        self.timestamp = peekable(self._timestamps())

    def _timestamps(self) -> Generator[float]:
        cur_timestamp = Decimal(0)
        while True:
            yield float(cur_timestamp.quantize(Decimal('0.0001')))
            cur_timestamp += Decimal(1)/(Decimal(1125)/Decimal(1 + self.samplerate_num))
    
    def store_reading(self, reading : bytes) -> None:
        raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
        accel_x = (raw_accel_x / 1024) - self._X_ERR
        accel_y = (raw_accel_y / 1024) - self._Y_ERR
        accel_z = (raw_accel_z / 1024) - self._Z_ERR
        cur_idx = next(self.timestamp)
        self.xs[cur_idx] = accel_x
        self.ys[cur_idx] = accel_y
        self.zs[cur_idx] = accel_z
        self.accel[cur_idx] = sqrt(accel_x**2 + accel_y**2 + accel_z**2)
