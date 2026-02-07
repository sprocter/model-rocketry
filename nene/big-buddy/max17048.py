"""A very (very!) simple driver for the MAX17048 Battery Monitor

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C
from struct import unpack

_MAX17048_ADDR = const(0x36)

_MAX17048_SOC = const(0x04)
_MAX17048_CRATE = const(0x16)


class MAX17048:

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c

    @property
    def charge_percent(self) -> float:
        soc = self.i2c.readfrom_mem(_MAX17048_ADDR, _MAX17048_SOC, 2)
        return (unpack(">H", soc)[0]) / 256.0

    @property
    def charge_rate(self) -> float:
        rate = self.i2c.readfrom_mem(_MAX17048_ADDR, _MAX17048_CRATE, 2)
        return (unpack(">h", rate)[0]) * 0.208
