"""A very simple driver for the ICM20649 Accelerometer and Gyroscope

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C
from struct import unpack
import time


ICM20649_ADDR = const(0x68)
G_TO_MS2 = const(9.80665)  # https://en.wikipedia.org/wiki/Standard_gravity

# Page and section numbers refer to the datasheet

## Bank 0
_REG_CHIPID = const(0x00)  # pg 38, 8.1.1
_REG_PWR_MGMT_1 = const(0x06)  # pg 39, 8.1.4
_REG_PWR_MGMT_2 = const(0x07)  # pg 39, 8.1.5
_REG_ACCEL_XOUT_H = const(0x2D)  # pg 44, 8.1.17
_REG_BANK_SEL = const(0x7F)  # pg 56, 8.1.65

## Bank 2
_REG_GYRO_SMPLRT_DIV = const(0x00)  # pg 61, 8.3.1
_REG_GYRO_CONFIG_1 = const(0x01)  # pg 61, 8.3.1
_REG_ACCEL_SMPLRT_DIV_2 = const(0x11)  # pg 65, 8.3.12
_REG_ACCEL_CONFIG = const(0x14)  # pg 66, 8.3.15
_REG_TEMP_CONFIG = const(0x53)  # pg 69, 8.3.18

_EXPECTED_DEVICE_ID = const(0xE1)  # pg 38, 8.1.1

_GYRO_SENSITIVITY = const(8.2)  # pg 12, 3.1
_ACCEL_SENSITIVITY = const(1024)  # pg 13, 3.2

_ACC_X_ERR = const(0.10822296)
_ACC_Y_ERR = const(-0.21025884)
_ACC_Z_ERR = const(0.58796824)
_GYRO_X_ERR = const(-0.58353684)
_GYRO_Y_ERR = const(1.4780478)
_GYRO_Z_ERR = const(-0.4469513)


_ACCEL_ADJUST = const(G_TO_MS2 / _ACCEL_SENSITIVITY)

_TEMP_SENSITIVITY = const(333.87)  # pg 15, 3.3.2
_TEMP_OFFSET = const(0)  # pg 15, 3.3.2


class ICM20649:

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        self.buffer = bytearray(14)
        self.addr = ICM20649_ADDR

    def initialize(self) -> None:
        if ICM20649_ADDR not in self.i2c.scan():
            raise OSError(f"ICM20649 not found at {ICM20649_ADDR}")
        actual_device_id = unpack(
            "<B", self.i2c.readfrom_mem(ICM20649_ADDR, _REG_CHIPID, 1)
        )[0]
        if actual_device_id != _EXPECTED_DEVICE_ID:
            raise OSError(f"ICM20649 has incorrect device id {actual_device_id}")

        # Reset the device
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_PWR_MGMT_1, b"\x80")

        time.sleep_ms(5)  # Give the device time to do its reset

        # Don't reset the device, wake it up from sleep, don't enable low-power
        # Write 0 to reserve (😰), enable temperature sensor,
        # auto select best clock source
        # 0 0 0 0 001
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_PWR_MGMT_1, b"\x01")

        # Write 00 to reserved spot, enable accelerometer, enable gyro
        # 00 000 000
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_PWR_MGMT_2, b"\x00")

        time.sleep_ms(50)  # Allow gyro and accelerometer to wake up

        # Switch to bank 2
        # Bits 7, 6, 3, 2, 1 and 0 are reserved
        # 00 10 0000
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_BANK_SEL, b"\x20")

        # Sample rate = 1.1kHz/(1+ X), X = 24, ODR = 45.8333Hz
        # 00011000
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_GYRO_SMPLRT_DIV, b"\x18")

        # Bits 7 and 6 are reserved, gyro DLPF is set to 3 (?? I don't get this)
        # Gyroscope is in full scale, enable gyro DLPF
        # 00 011 11 1
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_GYRO_CONFIG_1, b"\x1f")

        # Sample rate = 1.1kHz/(1+X), X = 24, ODR = 45.8333Hz
        # 00011000
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_ACCEL_SMPLRT_DIV_2, b"\x18")

        # Bits 7 and 6 are reserved, accel DLPF is set to 3 (? I don't get this)
        # Accelerometer is in full scale, enable accel DLPF
        # 00 011 11 1
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_ACCEL_CONFIG, b"\x1f")

        # Does a temperature sensor need a low pass filter? This seems weird
        # 00000 011
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_TEMP_CONFIG, b"\x03")

        # Back to bank 0
        # Bits 7, 6, 3, 2, 1 and 0 are reserved
        # 00 00 0000
        self.i2c.writeto_mem(ICM20649_ADDR, _REG_BANK_SEL, b"\x00")

        time.sleep_ms(50)  # Let the low-pass filters "warm up"

    def read_raw(self) -> None:
        self.i2c.readfrom_mem_into(ICM20649_ADDR, _REG_ACCEL_XOUT_H, self.buffer)

    def decode_reading(
        self, reading: bytearray
    ) -> tuple[float, float, float, float, float, float, float]:
        unpacked_reading = unpack(">hhhhhhh", reading)
        # TODO: Rewrite to call others
        return (
            unpacked_reading[0] * _ACCEL_ADJUST - _ACC_X_ERR,
            unpacked_reading[1] * _ACCEL_ADJUST - _ACC_Y_ERR,
            unpacked_reading[2] * _ACCEL_ADJUST - _ACC_Z_ERR,
            unpacked_reading[3] / _GYRO_SENSITIVITY - _GYRO_X_ERR,
            unpacked_reading[4] / _GYRO_SENSITIVITY - _GYRO_Y_ERR,
            unpacked_reading[5] / _GYRO_SENSITIVITY - _GYRO_Z_ERR,
            (unpacked_reading[6] - _TEMP_OFFSET) / _TEMP_SENSITIVITY + 21,
        )
    
    @micropython.native
    def decode_accel(self, reading: bytearray) -> tuple[float, float, float]:
        unpacked_reading = unpack(">hhhhhhh", reading)
        return (
            unpacked_reading[0] * _ACCEL_ADJUST - _ACC_X_ERR,
            unpacked_reading[1] * _ACCEL_ADJUST - _ACC_Y_ERR,
            unpacked_reading[2] * _ACCEL_ADJUST - _ACC_Z_ERR,
        )

    @micropython.native
    def decode_gyro(self, reading:bytearray) -> tuple[float, float, float]:
        unpacked_reading = unpack(">hhhhhhh", reading)
        return (
            unpacked_reading[3] / _GYRO_SENSITIVITY - _GYRO_X_ERR,
            unpacked_reading[4] / _GYRO_SENSITIVITY - _GYRO_Y_ERR,
            unpacked_reading[5] / _GYRO_SENSITIVITY - _GYRO_Z_ERR,
        )
    
    def decode_temp(self, reading:bytearray) -> float:
        unpacked_reading = unpack(">hhhhhhh", reading)
        return (unpacked_reading[6] - _TEMP_OFFSET) / _TEMP_SENSITIVITY + 21,

    @property
    def error(self) -> float:
        """
        Return the standard deviation of the sensor's error in meters per second per second

        :returns: The standard deviation of the decoded readings' error
        :rtype: float
        """
        return 4.0