"""Utility functions for use with the Nene model rocket flight computer.

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

import json, vfs, os, esp32, machine, time
from random import randint
from machine import I2C


def generate_secrets():
    secrets = {
        "wifi-ssid": "YourWiFiNameHere",
        "wifi-key": "YourWifiPasswordHere",
        "bigbuddy-addr": randint(0, 256),
        "lilbuddy-addr": randint(0, 256),
    }

    with open("/secrets.json", "w") as f:
        json.dump(secrets, f)


def reset_filesystem():
    vfs.umount("/")
    vfs.VfsLfs2.mkfs(bdev)  # type: ignore


def reset_nvs():
    p = esp32.Partition.find(esp32.Partition.TYPE_DATA, label="nvs")[0]

    # p.info()[3] is partition size
    for x in range(int(p.info()[3] / 4096)):
        p.writeblocks(x, bytearray(4096))

    machine.reset()


def print_filesystem_space():
    stat = os.statvfs("/")
    size = stat[1] * stat[2]
    free = stat[0] * stat[3]
    used = size - free

    KB = 1024
    MB = 1024 * 1024

    print("Size : {:,} bytes, {:,} KB, {} MB".format(size, size / KB, size / MB))
    print("Used : {:,} bytes, {:,} KB, {} MB".format(used, used / KB, used / MB))
    print("Free : {:,} bytes, {:,} KB, {} MB".format(free, free / KB, free / MB))


def print_adxl375_offsets():
    from adxl375 import ADXL375

    i2c = I2C(sda=41, scl=40)
    accelerometer = ADXL375(i2c)
    accelerometer.initialize()
    xs, ys, zs = [], [], []
    print("Please ensure X, Y, and Z ERR constants are set to 0 in the ADXL375 Driver")
    print("Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in five seconds.")
    time.sleep(5)
    print("Calibration beginning now, it will take 10 seconds...")
    for _ in range(250):
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        xs.append(accel_reading[0])
        ys.append(accel_reading[1])
        zs.append(accel_reading[2])
        time.sleep_ms(40)

    print("_X_ERR = ", sum(xs) / len(xs))
    print("_Y_ERR = ", sum(ys) / len(ys))
    print("_Z_ERR = ", sum(zs) / len(zs) - 1)


def print_icm20649_offsets():
    from icm20649 import ICM20649

    i2c = I2C(scl=9, sda=8)
    accelerometer = ICM20649(i2c)
    accelerometer.initialize()
    acc_xs, acc_ys, acc_zs = [], [], []
    gyro_xs, gyro_ys, gyro_zs = [], [], []
    print("Please ensure X, Y, and Z ERR constants are set to 0 in the ICM20649 Driver")
    print("Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in three seconds.")
    time.sleep(3)
    print("Calibration beginning now, it will take 10 seconds...")
    for _ in range(200):
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        acc_xs.append(accel_reading[0])
        acc_ys.append(accel_reading[1])
        acc_zs.append(accel_reading[2])
        gyro_xs.append(accel_reading[3])
        gyro_ys.append(accel_reading[4])
        gyro_zs.append(accel_reading[5])
        # print(f"X: {accel_reading[0]}")
        # print(f"Y: {accel_reading[1]}")
        # print(f"Z: {accel_reading[2]}")
        time.sleep_ms(50)

    print(f"_ACC_X_ERR = const({sum(acc_xs) / len(acc_xs)- 9.80665})")
    print(f"_ACC_Y_ERR = const({sum(acc_ys) / len(acc_ys)})")
    print(f"_ACC_Z_ERR = const({sum(acc_zs) / len(acc_zs)})")
    print(f"_GYRO_X_ERR = const({sum(gyro_xs) / len(gyro_xs)})")
    print(f"_GYRO_Y_ERR = const({sum(gyro_ys) / len(gyro_ys)})")
    print(f"_GYRO_Z_ERR = const({sum(gyro_zs) / len(gyro_zs)})")


def print_mmc5983_offsets(duration=10):
    """Calculate hard-iron and soft-iron distortion in order to compensate for it.

    This takes samples over the user-specified number of seconds (default 10) and then does some math to figure out hard and soft iron distortion. Offsets / scale factors to compensate for that distortion are then stored in class variables and used to adjust future readings.

    Note that the soft iron distortion is a "good enough" implementation that does not do an actual ellipsoid fit.

    :param int duration: How long to run the calibration for in seconds

    """
    from mmc5983ma import MMC5983MA

    i2c = I2C(scl=9, sda=8)
    mag = MMC5983MA(i2c)
    mag.initialize()

    print("Please ensure X, Y, and Z hard iron constants are set to 0 and the X, Y, and Z soft iron constants are set to 1 in the MMC5983MA Driver")
    print("Gently move the device in figure-eight like shapes while rotating it back and forth")
    print("Calibration begins in 3 seconds.")
    time.sleep(3)
    print(f"Calibration beginning now, it will take {duration} seconds...")

    start_ts = time.ticks_ms()

    x_min = y_min = z_min = 999999999
    x_max = y_max = z_max = -999999999

    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        mag.read_raw()
        (x, y, z) = mag.decode_mag(mag.buffer)
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
    x_hi_offset = (x_min + x_max) / 2
    y_hi_offset = (y_min + y_max) / 2
    z_hi_offset = (z_min + z_max) / 2

    # "Soft iron" distortion -- uses scale biases, would be more accurate with an ellipsoid fit. See https://www.appelsiini.net/2018/calibrate-magnetometer/
    avg_hi_offset = (x_hi_offset + y_hi_offset + z_hi_offset) / 3
    x_si_offset = avg_hi_offset / x_hi_offset
    y_si_offset = avg_hi_offset / y_hi_offset
    z_si_offset = avg_hi_offset / z_hi_offset

    print(f"_X_HI_OFFSET = const({x_hi_offset})")
    print(f"_Y_HI_OFFSET = const({y_hi_offset})")
    print(f"_Z_HI_OFFSET = const({z_hi_offset})")
    print(f"_X_SI_OFFSET = const({x_si_offset})")
    print(f"_Y_SI_OFFSET = const({y_si_offset})")
    print(f"_Z_SI_OFFSET = const({z_si_offset})")


def get_alti():
    from bmp581 import BMP581
    from machine import I2C

    i2c = I2C(scl=9, sda=8)
    alti = BMP581(i2c)
    alti.initialize()
    alti.read_raw()
    print(alti.decode_reading(alti.buffer))


print_icm20649_offsets()
