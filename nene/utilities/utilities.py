"""Utility functions for use with the Nene model rocket flight computer.

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""


def reset_filesystem():
    import vfs

    vfs.umount("/")
    vfs.VfsLfs2.mkfs(bdev)  # type: ignore


def reset_nvs():
    import esp32

    p = esp32.Partition.find(esp32.Partition.TYPE_DATA, label="nvs")[0]

    # p.info()[3] is partition size
    for x in range(int(p.info()[3] / 4096)):
        p.writeblocks(x, bytearray(4096))

    machine.reset()


def print_filesystem_space():
    import os

    stat = os.statvfs("/")
    size = stat[1] * stat[2]
    free = stat[0] * stat[3]
    used = size - free

    KB = 1024
    MB = 1024 * 1024

    print("Size : {:,} bytes, {:,} KB, {} MB".format(size, size / KB, size / MB))
    print("Used : {:,} bytes, {:,} KB, {} MB".format(used, used / KB, used / MB))
    print("Free : {:,} bytes, {:,} KB, {} MB".format(free, free / KB, free / MB))


def get_alti():
    from bmp581 import BMP581
    from machine import I2C

    i2c = I2C(scl=9, sda=8)
    alti = BMP581(i2c)
    alti.initialize()
    alti.read_raw()
    print(alti.decode_reading(alti.buffer))


def monitor_charging():
    """Print battery charging info to the terminal

    This is designed to be run on an Adafruit KB2040 (which they give away with larger orders, so it's easy to have spares laying around) that's connected to a MAX17048 breakout via STEMMA/QWIIC. It will print battery charge level and rate info to the terminal every five seconds. This is useful when you can't charge your batteries using a Feather (perhaps because they need to charge at a different rate). Using something like SparkFun's Adjustable LiPo Charger is nice for that, but it doesn't report detailed charge info -- just "Full" or "Not Full" via an onboard LED.
    """
    from machine import I2C
    from struct import unpack
    import time

    _MAX17048_ADDR = const(0x36)

    _MAX17048_SOC = const(0x04)
    _MAX17048_CRATE = const(0x16)

    i2c = I2C(scl=13, sda=12)

    while True:
        soc = i2c.readfrom_mem(_MAX17048_ADDR, _MAX17048_SOC, 2)
        charge_percent = (unpack(">H", soc)[0]) / 256.0

        rate = i2c.readfrom_mem(_MAX17048_ADDR, _MAX17048_CRATE, 2)
        charge_rate = (unpack(">h", rate)[0]) * 0.208

        print(f"Battery is at {charge_percent}%, charging at {charge_rate}%/hr.")
        time.sleep(5)


monitor_charging()
