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
    for _ in range(5):
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        acc_xs.append(accel_reading[0])
        acc_ys.append(accel_reading[1])
        acc_zs.append(accel_reading[2])
        gyro_xs.append(accel_reading[3])
        gyro_ys.append(accel_reading[4])
        gyro_zs.append(accel_reading[5])
        print(f"X: {accel_reading[0]}")
        print(f"Y: {accel_reading[1]}")
        print(f"Z: {accel_reading[2]}")
        time.sleep_ms(40)

    print(f"_ACC_X_ERR = const({sum(acc_xs) / len(acc_xs)})")
    print(f"_ACC_Y_ERR = const({sum(acc_ys) / len(acc_ys)})")
    print(f"_ACC_Z_ERR = const({sum(acc_zs) / len(acc_zs) - 9.80665})")
    print(f"_GYRO_X_ERR = const({sum(gyro_xs) / len(gyro_xs)})")
    print(f"_GYRO_Y_ERR = const({sum(gyro_ys) / len(gyro_ys)})")
    print(f"_GYRO_Z_ERR = const({sum(gyro_zs) / len(gyro_zs)})")

def get_alti():
    from bmp581 import BMP581
    from machine import I2C
    i2c = I2C(scl=9, sda=8)
    alti = BMP581(i2c)
    alti.initialize()
    alti.read_raw()
    print(alti.decode_reading(alti.buffer))

print_filesystem_space()
