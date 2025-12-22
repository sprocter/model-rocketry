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
    print("Calibration begins in five seconds.")
    time.sleep(5)
    print("Calibration beginning now, it will take 10 seconds...")
    for _ in range(1000):
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        acc_xs.append(accel_reading[0])
        acc_ys.append(accel_reading[1])
        acc_zs.append(accel_reading[2])
        gyro_xs.append(accel_reading[3])
        gyro_ys.append(accel_reading[4])
        gyro_zs.append(accel_reading[5])
        time.sleep_ms(10)

    print("_ACC_X_ERR = ", sum(acc_xs) / len(acc_xs))
    print("_ACC_Y_ERR = ", sum(acc_ys) / len(acc_ys))
    print("_ACC_Z_ERR = ", sum(acc_zs) / len(acc_zs) - 1)
    print("_GYRO_X_ERR = ", sum(gyro_xs) / len(gyro_xs))
    print("_GYRO_Y_ERR = ", sum(gyro_ys) / len(gyro_ys))
    print("_GYRO_Z_ERR = ", sum(gyro_zs) / len(gyro_zs))

def kalman_test():
    from ulab import numpy as np
    from kalman import KalmanFilter
    from machine import freq

    machine.freq(240000000)
    print(f"CPU Frequency: {machine.freq()}")

    init_timestamp = time.ticks_us()

    F = np.array([[1, 1], [0, 1]])
    B = np.array([[0.5], [1]])
    H = np.array([[1, 0]])
    Q = np.array([[1, 0], [0, 1]])
    R = np.array([[1]])

    x0 = np.array([[0], [1]])
    P0 = np.array([[1, 0], [0, 1]])

    kf = KalmanFilter(F, B, H, Q, R, x0, P0)

    u = np.array([[1]])
    z = np.array([[1]])

    setup_timestamp = time.ticks_us()

    predicted_state = kf.predict(u)
    # print("Predicted state:\n", predicted_state)

    prediction_timestamp = time.ticks_us()

    updated_state = kf.update(z)
    # print("Updated state:\n", updated_state)

    update_timestamp = time.ticks_us()

    setup_time = time.ticks_diff(setup_timestamp, init_timestamp)    
    prediction_time = time.ticks_diff(prediction_timestamp, setup_timestamp)
    update_time = time.ticks_diff(update_timestamp, prediction_timestamp)
    
    print(f"Setup Time: {setup_time}")
    print(f"Prediction Time: {prediction_time}")
    print(f"Update Time: {update_time}")



print_icm20649_offsets()
