"""Config file generator for use with the Nene model rocket flight computer

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

import os, json, sys, random, binascii, machine, time, json

_G_TO_MS2 = const(9.80665)  # https://en.wikipedia.org/wiki/Standard_gravity

idToDeviceInfo = {
    "48": ("Magnetometer", "MMC5983"),
    "54": ("Battery Monitor", "MAX17048"),
    "71": ("Barometric Pressure Sensor", "BMP581"),
    "83": ("Accelerometer", "ADXL375"),
    "104": ("Accelerometer & Gyroscope", "ICM20649"),
    "107": ("Accelerometer & Gyroscope", "ISM330DHCX"),
}


def _load_existing_config() -> dict:
    files_in_root = os.listdir("/")

    if "config.json" in files_in_root:
        with open("/config.json", "r") as f:
            config = json.loads(f.read())
    else:
        config = {}

    return config


def _get_input(
    group: str, name: str, default_val: str, message: str, config: dict
) -> str:
    if group in config and name in config[group]:
        suggestion = config[group][name]
    else:
        suggestion = default_val
    inp = input(f"{message} [{suggestion}]: ")
    if inp == "":
        return suggestion
    else:
        return inp


def _get_input_int(
    group: str, name: str, default_val: str, message: str, config: dict
) -> int:
    return int(_get_input(group, name, default_val, message, config))


def _get_input_enc(
    group: str, name: str, num_bits: int, message: str, config: dict
) -> bytes:
    elems = []
    for _ in range(num_bits / 8):
        elems.append(random.getrandbits(8))
    default_val = binascii.hexlify(bytes(elems))
    return _get_input(group, name, default_val, message, config)


def _core_config(config: dict) -> None:
    if "wifi" not in config:
        config["wifi"] = {}
    config["wifi"]["name"] = _get_input("wifi", "name", "", "Wi-Fi SSID Name", config)
    config["wifi"]["key"] = _get_input("wifi", "key", "", "Wi-Fi Password", config)
    if "lora" not in config:
        config["lora"] = {}
    config["lora"]["lilbuddy_addr"] = _get_input_int(
        "lora",
        "lilbuddy_addr",
        random.randint(0, 255),
        "Lil-Buddy Address (0-255)",
        config,
    )
    config["lora"]["bigbuddy_addr"] = _get_input_int(
        "lora",
        "bigbuddy_addr",
        random.randint(0, 255),
        "Big-Buddy Address (0-255)",
        config,
    )
    config["lora"]["key"] = _get_input_enc(
        "lora", "key", 256, "Encryption Key (256 bit, hexlified bytes)", config
    )
    config["lora"]["iv"] = _get_input_enc(
        "lora",
        "iv",
        128,
        "Encryption Initialization Vector (128 bit, hexlified bytes)",
        config,
    )


def _part_1(config: dict) -> None:
    print("")
    print("Part 1: Core Configuration")
    print("--------------------------")
    _core_config(config)


def _mmc5983_offsets(config: dict, i2c: I2C, duration: int = 20) -> None:
    """Calculate hard-iron and soft-iron distortion in order to compensate for it.

    This takes samples over the user-specified number of seconds (default 10) and then does some math to figure out hard and soft iron distortion. Offsets / scale factors to compensate for that distortion are then stored in class variables and used to adjust future readings.

    Note that the soft iron distortion is a "good enough" implementation that does not do an actual ellipsoid fit.

    :param int duration: How long to run the calibration for in seconds

    """
    from mmc5983ma import MMC5983MA

    mag = MMC5983MA(i2c)
    defaults = {}
    defaults["X_HI_OFFSET"] = 0.0
    defaults["Y_HI_OFFSET"] = 0.0
    defaults["Z_HI_OFFSET"] = 0.0
    defaults["X_SI_OFFSET"] = 1.0
    defaults["Y_SI_OFFSET"] = 1.0
    defaults["Z_SI_OFFSET"] = 1.0
    mag.initialize(defaults)

    print("MMC5983 (Magnetometer) Calibration.")
    print(
        "Directions: Gently move the device in figure-eight like shapes while rotating it back and forth"
    )
    print("Calibration begins in 10 seconds.")
    time.sleep(10)
    print(f"Calibration beginning now, it will take {duration} seconds...")

    start_ts = time.ticks_ms()

    x_min = y_min = z_min = 999999999
    x_max = y_max = z_max = -999999999

    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        mag.read_raw()
        (x, y, z) = mag._decode_mag_internal(mag.buffer)
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
        time.sleep_ms(23)

    # "Hard iron" distortion
    x_hi_offset = (x_min + x_max) / 2
    y_hi_offset = (y_min + y_max) / 2
    z_hi_offset = (z_min + z_max) / 2

    # "Soft iron" distortion -- uses scale biases, would be more accurate with an ellipsoid fit. See https://www.appelsiini.net/2018/calibrate-magnetometer/
    avg_hi_offset = (x_hi_offset + y_hi_offset + z_hi_offset) / 3
    x_si_offset = avg_hi_offset / x_hi_offset
    y_si_offset = avg_hi_offset / y_hi_offset
    z_si_offset = avg_hi_offset / z_hi_offset

    config["MMC5983"] = {}
    config["MMC5983"]["X_HI_OFFSET"] = x_hi_offset
    config["MMC5983"]["Y_HI_OFFSET"] = y_hi_offset
    config["MMC5983"]["Z_HI_OFFSET"] = z_hi_offset
    config["MMC5983"]["X_SI_OFFSET"] = x_si_offset
    config["MMC5983"]["Y_SI_OFFSET"] = y_si_offset
    config["MMC5983"]["Z_SI_OFFSET"] = z_si_offset


def _icm20649_offsets(config: dict, i2c: I2C, duration: int = 20) -> None:
    from icm20649 import ICM20649

    accelerometer = ICM20649(i2c)
    defaults = {}
    defaults["ACC_X_ERR"] = 0.0
    defaults["ACC_Y_ERR"] = 0.0
    defaults["ACC_Z_ERR"] = 0.0
    defaults["GYRO_X_ERR"] = 0.0
    defaults["GYRO_Y_ERR"] = 0.0
    defaults["GYRO_Z_ERR"] = 0.0
    accelerometer.initialize(defaults)
    acc_xs, acc_ys, acc_zs = [], [], []
    gyro_xs, gyro_ys, gyro_zs = [], [], []

    print("ICM20649 (Accelerometer & Gyroscope) Calibration.")
    print("Directions: Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in 10 seconds.")
    time.sleep(10)
    print(f"Calibration beginning now, it will take {duration} seconds...")

    start_ts = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_accel(accelerometer.buffer)
        acc_xs.append(accel_reading[0])
        acc_ys.append(accel_reading[1])
        acc_zs.append(accel_reading[2])
        gyro_reading = accelerometer.decode_gyro(accelerometer.buffer)
        gyro_xs.append(gyro_reading[0])
        gyro_ys.append(gyro_reading[1])
        gyro_zs.append(gyro_reading[2])
        time.sleep_ms(23)

    config["ICM20649"] = {}
    config["ICM20649"]["ACC_X_ERR"] = sum(acc_xs) / len(acc_xs) - _G_TO_MS2
    config["ICM20649"]["ACC_Y_ERR"] = sum(acc_ys) / len(acc_ys)
    config["ICM20649"]["ACC_Z_ERR"] = sum(acc_zs) / len(acc_zs)
    config["ICM20649"]["GYRO_X_ERR"] = sum(gyro_xs) / len(gyro_xs)
    config["ICM20649"]["GYRO_Y_ERR"] = sum(gyro_ys) / len(gyro_ys)
    config["ICM20649"]["GYRO_Z_ERR"] = sum(gyro_zs) / len(gyro_zs)


def _ism330DHCX_offsets(config: dict, i2c: I2C, duration: int = 20) -> None:
    from ism330dhcx import ISM330DHCX

    accelerometer = ISM330DHCX(i2c)
    defaults = {}
    defaults["ACC_X_ERR"] = 0.0
    defaults["ACC_Y_ERR"] = 0.0
    defaults["ACC_Z_ERR"] = 0.0
    defaults["GYRO_X_ERR"] = 0.0
    defaults["GYRO_Y_ERR"] = 0.0
    defaults["GYRO_Z_ERR"] = 0.0
    accelerometer.initialize(defaults)
    acc_xs, acc_ys, acc_zs = [], [], []
    gyro_xs, gyro_ys, gyro_zs = [], [], []

    print("ISM330DHCX (Accelerometer & Gyroscope) Calibration.")
    print("Directions: Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in 10 seconds.")
    time.sleep(10)
    print(f"Calibration beginning now, it will take {duration} seconds...")

    start_ts = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_accel(accelerometer.buffer)
        acc_xs.append(accel_reading[0])
        acc_ys.append(accel_reading[1])
        acc_zs.append(accel_reading[2])
        gyro_reading = accelerometer.decode_gyro(accelerometer.buffer)
        gyro_xs.append(gyro_reading[0])
        gyro_ys.append(gyro_reading[1])
        gyro_zs.append(gyro_reading[2])
        time.sleep_ms(23)

    config["ISM330DHCX"] = {}
    config["ISM330DHCX"]["ACC_X_ERR"] = sum(acc_xs) / len(acc_xs) - _G_TO_MS2
    config["ISM330DHCX"]["ACC_Y_ERR"] = sum(acc_ys) / len(acc_ys)
    config["ISM330DHCX"]["ACC_Z_ERR"] = sum(acc_zs) / len(acc_zs)
    config["ISM330DHCX"]["GYRO_X_ERR"] = sum(gyro_xs) / len(gyro_xs)
    config["ISM330DHCX"]["GYRO_Y_ERR"] = sum(gyro_ys) / len(gyro_ys)
    config["ISM330DHCX"]["GYRO_Z_ERR"] = sum(gyro_zs) / len(gyro_zs)


def _adxl375_offsets(config: dict, i2c: I2C, duration: int = 20) -> None:
    from adxl375 import ADXL375

    accelerometer = ADXL375(i2c)
    defaults = {}
    defaults["ACC_X_ERR"] = 0.0
    defaults["ACC_Y_ERR"] = 0.0
    defaults["ACC_Z_ERR"] = 0.0
    accelerometer.initialize(defaults)
    xs, ys, zs = [], [], []

    print("ADXL375 (Accelerometer) Calibration.")
    print("Directions: Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in 10 seconds.")
    time.sleep(10)
    print(f"Calibration beginning now, it will take {duration} seconds...")
    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        xs.append(accel_reading[0])
        ys.append(accel_reading[1])
        zs.append(accel_reading[2])
        time.sleep_ms(23)

    config["ADXL375"] = {}
    config["ADXL375"]["ACC_X_ERR"] = sum(xs) / len(xs)
    config["ADXL375"]["ACC_Y_ERR"] = sum(ys) / len(ys)
    config["ADXL375"]["ACC_Z_ERR"] = sum(zs) / len(zs) - _G_TO_MS2


def _part_2(config: dict) -> None:
    print("")
    print("Part 2: Sensor Calibration")
    print("--------------------------")
    i2c = machine.I2C(scl=9, sda=8)
    connected_devices = i2c.scan()
    print(f"There are {len(connected_devices)} devices connected:")
    for dev in connected_devices:
        info = idToDeviceInfo[str(dev)]
        print(f"\t* {info[1]} ({info[0]})")

    if 48 in connected_devices:
        print("")
        _mmc5983_offsets(config, i2c)
    else:
        print(
            "FATAL ERROR: No magnetometer connected. Please connect a supported magnetometer (i.e. MMC5983) and re-run this script."
        )

    if 104 in connected_devices:
        print("")
        _icm20649_offsets(config, i2c)

    if 107 in connected_devices:
        print("")
        _ism330DHCX_offsets(config, i2c)

    if 104 not in connected_devices and 107 not in connected_devices:
        print(
            "FATAL ERROR: No gyroscope connected. Please connect a supported gyroscope (i.e. ICM20649 or ISM330DHCX) and re-run this script."
        )

    if 83 in connected_devices:
        _adxl375_offsets(config, i2c)


def _part_3(config: dict) -> None:
    print("")
    print("Part 3: Configuration Tests")
    print("---------------------------")
    inp = 0
    while True:
        print("Options:")
        print("\t1. Display Config File")
        print("\t2. Attitude and Heading Reference System Test")
        print("\t3. Send Encrypted Message via LoRa")
        print("\t4. Turn on WiFi and FTP Server")
        print("\t9. Exit")
        inp = int(input(f"Input selection: "))
        if inp == 1:
            with open("/config.json", "r") as f:
                print(json.loads(f.read()))
        elif inp == 2:
            print("Not yet implemented.")
        elif inp == 3:
            print("Not yet implemented.")
        elif inp == 4:
            print("Not yet implemented.")
        elif inp == 9:
            break


print("")
print("Nene Configuration Generator")
print("============================")
config = _load_existing_config()

_part_1(config)
_part_2(config)

with open("/config.json", "w") as f:
    json.dump(config, f)
print("")
print("Configuration saved.")

_part_3(config)

