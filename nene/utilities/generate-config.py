"""Config file generator for use with the Nene model rocket flight computer

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

import os, json, sys, random, binascii, machine, time, json, random

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
    if "system" not in config:
        config["system"] = {}
    suffix = random.randrange(0, 9999)
    config["system"]["name"] = _get_input(
        "system", "name", f"Nene{str(suffix)}", "Device Name", config
    )
    if "wifi" not in config:
        config["wifi"] = {}
    config["wifi"]["name"] = _get_input("wifi", "name", "", "Wi-Fi SSID Name", config)
    config["wifi"]["key"] = _get_input("wifi", "key", "", "Wi-Fi Password", config)
    if "lora" not in config:
        config["lora"] = {}
    config["lora"]["freq"] = float(
        _get_input_int(
            "lora",
            "freq",
            random.randint(902, 928),
            "LoRa Frequency (902-928)",
            config,
        )
    )
    config["lora"]["key"] = _get_input_enc(
        "lora", "key", 256, "Encryption Key (256 bit, hexlified bytes)", config
    )
    config["pins"]["buzzer1"] = _get_input_int("pins", "buzzer1", "10", "Buzzer Pin 1", config)
    config["pins"]["buzzer2"] = _get_input_int("pins", "buzzer2", "11", "Buzzer Pin 2", config)
    config["system"]["buzzer_freq_1"] = _get_input_int("system", "buzzer_freq_1", "3201", "Buzzer Frequency 1", config)
    config["system"]["buzzer_freq_2"] = _get_input_int("system", "buzzer_freq_2", "1150", "Buzzer Frequency 2", config)


def _basic_config(config: dict) -> None:
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

    config["MMC5983MA"] = {}
    config["MMC5983MA"]["X_HI_OFFSET"] = x_hi_offset
    config["MMC5983MA"]["Y_HI_OFFSET"] = y_hi_offset
    config["MMC5983MA"]["Z_HI_OFFSET"] = z_hi_offset
    config["MMC5983MA"]["X_SI_OFFSET"] = x_si_offset
    config["MMC5983MA"]["Y_SI_OFFSET"] = y_si_offset
    config["MMC5983MA"]["Z_SI_OFFSET"] = z_si_offset


def _icm20649_offsets(config: dict, i2c: I2C, duration: int = 10) -> None:
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
    print("Directions: Put the device in its launch orientation and hold it still.")
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
    if config["orient"]["transpose"] == (2, 1, 0):
        config["ICM20649"]["ACC_X_ERR"] = sum(acc_xs) / len(acc_xs) - _G_TO_MS2
        config["ICM20649"]["ACC_Y_ERR"] = sum(acc_ys) / len(acc_ys)
    elif config["orient"]["transpose"] == (2, 0, 1):
        config["ICM20649"]["ACC_X_ERR"] = sum(acc_xs) / len(acc_xs)
        config["ICM20649"]["ACC_Y_ERR"] = sum(acc_ys) / len(acc_ys) + _G_TO_MS2
    else:
        print("FATAL ERROR: Attempting to calibrate ICM20649 with unknown orientation")
        raise Exception
    config["ICM20649"]["ACC_Z_ERR"] = sum(acc_zs) / len(acc_zs)
    config["ICM20649"]["GYRO_X_ERR"] = sum(gyro_xs) / len(gyro_xs)
    config["ICM20649"]["GYRO_Y_ERR"] = sum(gyro_ys) / len(gyro_ys)
    config["ICM20649"]["GYRO_Z_ERR"] = sum(gyro_zs) / len(gyro_zs)


def _ism330DHCX_offsets(config: dict, i2c: I2C, duration: int = 10) -> None:
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
    print("Directions: Put the device in its launch orientation and hold it still.")
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


def _adxl375_offsets(config: dict, i2c: I2C, duration: int = 10) -> None:
    from adxl375 import ADXL375

    accelerometer = ADXL375(i2c)
    defaults = {}
    defaults["ACC_X_ERR"] = 0.0
    defaults["ACC_Y_ERR"] = 0.0
    defaults["ACC_Z_ERR"] = 0.0
    accelerometer.initialize(defaults)
    xs, ys, zs = [], [], []

    print("ADXL375 (Accelerometer) Calibration.")
    print("Directions: Put the device in its launch orientation and hold it still.")
    print("Calibration begins in 10 seconds.")
    time.sleep(10)
    print(f"Calibration beginning now, it will take {duration} seconds...")
    start_ts = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_accel(accelerometer.buffer)
        xs.append(accel_reading[0])
        ys.append(accel_reading[1])
        zs.append(accel_reading[2])
        time.sleep_ms(23)

    config["ADXL375"] = {}
    config["ADXL375"]["ACC_X_ERR"] = sum(xs) / len(xs) - _G_TO_MS2
    config["ADXL375"]["ACC_Y_ERR"] = sum(ys) / len(ys)
    config["ADXL375"]["ACC_Z_ERR"] = sum(zs) / len(zs)


def _configure_pins(config: dict) -> None:
    if "pins" not in config:
        config["pins"] = {}
    config["orient"] = {}
    if len((machine.I2C(scl=9, sda=8)).scan()) > 0:
        # FeatherS3D config
        config["pins"]["i2c_scl"] = 9
        config["pins"]["i2c_sda"] = 8
        config["pins"]["uart_tx"] = 16
        config["pins"]["uart_rx"] = 15
        config["pins"]["spi_clk"] = 36
        config["pins"]["spi_mosi"] = 35
        config["pins"]["spi_miso"] = 37
        config["pins"]["lora_cs"] = 5
        config["pins"]["lora_dio1"] = 6
        config["pins"]["lora_rst"] = 44
        config["pins"]["lora_busy"] = 43
        config["pins"]["button"] = 0
        config["pins"]["led1"] = 40
        config["pins"]["led2"] = None
        config["system"]["gps_pmtk_cmds"] = True
        config["system"]["have_neopixel"] = True
        config["system"]["have_batt_mon"] = True
        config["orient"]["transpose"] = (2, 1, 0)
        config["orient"]["invert"] = (False, False, False)
    elif len((machine.I2C(scl=6, sda=5)).scan()) > 0:
        # Xiao ESP32S3 Plus config
        config["pins"]["i2c_scl"] = 6
        config["pins"]["i2c_sda"] = 5
        config["pins"]["uart_tx"] = 43
        config["pins"]["uart_rx"] = 44
        config["pins"]["spi_clk"] = 7
        config["pins"]["spi_mosi"] = 9
        config["pins"]["spi_miso"] = 8
        config["pins"]["lora_cs"] = 41
        config["pins"]["lora_dio1"] = 39
        config["pins"]["lora_rst"] = 42
        config["pins"]["lora_busy"] = 40
        config["pins"]["button"] = 21
        config["pins"]["led1"] = 48
        config["pins"]["led2"] = 21
        config["system"]["gps_pmtk_cmds"] = False
        config["system"]["have_neopixel"] = False
        config["system"]["have_batt_mon"] = False
        config["orient"]["transpose"] = (2, 0, 1)
        config["orient"]["invert"] = (False, False, True)
    else:
        print("FATAL ERROR: No Devices found on the I2C bus.")


def _sensor_calibration(config: dict) -> None:
    print("")
    print("Part 2: Device Calibration")
    print("--------------------------")
    print(f"\t1. Return to main menu")
    _configure_pins(config)
    i2c = machine.I2C(scl=config["pins"]["i2c_scl"], sda=config["pins"]["i2c_sda"])
    connected_devices = i2c.scan()
    i = 2
    menu_to_id = {}
    for dev in connected_devices:
        if dev == 54 or dev == 71:
            continue
        info = idToDeviceInfo[str(dev)]
        menu_to_id[i] = dev
        print(f"\t{i}. {info[1]} ({info[0]})")
        i += 1

    val_ok = False
    while not val_ok:
        try:
            inp = int(input(f"Input selection: "))
            val_ok = True
        except ValueError as verr:
            print("Bad input (can't convert to int)")

    if inp == 1:
        return
    elif menu_to_id[inp] == 48:
        print("")
        _mmc5983_offsets(config, i2c)
    elif menu_to_id[inp] == 104:
        print("")
        _icm20649_offsets(config, i2c)
    elif menu_to_id[inp] == 107:
        print("")
        _ism330DHCX_offsets(config, i2c)
    elif menu_to_id[inp] == 83:
        print("")
        _adxl375_offsets(config, i2c)


def _print_config() -> None:
    print(json.dumps(config))


def _test_AHRS() -> None:
    from marg import StateEstimator
    from bmp581 import BMP581
    from icm20649 import ICM20649
    from adxl375 import ADXL375
    from ism330dhcx import ISM330DHCX
    from mmc5983ma import MMC5983MA
    import fusion_wrapper as ahrs

    default = 60
    duration = input(f"How long (in seconds) should the test run? [{default}]: ")
    if duration == "":
        duration = default
    else:
        duration = int(duration)

    i2c = machine.I2C(scl=config["pins"]["i2c_scl"], sda=config["pins"]["i2c_sda"])
    connected_devices = i2c.scan()
    alti = BMP581(i2c)
    if ADXL375.ADDR in connected_devices:
        accel = ADXL375(i2c)
        accel.initialize(config["ADXL375"])
    elif ICM20649.ADDR in connected_devices:
        accel = ICM20649(i2c)
        accel.initialize(config["ICM20649"])
    elif ISM330DHCX.ADDR in connected_devices:
        accel = ISM330DHCX(i2c)
        accel.initialize(config["ISM330DHCX"])
    else:
        raise OSError(f"No accelerometer connected!")

    if ISM330DHCX.ADDR in connected_devices:
        if isinstance(accel, ISM330DHCX):
            gyro = accel
        else:
            gyro = ISM330DHCX(i2c)
            gyro.initialize(config["ISM330DHCX"])
    elif ICM20649.ADDR in connected_devices:
        if isinstance(accel, ICM20649):
            gyro = accel
        else:
            gyro = ICM20649(i2c)
            gyro.initialize(config["ICM20649"])
    else:
        raise OSError(f"No gyroscope connected!")

    mag = MMC5983MA(i2c)
    alti.initialize()
    mag.initialize(config["MMC5983MA"])
    estimator = StateEstimator(
        23,
        alti.error,
        accel.error,
        config["orient"]["transpose"],
        config["orient"]["invert"],
    )

    gain = 0.4
    gyro_range = 500.0
    accelerometer_reject = 10.0
    magnetometer_reject = 10.0
    rejection_timeout = 10.0  # Go higher? Like 15?
    declination = -9.38333  # Declination in degrees for PSC Launch site
    if config["orient"]["transpose"] == (2, 1, 0):
        alignment = 1
    elif config["orient"]["transpose"] == (2, 0, 1):
        alignment = 2
    else:
        raise LookupError(
            f"Unsupported sensor orientation: {config["orient"]["transpose"]}"
        )
    ahrs.init_ahrs(
        45,
        gain,
        gyro_range,
        accelerometer_reject,
        magnetometer_reject,
        rejection_timeout,
        declination,
        alignment,
    )

    start_ts = time.ticks_ms()
    i = 0
    while time.ticks_diff(time.ticks_ms(), start_ts) < duration * 1000:
        alti.read_raw()
        mag.read_raw()
        accel.read_raw()
        gyro.read_raw()
        mag_rdg = mag.decode_mag(mag.buffer)
        acc_rdg = accel.decode_accel(accel.buffer)
        estimator.acceleration = acc_rdg
        gyro_rdg = gyro.decode_gyro(gyro.buffer)
        estimator.altitude = alti.decode_alti(alti.buffer)
        i += 1
        ahrs.update(gyro_rdg, acc_rdg, mag_rdg)
        [roll, pitch, yaw, tilt] = ahrs.get_euler()
        altitude = estimator.altitude
        velocity = estimator.velocity
        if i % 10 == 0:
            print(
                f"Tilt: {tilt:.2f}\tHeading: {roll:.2f}\tPitch: {pitch:.2f}\tRoll: {yaw:.2f}\tAltitude: {altitude:.2f}\tVelocity: {velocity:.2f}"
            )
        time.sleep_ms(23)


def _send_lora_msg() -> None:
    from lora import SX1262
    from machine import Pin, SPI
    from binascii import unhexlify
    import struct, cryptolib

    spi_bus = 1
    baud_rate = 2_000_000
    polarity = 0
    phase = 0
    tcxo_mv = (
        3300 - 200
    )  # "TCXO voltage should always be 200 mV less than the VCC to ensure proper operation" -- 1262 datasheet, page 7
    clk = Pin(int(config["pins"]["spi_clk"]))
    mosi = Pin(int(config["pins"]["spi_mosi"]))
    miso = Pin(int(config["pins"]["spi_miso"]))
    cs = Pin(int(config["pins"]["lora_cs"]))
    irq = Pin(int(config["pins"]["lora_dio1"]))
    rst = Pin(int(config["pins"]["lora_rst"]))
    gpio = Pin(int(config["pins"]["lora_busy"]))

    frequency = int(float(config["lora"]["freq"]) * 1000)
    bandwidth = 500
    spreading_factor = 12
    coding_rate = 8
    preamble_length = 12
    tx_power = -5
    use_CRC = False

    lora_cfg = {
        "freq_khz": frequency,
        "sf": spreading_factor,
        "bw": bandwidth,  # kHz
        "coding_rate": coding_rate,
        "preamble_len": preamble_length,
        "output_power": tx_power,  # dBm
        "crc_en": use_CRC,
    }

    radio = SX1262(
        spi=SPI(
            spi_bus,
            baudrate=baud_rate,
            polarity=polarity,
            phase=phase,
            miso=miso,
            mosi=mosi,
            sck=clk,
        ),
        cs=cs,
        busy=gpio,
        dio1=irq,
        reset=rst,
        dio3_tcxo_millivolts=tcxo_mv,
        lora_cfg=lora_cfg,
    )

    encryption_key = unhexlify(config["lora"]["key"])
    encryptor = cryptolib.aes(encryption_key, 1)

    inp = float(input("Input a floating-point number to send to the Big Buddy: "))
    payload = struct.pack(">fHHBHHB", inp, 0, 0, ord('X'), 0, 0, ord('X'))
    pad = [random.getrandbits(8), random.getrandbits(8)]
    message = encryptor.encrypt(payload + bytes(pad))
    radio.send(message)


def _test_wifi_ftp() -> None:
    import network, uftpd

    default = 60
    duration = input(
        f"How long (in seconds) should the WiFi and FTP Server be on? [{default}]: "
    )
    if duration == "":
        duration = default
    else:
        duration = int(duration)

    ap_if = network.WLAN(network.AP_IF)
    time.sleep_ms(10)  # Give things a chance to settle
    ap_if.active(True)
    ap_if.config(ssid=config["wifi"]["name"], security=3, key=config["wifi"]["key"])
    while ap_if.active() == False:
        time.sleep_ms(10)  # Give things a chance to settle
    uftpd.stop()
    uftpd.start(verbose=True, splash=True)
    print(
        f"You can now join the WiFi network '{config["wifi"]["name"]}' using the password '{config["wifi"]["key"]}' for the next {duration} seconds. The FTP Server does not use a username or password."
    )
    time.sleep(duration)
    uftpd.stop()
    ap_if.active(False)


def _test_beeper() -> None:
    from machine import PWM, Pin

    default = 10
    duration = input(
        f"How long (in seconds) should the buzzer sound for (Warning: Loud) [{default}]: "
    )
    if duration == "":
        duration = default
    else:
        duration = int(duration)
    p1 = PWM(Pin(config["pins"]["buzzer1"]), freq=config["system"]["buzzer_freq_1"], duty_u16=32768)
    p2 = PWM(Pin(config["pins"]["buzzer2"]), freq=config["system"]["buzzer_freq_1"], duty_u16=32768, invert=True)
    while duration > 0:
        if p1.freq() == config["system"]["buzzer_freq_1"]:
            p1.freq(config["system"]["buzzer_freq_2"])
            p2.freq(config["system"]["buzzer_freq_2"])
        else:
            p1.freq(config["system"]["buzzer_freq_1"])
            p2.freq(config["system"]["buzzer_freq_1"])
        time.sleep(min(3, duration))
        duration -= 3
    p1.deinit()
    p2.deinit()


def _print_gps() -> None:
    from gps import GPS

    gps = GPS(
        config["pins"]["uart_tx"],
        config["pins"]["uart_rx"],
        config["system"]["gps_pmtk_cmds"],
    )
    gps.initialize()

    default = 5
    duration = input(
        f"Maximum time (in minutes) to wait for a GPS signal? [{default}]: "
    )
    if duration == "":
        duration = default
    else:
        duration = int(duration)

    gps_wait_time = duration * 60 * 1000  # minutes * seconds * milliseconds
    init_time = time.ticks_ms()
    while (time.ticks_diff(time.ticks_ms(), init_time) < gps_wait_time) and (
        gps.buffer == None or not hasattr(gps, "valid") or gps.valid == False
    ):

        gps.clear_buffer()
        time.sleep_ms(100)
        gps.read_raw()

        if gps.buffer != None:
            gps.decode_reading(gps.buffer)

    lat = gps.lat  # ddmm.mmmm
    latdd = int(lat[0:2])
    latmm = float(lat[2:])
    lon = gps.lon  # dddmm.mmmm
    londd = int(lon[0:3])
    lonmm = float(lon[3:])
    print(
        f"Your location is {latdd}°{latmm}{str(gps.latNS.decode("utf-8"))} {londd}°{lonmm}{str(gps.lonEW.decode("utf-8"))}. It is {gps.year}-{gps.month:02d}-{gps.day:02d}T{gps.hour:02d}:{gps.minute:02d}:{gps.second:02d} UTC."
    )


def _write_config() -> None:
    with open("/config.json", "w") as f:
        json.dump(config, f)
    print("Configuration saved.")


print("")
print("Nene Configuration Generator")
print("============================")
config = _load_existing_config()
_configure_pins(config)

print("")
while True:
    print("Options:")
    print("\t1. Update Basic Configuartion")
    print("\t2. Update Sensor Calibration")
    print("\t3. Display Configuration")
    print("\t4. Attitude and Heading Reference System Test")
    print("\t5. Send Message via LoRa")
    print("\t6. Turn on WiFi and FTP Server")
    print("\t7. Test Beeper")
    print("\t8. Print GPS Coordinates")
    print("\t9. Save Configuration / Calibration Data and Exit")
    val_ok = False
    while not val_ok:
        try:
            inp = int(input(f"Input selection: "))
            val_ok = True
        except ValueError as verr:
            print("Bad input (can't convert to int)")
    if inp == 1:
        _basic_config(config)
    elif inp == 2:
        _sensor_calibration(config)
    elif inp == 3:
        _print_config()
    elif inp == 4:
        _test_AHRS()
    elif inp == 5:
        _send_lora_msg()
    elif inp == 6:
        _test_wifi_ftp()
    elif inp == 7:
        _test_beeper()
    elif inp == 8:
        _print_gps()
    elif inp == 9:
        _write_config()
        break
