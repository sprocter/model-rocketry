"""A script for a model rocket altimeter / accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025-2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import PWM, Pin, I2C, Timer, RTC
from struct import pack, unpack
from neopixel import NeoPixel
from collections import deque
from micropython import schedule, const
from esp32 import mcu_temperature
from math import sqrt

from bmp581 import BMP581
from adxl375 import ADXL375
from icm20649 import ICM20649
from mmc5983ma import MMC5983MA
from max17048 import MAX17048
from pa1010 import PA1010
from sx1262 import SX1262
from marg import StateEstimator

import time, gc, json, vfs, machine, network, uftpd, os, deflate
import adxl375, icm20649
import hidden_buffer as buff


_MODE_INITIALIZE = const(0)
_MODE_LAUNCHPAD = const(1)
_MODE_ASCENT = const(2)
_MODE_DESCENT = const(3)
_MODE_TOUCHDOWN = const(4)
_MODE_FINISHED = const(5)
_MODE_WIFI = const(6)

_SENSOR_FREQ_HZ = const(45)
_PERIOD = const(1000 / _SENSOR_FREQ_HZ)

# Number of seconds to record before launch / after touchdown
_INITIAL_FINAL_TIME_SEC = const(3)

_LAUNCHPAD_READINGS = const(_INITIAL_FINAL_TIME_SEC * _SENSOR_FREQ_HZ)

# How far above ground level / below apogee we should be before confirming mode change
_ALTITUDE_DIFFERENCE = const(20)

# How many readings to consider when changing modes
_RECENT_READINGS = const(5)

# How many of those readings should be above (for liftoff) or below (for descent)
_CONFIRMATORY_READINGS = const(4)

# LED Control #
_NPXL_BRIGHTNESS = const(10)  # 1-255
_NPXL_OFF = (0, 0, 0)
_NPXL_WHT = (_NPXL_BRIGHTNESS, _NPXL_BRIGHTNESS, _NPXL_BRIGHTNESS)
_NPXL_RED = (_NPXL_BRIGHTNESS, 0, 0)
_NPXL_ORA = (_NPXL_BRIGHTNESS, _NPXL_BRIGHTNESS // 2, 0)
_NPXL_YLW = (_NPXL_BRIGHTNESS, _NPXL_BRIGHTNESS, 0)
_NPXL_GRN = (0, _NPXL_BRIGHTNESS, 0)
_NPXL_CYA = (0, _NPXL_BRIGHTNESS, _NPXL_BRIGHTNESS)
_NPXL_BLU = (0, 0, _NPXL_BRIGHTNESS)
_NPXL_PUR = (_NPXL_BRIGHTNESS // 2, 0, _NPXL_BRIGHTNESS)

_MODE_TO_LED = {
    _MODE_INITIALIZE: _NPXL_WHT,
    _MODE_LAUNCHPAD: _NPXL_RED,
    _MODE_ASCENT: _NPXL_ORA,
    _MODE_DESCENT: _NPXL_GRN,
    _MODE_TOUCHDOWN: _NPXL_CYA,
    _MODE_FINISHED: _NPXL_BLU,
    _MODE_WIFI: _NPXL_PUR,
}

_GPS_CONNECTED = False

# 2M Floats is 8MiB, which is how much PSRAM we have.
# This gives us over half an hour of data storing 22 values per reading, 45
# times a second.
_BUFFER_SIZE = const(2_000_000)

####################################
# BEGIN Interrupt Service Routines #
####################################


def toggle_buzzer_freq(timer: Timer) -> None:
    global p1, p2
    if p1.freq() == 5198:
        p1.freq(1800)
        p2.freq(1800)
    else:
        p1.freq(5200)
        p2.freq(5200)


def get_sensor_readings(timer: Timer) -> None:
    global previous_gps_read_ts

    timestamp = time.ticks_diff(time.ticks_ms(), launch_time_ms)
    alti.read_raw()
    accel.read_raw()
    mag.read_raw()

    if timestamp - previous_gps_read_ts > 100:
        previous_gps_read_ts = timestamp
        if _GPS_CONNECTED:
            gps.read_raw()

    schedule(
        process_reading,
        (
            timestamp.to_bytes(3, "little"),
            alti.buffer,
            mag.buffer,
            accel.buffer,
            gps.buffer,
        ),
    )


def send_radio_message(timer: Timer) -> None:
    schedule(send_message, None)


def button_handler(boot_button: Pin) -> None:
    global button_pressed
    if button_pressed:
        return
    button_pressed = True
    buzzer_timer.deinit()
    radio_timer.deinit()
    share_files()


##################################
# END Interrupt Service Routines #
##################################


def enable_buzzer() -> None:
    global p1, p2
    buzzer_timer.init(mode=Timer.PERIODIC, period=3000, callback=toggle_buzzer_freq)
    p1 = PWM(Pin(17), freq=5200, duty_u16=32768)
    p2 = PWM(Pin(18), freq=5200, duty_u16=32768, invert=True)


def enable_sensor_recording() -> None:
    sensor_reading_timer.init(
        mode=Timer.PERIODIC, freq=_SENSOR_FREQ_HZ, callback=get_sensor_readings
    )


def enable_radio() -> None:
    radio_timer.init(mode=Timer.PERIODIC, period=60000, callback=send_radio_message)


@micropython.native
def process_reading(
    reading: tuple[bytes, bytearray, bytearray, bytearray, tuple],
) -> None:
    global reading_num, gps_reading_count, recent_altis, apogee

    timestamp = int.from_bytes(reading[0], "little")
    (raw_altitude, ambient_temp) = alti.decode_reading(reading[1])
    barometric_altitude = raw_altitude - initial_altitude
    mag_rdg = mag.decode_mag(reading[2])
    estimator.magnetometer = mag_rdg
    acc_rdg = accel.decode_accel(reading[3])
    estimator.acceleration = acc_rdg
    gyro_rdg = gyro.decode_gyro(reading[3])
    estimator.gyroscope = gyro_rdg
    estimator.altitude = barometric_altitude  # TODO: Shouldn't this be last? Should probably manually trigger computation
    estimated_altitude = estimator.altitude  # Use the estimated altitude
    if mode == _MODE_ASCENT and estimated_altitude > apogee:
        apogee = estimated_altitude
    recent_altis.append(estimated_altitude)
    update_mode()
    if _GPS_CONNECTED and timestamp - previous_gps_read_ts > 100:
        gps.decode_reading(gps.buffer)
    if mode == _MODE_LAUNCHPAD:
        packed_reading = pack(
            ">ffffffffffffffffffffff",
            float(timestamp),
            acc_rdg[0],
            acc_rdg[1],
            acc_rdg[2],
            gyro_rdg[0],
            gyro_rdg[1],
            gyro_rdg[2],
            mag_rdg[0],
            mag_rdg[1],
            mag_rdg[2],
            barometric_altitude,
            gps.altitude,
            ambient_temp,
            gps.heading,
            gps.speed,
            float(gps.lat),
            float(gps.lon),
            estimator.heading,
            estimator.pitch,
            estimator.roll,
            estimated_altitude,
            estimator.velocity,
        )
        ground_readings.append(packed_reading)
    elif mode == _MODE_ASCENT or mode == _MODE_DESCENT or mode == _MODE_TOUCHDOWN:
        idx_start = reading_num * 22
        buff.store(idx_start + 0, float(timestamp))
        buff.store(idx_start + 1, acc_rdg[0])
        buff.store(idx_start + 2, acc_rdg[1])
        buff.store(idx_start + 3, acc_rdg[2])
        buff.store(idx_start + 4, gyro_rdg[0])
        buff.store(idx_start + 5, gyro_rdg[1])
        buff.store(idx_start + 6, gyro_rdg[2])
        buff.store(idx_start + 7, mag_rdg[0])
        buff.store(idx_start + 8, mag_rdg[1])
        buff.store(idx_start + 9, mag_rdg[2])
        buff.store(idx_start + 10, barometric_altitude)
        buff.store(idx_start + 11, gps.altitude)
        buff.store(idx_start + 12, ambient_temp)

        buff.store(idx_start + 13, gps.heading)
        buff.store(idx_start + 14, gps.speed)
        # TODO: Should probably check latNS and lonEW once this has been
        # verified to work
        buff.store(idx_start + 15, float(gps.lat))
        buff.store(idx_start + 16, float(gps.lon))

        buff.store(idx_start + 17, estimator.heading)
        buff.store(idx_start + 18, estimator.pitch)
        buff.store(idx_start + 19, estimator.roll)
        buff.store(idx_start + 20, estimated_altitude)
        buff.store(idx_start + 21, estimator.velocity)

        reading_num += 1

    # Garbage collect every third reading, unless we took a GPS reading this
    # period, in which case skip this garbage collection entirely
    if gps_reading_count % 3 == 0 and timestamp - previous_gps_read_ts <= 100:
        gc.collect()
    gps_reading_count += 1


def update_mode() -> None:
    global mode
    if mode == _MODE_LAUNCHPAD and have_liftoff():
        ascent()
    elif mode == _MODE_ASCENT and started_descent():
        descent()
    elif mode == _MODE_DESCENT and touched_down():
        mode = _MODE_TOUCHDOWN
        _update_neopixel()
        # Wait a final few seconds then remove timer and turn off sensors
        touchdown_timer.init(mode=Timer.ONE_SHOT, period=3000, callback=touchdown)


@micropython.native
def have_liftoff() -> bool:
    threshold = _ALTITUDE_DIFFERENCE
    return sum(i > threshold for i in recent_altis) > _CONFIRMATORY_READINGS


@micropython.native
def started_descent() -> bool:
    threshold = apogee - _ALTITUDE_DIFFERENCE
    return sum(i < threshold for i in recent_altis) > _CONFIRMATORY_READINGS


@micropython.native
def touched_down() -> bool:
    if apogee < _ALTITUDE_DIFFERENCE:
        return False  # If we haven't gone up at least _A_D
    if max(recent_altis) > (apogee - _ALTITUDE_DIFFERENCE):
        return False  # If we haven't fallen at least _A_D from apogee
    n = len(recent_altis)
    u = sum(recent_altis) / n
    u2 = u**2
    variance = (1 / n) * sum([x**2 - 2 * u * x + u2 for x in recent_altis])
    return abs(variance) < 0.001


def send_message(arg=None) -> None:
    global msg_id, msg_header

    hdr_id = msg_id % 255
    if _GPS_CONNECTED:
        hdr_flags = msg_id % 2
    else:
        hdr_flags = 0
    msg_header[2] = hdr_id
    msg_header[3] = hdr_flags

    if hdr_flags == 0:
        payload = pack(">d", apogee)
    elif hdr_flags == 1:
        gps.clear_buffer()
        time.sleep_ms(100)
        gps.read_raw()
        retries = 0  # Avoid hanging if something went wrong with the GPS
        while retries < 30 and gps.buffer[0] == None and gps.buffer[1] == None:
            time.sleep_ms(100)
            gps.read_raw()
            retries += 1

        if retries >= 30 or not gps.valid:
            payload = pack(">HHBHHB", 0, 0, 0, 0, 0, 0)
        else:
            lat_str = gps.lat
            lat_dir = ord(gps.latNS)
            lon_str = gps.lon
            lon_dir = ord(gps.lonEW)
            lat_elems = [int(x) for x in lat_str.split(b".")]
            lon_elems = [int(x) for x in lon_str.split(b".")]
            payload_elems = lat_elems + [lat_dir] + lon_elems + [lon_dir]
            payload = pack(">HHBHHB", *payload_elems)

    radio.send(msg_header + payload)

    msg_id += 1


def _update_neopixel() -> None:
    if mode == _MODE_LAUNCHPAD and _GPS_CONNECTED:
        neopixel[0] = _NPXL_YLW
    else:
        neopixel[0] = _MODE_TO_LED[mode]
    neopixel.write()


def _init_radio():
    global msg_header, msg_id

    spi_bus = 1
    clk = 36
    mosi = 35
    miso = 37
    cs = 5
    irq = 6  # "DIO1"
    rst = 44
    gpio = 43  # "Busy"

    radio = SX1262(spi_bus, clk, mosi, miso, cs, irq, rst, gpio)

    frequency = 917.0
    bandwidth = 125
    spreading_factor = 10
    coding_rate = 8
    sync_word = 0x12  # private
    tx_power = -5  # 22
    mA_limit = 125.0
    implicit_header = False
    use_CRC = False
    use_LDRO = True  # Low Data-Rate Optimizer
    radio.begin(
        freq=frequency,
        bw=bandwidth,
        sf=spreading_factor,
        cr=coding_rate,
        syncWord=sync_word,
        power=tx_power,
        currentLimit=mA_limit,
        implicit=implicit_header,
        crcOn=use_CRC,
    )
    radio.forceLDRO(use_LDRO)
    msg_id = 1
    hdr_to = secrets["bigbuddy-addr"]
    hdr_from = secrets["lilbuddy-addr"]
    msg_header = bytearray(4)
    msg_header[0] = hdr_to
    msg_header[1] = hdr_from

    return radio


def _init_devices() -> None:
    global accel, alti, gyro, mag, gps, radio, batt_monitor, clock, _GPS_CONNECTED
    i2c = I2C(scl=9, sda=8)
    connected_devices = i2c.scan()

    alti = BMP581(i2c)
    mag = MMC5983MA(i2c)
    gps = PA1010(16, 15)

    if adxl375.ADXL375_ADDR in connected_devices:
        accel = ADXL375(i2c)  # Use the ADXL if we have multiple accelerometers
    elif icm20649.ICM20649_ADDR in connected_devices:
        accel = ICM20649(i2c)

    if icm20649.ICM20649_ADDR in connected_devices:
        if isinstance(accel, ICM20649):
            gyro = accel  # Share object rather than re-create it
        else:
            gyro = ICM20649(i2c)

    alti.initialize()
    mag.initialize()
    accel.initialize()
    gps.initialize()

    radio = _init_radio()

    # Even though the battery monitor is on the board, it communicates via I2C
    # so we initialize it here, rather than _init_board
    batt_monitor = MAX17048(i2c)


def _init_board():
    global radio_timer, sensor_reading_timer, buzzer_timer, button_pressed, touchdown_timer, neopixel

    # Clean up memory to reduce issues with fragmentation
    gc.collect()
    # Initialize our giant buffer which will hold our readings
    buff.init_buffer(_BUFFER_SIZE)

    # Initialize the status LED
    neopixel_pin = Pin(40, Pin.OUT)
    neopixel = NeoPixel(neopixel_pin, 1)
    _update_neopixel()

    # Overclock the CPU
    machine.freq(240000000)

    # Initialize timers
    sensor_reading_timer = Timer(0)  # Controls reading from sensors
    touchdown_timer = Timer(1)  # One-shot after landing to get a few extra seconds
    radio_timer = Timer(2)  # Controls radio broadcasts
    buzzer_timer = Timer(3)  # Controls the changing pitch of the buzzer

    # Initialize the "boot" button and register its handler
    boot_button = Pin(0, Pin.IN, Pin.PULL_UP)
    boot_button.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
    button_pressed = False

    # Mount the filesystem so we can read the config json file and 
    # (after touchdown) write the recorded data to flash
    vfs.mount(vfs.VfsLfs2(bdev, readsize=2048, progsize=256, lookahead=256, mtime=False), "/")  # type: ignore


def initialize():
    global mode, reading_num, gps_reading_count, radio, initial_altitude, apogee, launch_time_ms, debounce_time, secrets, ground_readings, recent_altis, init_time, estimator, previous_gps_read_ts, clock, _GPS_CONNECTED

    mode = _MODE_INITIALIZE

    init_time = time.ticks_ms()
    launch_time_ms = init_time  # launch time will be reset when liftoff is detected

    _init_board()

    with open("/secrets.json", "r") as f:
        secrets = json.loads(f.read())

    _init_devices()

    estimator = StateEstimator(_PERIOD, alti.error, accel.error)

    gps_reading_count = 0
    reading_num = 0  # _LAUNCHPAD_READINGS + 1
    alti.read_raw()
    initial_altitude = alti.decode_alti(alti.buffer)
    apogee = 0.0

    ground_readings = deque([], _LAUNCHPAD_READINGS)
    recent_altis = deque([], _RECENT_READINGS)

    gps.clear_buffer()
    time.sleep_ms(100)
    gps.read_raw()
    while gps.buffer[0] == None and gps.buffer[1] == None:
        time.sleep_ms(100)
        gps.read_raw()
    gps.decode_reading(gps.buffer)
    if hasattr(gps, "valid") and gps.valid == True:
        _GPS_CONNECTED = True
        clock = RTC()
        clock.init(
            (
                gps.year,
                gps.month,
                gps.day,
                gps.hour,
                gps.minute,
                gps.second,
                0,  # microsecond, ignored
                0,  # tzinfo, ignored
            )
        )
    else:
        _GPS_CONNECTED = False

    gc.collect()
    gps.clear_buffer()
    previous_gps_read_ts = time.ticks_diff(time.ticks_ms(), init_time)
    enable_sensor_recording()
    gc.disable()

    mode = _MODE_LAUNCHPAD
    _update_neopixel()


def ascent() -> None:
    global launch_time_ms, launch_time_ymdwhms, mode
    # Nothing really to do here -- sensors are already on, and the switch to NVS is handled by the process_reading function
    launch_time_ms = time.ticks_ms()
    if _GPS_CONNECTED:
        launch_time_ymdwhms = clock.datetime()
    mode = _MODE_ASCENT
    _update_neopixel()


def descent() -> None:
    global mode
    mode = _MODE_DESCENT
    _update_neopixel()
    # enable_buzzer()


def _write_data() -> None:
    time_offset_ms = time.ticks_diff(launch_time_ms, init_time)
    adjusted_ground_readings = []
    while len(ground_readings) > 0:
        entry = ground_readings.popleft()
        unpacked = list(unpack(">ffffffffffffffffffffff", entry))
        unpacked[0] -= time_offset_ms
        adjusted_ground_readings.append(pack(">ffffffffffffffffffffff", *unpacked))
    header_str = "time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, baro_alt, gps_alt, temp, gps_heading, gps_speed, lat, lon, est_yaw, est_pitch, est_roll, est_alt, est_speed\n"
    if _GPS_CONNECTED:
        year = launch_time_ymdwhms[0]
        month = launch_time_ymdwhms[1]
        day = launch_time_ymdwhms[2]
        hour = launch_time_ymdwhms[4]
        minute = launch_time_ymdwhms[5]
        second = launch_time_ymdwhms[6]
        header_str = f"{year}-{month:02d}-{day:02d}T{hour:02d}:{minute:02d}:{second:02d}Z \n {header_str}"

    # Expect filenames of the form 'launch-XXXX.csv'
    prev_launches = sorted(list(filter(lambda y: y.startswith('launch-') and y.endswith('.csv.gz'), os.listdir())))
    if len(prev_launches) == 0:
        launch_num = 1
    else:
        launch_num = int(prev_launches[-1][-11:-7]) + 1

    with open(f"launch-{launch_num:04d}.csv.gz", "wb") as f:
        with deflate.DeflateIO(f, deflate.GZIP, 8) as d:
            d.write(header_str.encode('UTF-8'))
            for packed_reading in adjusted_ground_readings:
                d.write(
                    (", ".join(str(x) for x in unpack(">ffffffffffffffffffffff", packed_reading)) + "\n").encode('UTF-8')
                )
            reading = [0.0] * 22
            for i in range(reading_num):
                for j in range(22):
                    reading[j] = buff.retrieve_from(i * 22 + j)
                d.write((", ".join(str(x) for x in reading)+"\n").encode('UTF-8'))

    # if header_str is not None:
    #     print(header_str)
    # for packed_reading in adjusted_ground_readings:
    #     print(
    #         ", ".join(str(x) for x in unpack(">ffffffffffffffffffffff", packed_reading))
    #     )
    # reading = [0.0] * 22
    # for i in range(reading_num):
    #     for j in range(22):
    #         reading[j] = buff.retrieve_from(i * 22 + j)
    #     print(", ".join(str(x) for x in reading))


def touchdown(timer: Timer) -> None:
    global mode
    
    sensor_reading_timer.deinit()
    gc.enable()
    gc.collect()

    # Note that we can't turn the radio on until the sensors are off because it takes so long (~400ms) to send a message
    
    send_message() # Send a message as soon as we hit the ground
    enable_radio() # Send future messages once per minute

    _write_data()

    mode = _MODE_FINISHED
    _update_neopixel()


def share_files() -> None:
    """Turns on wifi and an FTP server

    This will turn the device into a Wi-Fi access point (using the SSID and password from the file "secrets.json"), and then turn on a FTP server (no username or password).
    """

    global mode
    mode = _MODE_WIFI
    _update_neopixel()
    with open("/secrets.json", "r") as f:
        secrets = json.loads(f.read())
    gc.collect()
    ap_if = network.WLAN(network.AP_IF)
    time.sleep_ms(10)  # Give things a chance to settle
    ap_if.active(True)
    ap_if.config(ssid=secrets["ssid"], security=3, key=secrets["key"])
    while ap_if.active() == False:
        time.sleep_ms(10)  # Give things a chance to settle
    gc.collect()
    uftpd.start(splash=False)


initialize()
time.sleep(15)
ascent()
time.sleep(5)
descent()
time.sleep(5)
mode = _MODE_TOUCHDOWN
_update_neopixel()
touchdown(None)
print("Done!")
while True:
    time.sleep(5)
