"""A script for a model rocket altimeter / accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import PWM, Pin, I2C, Timer
from struct import pack, unpack
from neopixel import NeoPixel
from collections import deque
from micropython import schedule, const
from math import sqrt

from bmp581 import BMP581
from adxl375 import ADXL375
from icm20649 import ICM20649
from pa1010 import PA1010
from sx1262 import SX1262
from kalman import StateEstimator

import time, gc, json, vfs, machine, network, uftpd
import adxl375, icm20649

_MODE_INITIALIZE = const(0)
_MODE_LAUNCHPAD = const(1)
_MODE_ASCENT = const(2)
_MODE_DESCENT = const(3)
_MODE_TOUCHDOWN = const(4)
_MODE_FINISHED = const(5)
_MODE_WIFI = const(6)

_SENSOR_FREQ_HZ = const(25)
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

####################################
# BEGIN Interrupt Service Routines #
####################################


def toggle_buzzer_freq(timer: Timer) -> None:
    global p1, p2
    if p1.freq() == 5000:
        p1.freq(2000)
        p2.freq(2000)
    else:
        p1.freq(5000)
        p2.freq(5000)


def get_sensor_readings(timer: Timer) -> None:
    alti.read_raw()
    accel.read_raw()

    schedule(
        process_reading,
        (
            time.ticks_diff(time.ticks_ms(), launch_time_ms).to_bytes(3, "little"),
            alti.buffer,
            accel.buffer,
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
    p1 = PWM(Pin(6), freq=5000, duty_u16=32768)
    p2 = PWM(Pin(5), freq=5000, duty_u16=32768, invert=True)


def enable_sensor_recording() -> None:
    sensor_reading_timer.init(
        mode=Timer.PERIODIC, freq=25, callback=get_sensor_readings
    )


def enable_radio() -> None:
    radio_timer.init(mode=Timer.PERIODIC, period=60000, callback=send_radio_message)


def process_reading(reading: tuple[bytes, bytearray, bytearray]) -> None:
    global reading_num, recent_altis, apogee
    start_timestamp = time.ticks_us()
    timestamp = int.from_bytes(reading[0], "little")
    sensor_altitude = alti.decode_reading(reading[1]) - initial_altitude
    estimator.altitude = sensor_altitude # Give the estimator our sensor reading
    estimated_altitude = estimator.altitude # Use the estimated altitude
    (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp) = accel.decode_reading(
        reading[2]
    )
    estimator.acceleration = acc_y
    packed_reading = pack(
        ">iffffffff",
        timestamp,
        sensor_altitude,
        acc_x,
        acc_y,
        acc_z,
        gyro_x,
        gyro_y,
        gyro_z,
        temp,
    )
    if mode == _MODE_ASCENT and sensor_altitude > apogee:
        apogee = estimated_altitude
    recent_altis.append(estimated_altitude)
    update_mode()
    if mode == _MODE_LAUNCHPAD:
        ground_readings.append(packed_reading)
    elif mode == _MODE_ASCENT or mode == _MODE_DESCENT or mode == _MODE_TOUCHDOWN:
        # TODO: Store reading in memory
        # nvs.set_blob(str(reading_num), packed_reading)
        reading_num += 1
    end_timestamp = time.ticks_us()
    print(timestamp)


def update_mode() -> None:
    global mode
    if mode == _MODE_LAUNCHPAD and have_liftoff(recent_altis):
        ascent()
    elif (mode == _MODE_ASCENT or mode == _MODE_LAUNCHPAD) and started_descent(
        recent_altis, apogee
    ):
        # We can go straight to descent from launchpad mode because mid-air reboots happen, unfortunately. We don't want to lose data.
        descent()
    elif mode == _MODE_DESCENT and touched_down(recent_altis, apogee):
        mode = _MODE_TOUCHDOWN
        _update_neopixel()
        # Wait a final few seconds then remove timer and turn off sensors
        touchdown_timer.init(mode=Timer.ONE_SHOT, period=3000, callback=touchdown)


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
        payload = pack(">d", apogee - initial_altitude)
    elif hdr_flags == 1:
        gps.update()
        lat_str = gps.lat
        lat_dir = ord(gps.latNS)
        lon_str = gps.lon
        lon_dir = ord(gps.lonEW)
        lat_elems = [int(x) for x in lat_str.split(".")]
        lon_elems = [int(x) for x in lon_str.split(".")]
        payload_elems = lat_elems + [lat_dir] + lon_elems + [lon_dir]
        payload = pack(">HHBHHB", *payload_elems)

    radio.send(msg_header + payload)

    msg_id += 1


def _update_neopixel() -> None:
    neopixel[0] = _MODE_TO_LED[mode]
    neopixel.write()


def _init_radio():
    global msg_header, msg_id
    radio = SX1262(1, 36, 35, 37, 8, 18, 9, 17)

    frequency = 917.0
    bandwidth = 125
    spreading_factor = 10
    coding_rate = 8
    sync_word = 0x12  # private
    tx_power = 22  # -5 # 22
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
    msg_id = 0
    hdr_to = secrets["bigbuddy-addr"]
    hdr_from = secrets["lilbuddy-addr"]
    msg_header = bytearray(4)
    msg_header[0] = hdr_to
    msg_header[1] = hdr_from

    return radio


def _init_devices() -> None:
    global accel, alti, gyro, gps, radio, clock, _GPS_CONNECTED
    i2c = I2C(scl=9, sda=8)
    connected_devices = i2c.scan()
    
    alti = BMP581(i2c)
    
    if adxl375.ADXL375_ADDR in connected_devices:
        accel = ADXL375(i2c)  # Use the ADXL if we have multiple accelerometers
    elif icm20649.ICM20649_ADDR in connected_devices:
        accel = ICM20649(i2c)

    if icm20649.ICM20649_ADDR in connected_devices:
        if isinstance(accel, ICM20649):
            gyro = accel  # Share object rather than re-create it
        else:
            gyro = ICM20649(i2c)
    
    accel.initialize()
    alti.initialize()

    # radio = _init_radio()

        
    if PA1010.I2C_ADDR in connected_devices:
        _GPS_CONNECTED = True
        clock = RTC()
        gps = PA1010(i2c)
        gps.set_update_rate(1)
        while not gps.update():
            time.sleep(5)
        gps.update()  # We have a fix but it could be up to 5s outdated
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


def _init_board():
    global radio_timer, sensor_reading_timer, buzzer_timer, button_pressed, touchdown_timer, neopixel

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

    # TODO: Since we don't do any reading or writing in-flight, move this to touchdown?
    vfs.mount(vfs.VfsLfs2(bdev, readsize=2048, progsize=256, lookahead=256, mtime=False), "/")  # type: ignore


def initialize():
    global mode, reading_num, radio, initial_altitude, apogee, launch_time_ms, debounce_time, secrets, ground_readings, recent_altis, init_time, estimator

    mode = _MODE_INITIALIZE

    init_time = time.ticks_ms()
    launch_time_ms = init_time  # launch time will be reset when liftoff is detected

    _init_board()

    with open("/secrets.json", "r") as f:
        secrets = json.loads(f.read())

    _init_devices()

    estimator = StateEstimator(_PERIOD, alti.error, accel.error)

    reading_num = _LAUNCHPAD_READINGS + 1
    alti.read_raw()
    initial_altitude = alti.decode_reading(alti.buffer)
    apogee = 0.0

    ground_readings = deque([], _LAUNCHPAD_READINGS)
    recent_altis = deque([], _RECENT_READINGS)

    gc.collect()

    enable_sensor_recording()

    mode = _MODE_LAUNCHPAD
    _update_neopixel()


def have_liftoff(recent_altis: deque) -> bool:
    threshold = _ALTITUDE_DIFFERENCE
    return sum(i > threshold for i in recent_altis) > _CONFIRMATORY_READINGS


def started_descent(recent_altis: deque, apogee: float) -> bool:
    threshold = apogee - _ALTITUDE_DIFFERENCE
    return sum(i < threshold for i in recent_altis) > _CONFIRMATORY_READINGS


def touched_down(recent_altis: deque, apogee: float) -> bool:
    if apogee < _ALTITUDE_DIFFERENCE:
        return False  # If we haven't gone up at least _A_D
    if max(recent_altis) > (apogee - _ALTITUDE_DIFFERENCE):
        return False  # If we haven't fallen at least _A_D from apogee
    n = len(recent_altis)
    u = sum(recent_altis) / n
    u2 = u**2
    variance = (1 / n) * sum([x**2 - 2 * u * x + u2 for x in recent_altis])
    return abs(variance) < 0.001


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
    enable_buzzer()


def _write_initial_data() -> None:
    time_offset_ms = time.ticks_diff(launch_time_ms, init_time)
    adjusted_ground_readings = []
    while len(ground_readings) > 0:
        entry = ground_readings.popleft()
        unpacked = list(unpack(">iffffffff", entry))
        unpacked[0] = unpacked[0] - time_offset_ms
        adjusted_ground_readings.append(pack(">iffffffff", *unpacked))
    header_str = (
        "timestamp, altitude, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp\n"
    )
    if _GPS_CONNECTED:
        header_str = str(launch_time_ymdwhms) + "\n" + header_str
    _write_data(list(adjusted_ground_readings), header_str)


def _write_data(
    packed_readings: list[bytearray], header_str: Optional[str] = None
) -> None:
    with open(f"launch-{launch_num}.csv", "at") as f:
        if header_str is not None:
            f.write(header_str)
        for packed_reading in packed_readings:
            f.write(
                ", ".join(str(x) for x in unpack(">iffffffff", packed_reading)) + "\n"
            )


def touchdown(timer: Timer) -> None:
    global mode
    sensor_reading_timer.deinit()
    enable_radio()  # The radio lags out sensors, so we can't turn it on until we're on the ground
    # TODO: Add shutoff to sensors?

    gc.collect()

    _write_initial_data()

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
time.sleep(5)
# ascent()
# time.sleep(5)
# descent()
# time.sleep(5)
# mode = _MODE_TOUCHDOWN
# _update_neopixel()
# touchdown(None)
while True:
    time.sleep(5)
