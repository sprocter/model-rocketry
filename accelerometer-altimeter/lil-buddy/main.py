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
from collections import deque, namedtuple
from micropython import RingIO, schedule
from math import sqrt

from bmp581 import BMP581
from adxl375 import ADXL375
from sx1262 import SX1262

import time, gc, machine, network, esp32, ftp, json

_MODE_INITIALIZE = const(0)
_MODE_LAUNCHPAD = const(1)
_MODE_ASCENT = const(2)
_MODE_DESCENT = const(3)
_MODE_TOUCHDOWN = const(4)
_MODE_FINISHED = const(5)
_MODE_WIFI = const(6)

_SENSOR_FREQ_HZ = const(25)

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

boot_button = Pin(0, Pin.IN, Pin.PULL_UP)

####################################
# BEGIN Interrupt Service Routines #
####################################


def toggle_buzzer_freq(timer: Timer) -> None:
    global p1, p2
    if p1.freq() == 4798:
        p1.freq(4000)
        p2.freq(4000)
    else:
        p1.freq(4800)
        p2.freq(4800)


def get_sensor_readings(timer: Timer) -> None:
    alti.read_raw()
    accel.read_raw()

    # unstored_readings.write(time.ticks_diff(time.ticks_ms(), launch_time_ms).to_bytes(3, "little"))
    # unstored_readings.write(alti.buffer)
    # unstored_readings.write(accel.buffer)

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
    global debounce_time
    # debounce
    if time.ticks_diff(time.ticks_ms(), debounce_time) < 400:
        return
    debounce_time = time.ticks_ms()
    if mode == _MODE_FINISHED:
        schedule(share_files, None)


##################################
# END Interrupt Service Routines #
##################################


def enable_buzzer() -> None:
    global p1, p2, buzzer_timer
    buzzer_timer = Timer(3)
    buzzer_timer.init(mode=Timer.PERIODIC, period=3000, callback=toggle_buzzer_freq)
    p1 = PWM(Pin(18), freq=4800, duty_u16=32768)
    p2 = PWM(Pin(9), freq=4800, duty_u16=32768, invert=True)


def enable_sensor_recording() -> None:
    global sensor_reading_timer
    sensor_reading_timer = Timer(0)
    sensor_reading_timer.init(
        mode=Timer.PERIODIC, freq=25, callback=get_sensor_readings
    )


def enable_radio() -> None:
    global radio_timer
    radio_timer = Timer(2)
    radio_timer.init(mode=Timer.PERIODIC, period=10000, callback=send_radio_message)


def process_reading(reading: tuple[bytes, bytearray, bytearray]) -> None:
    global nvs, reading_num, recent_altis, apogee
    timestamp = int.from_bytes(reading[0], "little")
    altitude = alti.decode_reading(reading[1]) - initial_altitude
    (acc_x, acc_y, acc_z) = accel.decode_reading(reading[2])
    acc = sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    packed_reading = pack(">ifffff", timestamp, altitude, acc, acc_x, acc_y, acc_z)
    if mode == _MODE_ASCENT and altitude > apogee:
        apogee = altitude
    recent_altis.append(altitude)
    if mode == _MODE_LAUNCHPAD:
        ground_readings.append(packed_reading)
    elif mode == _MODE_ASCENT or mode == _MODE_DESCENT:
        nvs.set_blob(str(reading_num), packed_reading)
        reading_num += 1


def update_mode(recent_altis: deque) -> None:
    global mode
    if mode == _MODE_LAUNCHPAD and have_liftoff(recent_altis, initial_altitude):
        ascent()
    elif (mode == _MODE_ASCENT or mode == _MODE_LAUNCHPAD) and started_descent(
        recent_altis, apogee
    ):
        # We can go straight to descent from launchpad mode because mid-air reboots happen, unfortunately. We don't want to lose data.
        descent()
    elif mode == _MODE_DESCENT and touched_down(recent_altis, apogee, initial_altitude):
        touchdown()


def send_message(arg=None) -> None:
    lora.send("testing...", secrets["bigbuddy-addr"])


def _update_neopixel() -> None:
    neopixel[0] = _MODE_TO_LED[mode]
    neopixel.write()


def _initialize_nvs() -> None:
    """Creates a new NVS namespace in the "nvs" global variable

    This sets the `nvs` global variable to an instance of the "Non-Volatile Storage" class. This is ESP-specific functionality that implements a flash-backed keystore. The instance will be configured with a new / empty (except for a sentinel value) namespace. It will also set the `launch_num` variable so the file i/o stuff knows which launch number this was.
    """
    global nvs, launch_num
    # There seems to be no way to check if a namespace exists, so we just check
    # for a sentinel value until we get an OSError. I don't love using
    # exceptions for control flow, but here we are.
    launch_num = 1
    while True:
        try:
            nvs = esp32.NVS(str(launch_num))
            nvs.get_i32(str(f"namespace {launch_num}"))
        except OSError:
            nvs.set_i32(str(f"namespace {launch_num}"), launch_num)
            nvs.commit()
            break
        launch_num += 1


def _init_sensors() -> None:
    global accel, alti
    i2c = I2C(scl=40, sda=41)
    accel = ADXL375(i2c)
    alti = BMP581(i2c)

    accel.initialize()
    alti.initialize()


def initialize():
    global mode, unstored_readings, reading_num, lora, initial_altitude, apogee, neopixel, launch_time_ms, debounce_time, secrets, ground_readings, recent_altis, init_time

    mode = _MODE_INITIALIZE

    # Status LEDs #
    neopixel_pwr_pin = Pin(38, Pin.OUT)
    neopixel_pwr_pin.on()
    neopixel_pin = Pin(39, Pin.OUT)
    neopixel = NeoPixel(neopixel_pin, 1)

    _update_neopixel()

    launch_time_ms = time.ticks_ms()
    debounce_time = launch_time_ms
    init_time = launch_time_ms

    vfs.mount(vfs.VfsLfs2(bdev, readsize=2048, progsize=256, lookahead=256, mtime=False), "/")  # type: ignore

    with open("/secrets.json", "r") as f:
        secrets = json.loads(f.read())

    _init_sensors()

    _initialize_nvs()

    lora = SX1262(1, 2, 3, 4, 5, 6, 7, 8)

    boot_button.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

    reading_num = _LAUNCHPAD_READINGS + 1
    alti.read_raw()
    initial_altitude = alti.decode_reading(alti.buffer)
    apogee = initial_altitude

    mode = _MODE_LAUNCHPAD
    _update_neopixel()

    ground_readings = deque([], _LAUNCHPAD_READINGS)
    recent_altis = deque([], _RECENT_READINGS)
    gc.collect()
    print(gc.mem_free())
    enable_sensor_recording()


def have_liftoff(recent_altis: deque, ground_level: float) -> bool:
    threshold = ground_level + _ALTITUDE_DIFFERENCE
    return sum(i > threshold for i in recent_altis) > _CONFIRMATORY_READINGS


def started_descent(recent_altis: deque, apogee: float) -> bool:
    threshold = apogee - _ALTITUDE_DIFFERENCE
    return sum(i < threshold for i in recent_altis) > _CONFIRMATORY_READINGS


def touched_down(recent_altis: deque, apogee: float, ground_level: float) -> bool:
    if apogee < (ground_level + _ALTITUDE_DIFFERENCE):
        return False  # If we haven't gone up at least _A_D
    if max(recent_altis) > (apogee - _ALTITUDE_DIFFERENCE):
        return False  # If we haven't fallen at least _A_D from apogee
    n = len(recent_altis)
    u = sum(recent_altis) / n
    u2 = u**2
    variance = (1 / n) * sum([x**2 - 2 * u * x + u2 for x in recent_altis])
    return abs(variance) < 0.001


def ascent() -> None:
    global launch_time_ms, mode
    # Nothing really to do here -- sensors are already on, and the switch to NVS is handled by the process_reading function
    launch_time_ms = time.ticks_ms()
    mode = _MODE_ASCENT
    _update_neopixel()


def descent() -> None:
    global mode
    mode = _MODE_DESCENT
    _update_neopixel()
    enable_buzzer()
    enable_radio()


def share_files(arg=None) -> None:
    """Turns on wifi and an FTP server

    This will turn the device into a Wi-Fi access point (using the SSID and password from the file "secrets.json"), and then turn on a single-user FTP server (no username or password). After the user disconnects from that FTP server, this will return.
    """
    global mode
    mode = _MODE_WIFI
    _update_neopixel()

    radio_timer.deinit()
    buzzer_timer.deinit()
    p1.deinit()
    p2.deinit()

    ap_if = network.WLAN(network.AP_IF)
    ap_if.active(True)
    time.sleep_ms(10)  # Give things a chance to settle
    ap_if.config(ssid=secrets["wifi-ssid"], security=3, key=secrets["wifi-key"])
    ftp.ftpserver()


def _write_initial_data() -> None:
    time_offset_ms = time.ticks_diff(launch_time_ms, init_time)
    adjusted_ground_readings = []
    while len(ground_readings) > 0:
        entry = ground_readings.popleft()
        unpacked = list(unpack(">ifffff", entry))
        unpacked[0] = unpacked[0] - time_offset_ms
        adjusted_ground_readings.append(pack(">ifffff", *unpacked))
    header_str = "timestamp, altitude, acceleration, acc_x, acc_y, acc_z\n"
    _write_data(list(adjusted_ground_readings), header_str)


def _write_nvs_data() -> None:
    first_entry = _LAUNCHPAD_READINGS + 1
    if reading_num - first_entry > 100:
        last_entry = first_entry + 100
    else:
        last_entry = reading_num

    while True:
        packed_readings = []
        for i in range(first_entry, last_entry):
            packed_readings.append(bytearray(24))
            nvs.get_blob(str(i), packed_readings[-1])

        _write_data(packed_readings)

        if last_entry != reading_num:
            first_entry = last_entry
            if reading_num - first_entry > 100:
                last_entry = first_entry + 100
            else:
                last_entry = reading_num
        else:
            break


def _write_data(
    packed_readings: list[bytearray], header_str: Optional[str] = None
) -> None:
    with open(f"launch-{launch_num}.csv", "at") as f:
        if header_str is not None:
            f.write(header_str)
        for packed_reading in packed_readings:
            f.write(", ".join(str(x) for x in unpack(">ifffff", packed_reading)) + "\n")


def _wipe_nvs_namespace() -> None:
    """Wipe all sensor readings from non-volatile storage to save space.

    This removes all sensor readings from non-volatile storage (NVS). Note that it leaves the sentinel value so this launch's namespace will not be re-used. This avoids filename conflicts since we use the launch number in both the namespace and final csv filename.
    """
    first_entry = _LAUNCHPAD_READINGS + 1
    last_entry = reading_num
    for i in range(first_entry, last_entry):
        if i % 100 == 0:
            # Things seem to bog down with longer erases, I'm not sure why
            nvs.commit()
            gc.collect()
            time.sleep(1)
        nvs.erase_key(str(i))


def touchdown() -> None:
    global mode

    # Wait a final few seconds then remove timer and turn off sensors
    time.sleep(_INITIAL_FINAL_TIME_SEC)
    mode = _MODE_TOUCHDOWN
    _update_neopixel()

    sensor_reading_timer.deinit()
    # # TODO: Add shutoff to sensors?

    gc.collect()

    _write_initial_data()
    _write_nvs_data()
    startwipe = time.ticks_ms()
    _wipe_nvs_namespace()
    print(f"Erase took {time.ticks_diff(time.ticks_ms(), startwipe)}ms.")

    mode = _MODE_FINISHED
    _update_neopixel()


initialize()
time.sleep(5)
ascent()
time.sleep(10)
touchdown()
