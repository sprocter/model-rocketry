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

from ulora import LoRa, ModemConfig, SPIConfig
from bno055 import BNO055
from bmp280 import BMP280

import time, gc, machine, network, esp32, ftp, json

_MODE_LAUNCHPAD = const(0)
_MODE_ASCENT = const(1)
_MODE_DESCENT = const(2)
_MODE_TOUCHDOWN = const(3)
_MODE_FINISHED = const(4)

_SENSOR_FREQ_HZ = const(25)
_PRE_LAUNCH_TIME_SEC = const(3)
_GROUND_READINGS = const(_PRE_LAUNCH_TIME_SEC * _SENSOR_FREQ_HZ)

_READING_COUNT = const(19)  # 1 timestamp, 18 sensor readings
_READING_SIZE = const(4 * _READING_COUNT)  # 4 bytes per int (timestamp) & float
_RINGIO_SIZE = const(5 * _READING_SIZE)  # Store up to 5 readings

_BIG_BUDDY_ADDR = const(110)
_LITTLE_BUDDY_ADDR = const(148)

# How far above ground level / below apogee we should be before confirming mode change
_ALTITUDE_DIFFERENCE = const(20)

# How many readings to consider when changing modes
_RECENT_READINGS = const(5)

# How many of those readings should be above (for liftoff) or below (for descent)
_CONFIRMATORY_READINGS = const(4)

# LED Control #
_NEOPIXEL_BRIGHTNESS = const(10)  # 1-255
_NEOPIXEL_OFF = (0, 0, 0)
#                                             # LED Meaning:
_NEOPIXEL_RED = (_NEOPIXEL_BRIGHTNESS, 0, 0)  # Initialization
_NEOPIXEL_ORA = (_NEOPIXEL_BRIGHTNESS, _NEOPIXEL_BRIGHTNESS // 2, 0)  # Ascent
_NEOPIXEL_GRN = (0, _NEOPIXEL_BRIGHTNESS, 0)  # Touchdown
_NEOPIXEL_CYA = (0, _NEOPIXEL_BRIGHTNESS, _NEOPIXEL_BRIGHTNESS)  # Descent
_NEOPIXEL_BLU = (0, 0, _NEOPIXEL_BRIGHTNESS)  # Done
_NEOPIXEL_PUR = (_NEOPIXEL_BRIGHTNESS // 2, 0, _NEOPIXEL_BRIGHTNESS)  # WiFi
_NEOPIXEL_WHT = (
    _NEOPIXEL_BRIGHTNESS,
    _NEOPIXEL_BRIGHTNESS,
    _NEOPIXEL_BRIGHTNESS,
)  # Launchpad

boot_button = Pin(0, Pin.IN, Pin.PULL_UP)

SensorReading = namedtuple(
    "SensorReading",
    (
        "timestamp",
        "altitude",
        "temperature",
        "lin_acc_x",
        "lin_acc_y",
        "lin_acc_z",
        "mag_x",
        "mag_y",
        "mag_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "euler_heading",
        "euler_roll",
        "euler_pitch",
        "quaternion_w",
        "quaternion_x",
        "quaternion_y",
        "quaternion_z",
    ),
)


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
    global unstored_readings, launch_time_ms
    # TODO: Write custom getter that burst reads relevant values
    unstored_readings.write(
        take_readings(time.ticks_diff(time.ticks_ms(), launch_time_ms))
    )
    schedule(process_reading, None)


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
    global p1, p2
    buzzer_timer = Timer(3)
    buzzer_timer.init(mode=Timer.PERIODIC, period=3000, callback=toggle_buzzer_freq)
    p1 = PWM(Pin(18), freq=4800, duty_u16=32768)
    p2 = PWM(Pin(9), freq=4800, duty_u16=32768, invert=True)


def enable_sensor_recording() -> None:
    sensor_reading_timer = Timer(0)
    sensor_reading_timer.init(
        mode=Timer.PERIODIC, freq=25, callback=get_sensor_readings
    )


def enable_radio() -> None:
    radio_timer = Timer(2)
    radio_timer.init(mode=Timer.PERIODIC, period=10000, callback=send_radio_message)


def process_reading(arg=None) -> None:
    global nvs, reading_num, recent_altis, apogee
    if unstored_readings.any() < _READING_SIZE:
        return
    packed_reading = unstored_readings.read(_READING_SIZE)
    unpacked_reading = SensorReading(*unpack(">iffffffffffffffffff", packed_reading))
    if mode == _MODE_ASCENT and unpacked_reading.altitude > apogee:
        apogee = unpacked_reading.altitude
    recent_altis.append(unpacked_reading.altitude)
    if mode == _MODE_LAUNCHPAD:
        ground_readings.append(unpacked_reading)
    else:
        nvs.set_blob(str(reading_num), packed_reading)
        reading_num += 1


def update_mode(recent_altis: deque) -> None:
    global mode
    # Mid-air reboots happen, unfortunately. We don't want to lose data
    if mode == _MODE_LAUNCHPAD and have_liftoff(recent_altis, initial_altitude):
        mode = _MODE_ASCENT
        ascent()
    elif (mode == _MODE_ASCENT or mode == _MODE_LAUNCHPAD) and started_descent(
        recent_altis, apogee
    ):
        mode = _MODE_DESCENT
        descent()
    elif mode == _MODE_DESCENT and touched_down(recent_altis, apogee, initial_altitude):
        mode = _MODE_TOUCHDOWN
        touchdown()


def send_message(arg=None) -> None:
    lora.send("testing...", _BIG_BUDDY_ADDR)


def take_readings(timestamp: int) -> bytes:
    mag = imu.mag()
    lin_acc = imu.lin_acc()
    gyro = imu.gyro()
    euler = imu.euler()
    temp2 = bmp.temperature
    alti = bmp.altitude
    quaternion = imu.quaternion()
    b = pack(
        ">iffffffffffffffffff",
        *[
            timestamp,
            alti,
            lin_acc[0],
            lin_acc[1],
            lin_acc[2],
            temp2,
            mag[0],
            mag[1],
            mag[2],
            gyro[0],
            gyro[1],
            gyro[2],
            euler[0],
            euler[1],
            euler[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
        ],
    )
    return b


def initialize():
    global mode, imu, bmp, unstored_readings, nvs, reading_num, lora, initial_altitude, apogee, neopixel, launch_time_ms, debounce_time

    # Status LEDs #
    neopixel_pwr_pin = Pin(38, Pin.OUT)
    neopixel_pwr_pin.on()
    neopixel_pin = Pin(39, Pin.OUT)
    neopixel = NeoPixel(neopixel_pin, 1)

    neopixel[0] = _NEOPIXEL_RED  # type: ignore
    neopixel.write()

    mode = _MODE_LAUNCHPAD

    launch_time_ms = time.ticks_ms()
    debounce_time = launch_time_ms

    i2c = I2C(scl=6, sda=7)

    imu = BNO055(i2c)
    bmp = BMP280(i2c, 0x77)
    # TODO: Initialize / configure / calibrate sensors?

    unstored_readings = RingIO(_RINGIO_SIZE)

    # TODO: Namespace rotation
    nvs = esp32.NVS("SammyNamespace")

    lora = LoRa(
        spi_channel=SPIConfig.esp32s3_1,
        interrupt=16,
        this_address=_LITTLE_BUDDY_ADDR,
        cs_pin=5,
        reset_pin=8,
        freq=915.0,
        tx_power=5,
        modem_config=ModemConfig.USLegalLongRange,
        receive_all=False,
        acks=False,
        crypto=None,
    )

    boot_button.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

    reading_num = _GROUND_READINGS + 1
    initial_altitude = bmp.altitude
    apogee = initial_altitude


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


def launchpad(initial_altitude: float) -> tuple[list, int]:
    global ground_readings, recent_altis
    neopixel[0] = _NEOPIXEL_WHT  # type: ignore
    neopixel.write()
    ground_readings = deque([], _GROUND_READINGS)
    recent_altis = deque([], _RECENT_READINGS)
    enable_sensor_recording()
    time.sleep(5)


def ascent() -> None:
    # Nothing really to do here -- sensors are already on, and the switch to NVS is handled by the process_reading function
    neopixel[0] = _NEOPIXEL_ORA  # type: ignore
    neopixel.write()
    pass


def descent() -> None:
    neopixel[0] = _NEOPIXEL_CYA  # type: ignore
    neopixel.write()
    enable_buzzer()
    enable_radio()


def share_files(arg=None) -> None:
    """Turns on wifi and an FTP server

    This will turn the device into a Wi-Fi access point (using the SSID and password from the file "wifi.txt"), and then turn on a single-user FTP server (no username or password). After the user disconnects from that FTP server, this will return.
    """
    neopixel[0] = _NEOPIXEL_PUR  # type: ignore
    neopixel.write()
    # TODO: Disable buzzer and radio
    with open("/wifi.txt", "r") as f:
        wifi = json.loads(f.read())
    ap_if = network.WLAN(network.AP_IF)
    ap_if.active(True)
    time.sleep_ms(10)  # Give things a chance to settle
    ap_if.config(ssid=wifi["ssid"], security=3, key=wifi["key"])
    ftp.ftpserver()


def touchdown() -> None:
    global mode
    neopixel[0] = _NEOPIXEL_GRN  # type: ignore
    neopixel.write()
    # TODO: Store initial data in a file
    # TODO: Move / convert data from NVS to the file
    # TODO: Erase NVS? Or nah
    mode = _MODE_FINISHED
    neopixel[0] = _NEOPIXEL_BLU  # type: ignore
    neopixel.write()


initialize()
time.sleep(2)
initial_data, initial_data_idx = launchpad(initial_altitude)
