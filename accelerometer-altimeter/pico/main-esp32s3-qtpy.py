"""A script for a model rocket altimeter / accelerometer

This script contains constants (both user-modifiable and not-modifiable) and
behavior necessary to measure the altitude and acceleration of a model rocket.
Importantly, these measurements are stored on the device in a binary format
that is designed to be read by a sibling script on a conventional computer
(such as a laptop); it does not produce readable output by itself.

The primary design goal of this script is low power consumption, the secondary
goal is high-resolution altitude and acceleration data. Lowering the power
consumption is vital to reducing the size of the battery required to run the
device, which makes the overall device lighter, which means the model rocket
can go faster and higher.

To use this script, copy it and the device drivers to your ESP32-S3 QT Py, 
rename this script to "main.py," then reboot the device. It will turn on and 
run automatically: you should see a red LED, followed by (very briefly) a green 
and then blue LED. The LED should then turn off before briefly showing green 
and then blue again, for the duration specified using the user-modifiable 
constant _DURATION_MINS.

When you want to get the files off the device, hold the "Boot" button down. You 
may have to hold it for a couple of seconds -- it's only checked when the 
computer is awake; it will not wake the computer up or skip sensor reading. 
This will end the sensing loop, turn on the device's wifi (in access point 
mode, so you'll have to join its network, named something like ESP_XXXXXX), and 
start a FTP server using the implementation from Christopher Popp and Paul 
Sokolovsky: https://github.com/robert-hh/FTP-Server-for-ESP8266-ESP32-and-PYBD. 
Note: When copying files off of the device, you should plug it into a computer, 
USB power bank, car charger, etc -- the wifi uses a tremendous amount of power 
and it is not recommended to run it off of the small battery used for recording 
data.

At a high level, this script:

1. Initializes the altimeter, accelerometer, (collectively the
sensors), status LED, and filesystem
2. Enters the main loop, where it
  a. Reads from the sensors' FIFO caches
  b. Stores the sensor readings in its onboard flash memory
  c. Goes into "lightsleep" for a length of time
  d. Returns to the top of the loop (step 2.a)
"""

from machine import Pin, Signal, I2C
from micropython import const
from neopixel import NeoPixel
import time, gc, machine, os, vfs, network, ftp

from bmp390 import BMP390
from icm20649 import ICM20649

#############################
# User-Modifiable Constants #
#############################

_RESET_DATA = const(False)  # True to wipe all launch history
"""boolean: True to wipe all launch history / free up disk space"""


_USE_LIGHTSLEEP = const(True)
"""boolean: True to use lightsleep instead of just time.sleep

Note that setting this to True will break USB output. This is intended to save 
power, use it when running on batteries.
"""

_DURATION_MINS = const(5)
"""int: How long (in minutes) the device will run after powering up.

This will be approximated, unless period_ms divides evenly into the duration.
"""

_RESOLUTION = const(1)
"""int (1-3): The 'resolution' of the sensor readings.

Higher values mean more sensor readings, more current draw, and more disk usage."""

#################################
# Non-User-Modifiable Constants #
#################################

# Resolution #
if _RESOLUTION == 1:
    _PERIOD_MS = 2800  # Determined based on how fast the altimeter cache fills
    _CPU_FREQUENCY = 40000000
if _RESOLUTION == 2:
    _PERIOD_MS = 1400
    _CPU_FREQUENCY = 80000000
elif _RESOLUTION == 3:
    _PERIOD_MS = 700
    _CPU_FREQUENCY = 160000000

# Timing #
_DURATION_PERIODS = ((_DURATION_MINS * 60) * 1000) // _PERIOD_MS

# LED Control #
_NEOPIXEL_BRIGHTNESS = const(3)  # 1-255
_NEOPIXEL_OFF = (0, 0, 0)
#                                             # LED Meaning:
_NEOPIXEL_RED = (_NEOPIXEL_BRIGHTNESS, 0, 0)  # Initialization
_NEOPIXEL_GRN = (0, _NEOPIXEL_BRIGHTNESS, 0)  # Reading FIFO buffers
_NEOPIXEL_BLU = (0, 0, _NEOPIXEL_BRIGHTNESS)  # File I/O

# Status LEDs #
neopixel_pwr_pin = Pin(38, Pin.OUT)
neopixel_pwr_pin.on()
neopixel_pin = Pin(39, Pin.OUT)
neopixel = NeoPixel(neopixel_pin, 1)
neopixel[0] = _NEOPIXEL_RED  # type: ignore
neopixel.write()

# Save / shutdown button #
shutdown_button = Signal(Pin(0, Pin.IN), invert=True)

# Connect to sensors #
i2c = I2C(1, scl=40, sda=41, freq=400000)


def initialize_filesystem() -> None:
    """Prepares the filesystem for use.

    After this is complete, the current working directory will be empty, and
    named with a decimal value that is one higher than the previously-highest
    directory name.

    Example:
        If the _RESET_DATA global variable is True:
            The current working directory will be "1"
        If the _RESET_DATA global variable is False and the highest directory 
        prior to this method's invocation was "42"
            The current working directory will be "43"
    """

    # If you ever need to re-make the filesystem:
    # vfs.umount('/')
    # vfs.VfsLfs2.mkfs(bdev) # type: ignore

    vfs.mount(vfs.VfsLfs2(bdev, readsize=2048, progsize=256, lookahead=256, mtime=False), "/")  # type: ignore
    os.chdir("data")

    if _RESET_DATA:
        # Wipe all existing data
        for launch_entry in os.ilistdir():
            os.chdir(launch_entry[0])
            for file_entry in os.ilistdir():
                os.remove(file_entry[0])
            os.chdir("..")
            os.rmdir(launch_entry[0])
        new_dir = "1"
    else:
        # Make a new directory named as an integer one higher than the highest
        # existing directory
        new_dir = str(int(sorted([int(i) for i in os.listdir()]).pop()) + 1)

    os.mkdir(new_dir)
    os.chdir(new_dir)


def init() -> tuple[ICM20649, BMP390, network.WLAN]:
    """Initializes the sensors and computer.

    This method:
    1. Clocks the computer's CPU up or down (according to the frequency aligned 
    with the user's resolution selection in the _RESOLUTION global)
    2. Instantiates the sensors' driver classes
    3. Initializes the filesystem
    4. Turns off the status LED (which had been red) as initialization is 
    complete.

    :return: An instance of the altimeter and accelerometer driver classes, 
    respectively.
    :rtype: BMP390, ICM20649
    """
    machine.freq(_CPU_FREQUENCY)
    accel = ICM20649(_RESOLUTION, i2c)
    alti = BMP390(_RESOLUTION, i2c)
    accel.initialize_device()
    alti.initialize_device()
    initialize_filesystem()
    ap_if = network.WLAN(network.AP_IF)
    neopixel[0] = _NEOPIXEL_OFF  # type: ignore
    neopixel.write()
    return accel, alti, ap_if


def read_fifo(accel: ICM20649, alti: BMP390) -> tuple[int, int]:
    """Reads from the sensors' FIFO caches

    Invokes the read_fifo() methods of the device drivers and returns the 
    number of bytes read. Sets the status LED to green.

    :param ICM20649 accel: An instance of the accelerometer driver class which 
    has been initialized
    :param BMP390 alti: An instance of the altimeter driver class which has 
    been initialized
    :return: The number of bytes read from the accelerometer and altimeter, 
    respectively
    :rtype: int, int
    """
    neopixel[0] = _NEOPIXEL_GRN  # type: ignore
    neopixel.write()
    accel_bytes = accel.read_fifo()
    alti_bytes = alti.read_fifo()
    return accel_bytes, alti_bytes


def write_files(accel_mv: memoryview, alti_mv: memoryview) -> None:
    """Writes the sensor readings to the filesystem

    Note that the entire memoryview will be written -- they should be resized to the appropriate length prior to calling this method.

    :param memoryview accel_mv: The accelerometer readings. 
    :param memoryview alti_mv: The altimeter readings.
    """
    neopixel[0] = _NEOPIXEL_BLU  # type: ignore
    neopixel.write()
    gc.collect()
    with open("accel.bin", "ab") as f:
        f.write(accel_mv)
    with open("alti.bin", "ab") as f:
        f.write(alti_mv)


@micropython.native
def main_loop(accel: ICM20649, alti: BMP390) -> None:
    """Core loop: Read from sensors, write to files, sleep, repeat.

    This will loop for the number of periods (as stored in the 
    _DURATION_PERIODS global). You can exit the loop early -- which will also avoid sleeping, if on battery power -- by holding the "Boot" button.

    :param ICM20649 accel: An instance of the accelerometer driver class which 
    has been initialized
    :param BMP390 alti: An instance of the altimeter driver class which has 
    been initialized
    """

    for _ in range(_DURATION_PERIODS):
        start_ms = time.ticks_ms()
        accel_bytes, alti_bytes = read_fifo(accel, alti)
        write_files(accel.mv[0:accel_bytes], alti.mv[0:alti_bytes])

        if shutdown_button.value() == 1:
            # Things keep running if the shutdown button is pushed -- the
            # device only turns off if it runs the full duration
            return

        neopixel[0] = _NEOPIXEL_OFF  # type: ignore
        neopixel.write()
        loop_time = time.ticks_diff(time.ticks_ms(), start_ms)
        print(loop_time)
        if _USE_LIGHTSLEEP:
            machine.lightsleep(max(0, (_PERIOD_MS - loop_time)))
        else:
            time.sleep_ms(max(0, (_PERIOD_MS - loop_time)))

    if _USE_LIGHTSLEEP:
        # Lightsleep is used when on battery power.
        # Since we're on battery power, and we've run for the full duration, we
        # should try and save power further -- this is as close as we can get
        # to shutting everything off

        alti.shutdown()
        accel.shutdown()
        time.sleep_ms(10)  # Give things a chance to settle
        machine.deepsleep()


# This lets us skip running the code (so we can drop into REPL / get the files)
if shutdown_button.value() == 0:
    accel, alti, wlan = init()
    main_loop(accel, alti)
    machine.freq(240000000) # The wifi seems to hang if we run it at 40mhz
    wlan.active(True)
    ftp.ftpserver()
