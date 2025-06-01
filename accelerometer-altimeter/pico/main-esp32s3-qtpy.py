from machine import Pin, Signal, I2C
from micropython import const
from neopixel import NeoPixel
import time, gc, machine, os, vfs

from bmp390 import BMP390
from icm20649 import ICM20649

############################
# Initialization / Startup #
############################

### Constants ###

## User-Modifiable ##
_RESET_DATA = const(False) # True to wipe all launch history

_USE_LIGHTSLEEP = const(True) # True to use lightsleep instead of just
#                              # time.sleep. Note that setting this to True will
#                              # break USB output. This is intended to save 
#                              # power, use it when running on batteries

_DURATION_MINS = const(5) # This will be approximated, unless period_ms divides 
#                         # evenly into the duration

_RESOLUTION = const(1) # 1-3. Higher = more sensor readings, current draw, and 
                       # disk usage

## Not User-Modifiable ##

# Resolution #
if _RESOLUTION == 1:
    _PERIOD_MS = 2800
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
_NEOPIXEL_BRIGHTNESS = const(3) # 1-255
_NEOPIXEL_OFF = (0, 0, 0)
#                                            # LED Meaning:
_NEOPIXEL_RED = (_NEOPIXEL_BRIGHTNESS, 0, 0) # Initialization
_NEOPIXEL_GRN = (0, _NEOPIXEL_BRIGHTNESS, 0) # Reading FIFO buffers
_NEOPIXEL_BLU = (0, 0, _NEOPIXEL_BRIGHTNESS) # File I/O

### Set up Globals ###

# Set up status LEDs #
neopixel_pwr_pin = Pin(38, Pin.OUT)
neopixel_pwr_pin.on()
neopixel_pin = Pin(39, Pin.OUT)
neopixel = NeoPixel(neopixel_pin, 1)
neopixel[0] = _NEOPIXEL_RED # type: ignore
neopixel.write()

# Set up save / shutdown button #
shutdown_button = Signal(Pin(0, Pin.IN), invert = True)

### Connect to sensors ###
i2c = I2C(1, scl=40, sda=41, freq=400000)

def initialize_filesystem() -> None:
    # vfs.umount('/')
    # vfs.VfsLfs2.mkfs(bdev) # type: ignore
    vfs.mount(vfs.VfsLfs2(bdev,readsize=2048,progsize=256,lookahead=256, mtime=False),"/") # type: ignore
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

def init() -> tuple[BMP390, ICM20649]:
    machine.freq(_CPU_FREQUENCY)
    alti = BMP390(_RESOLUTION, i2c)
    accel = ICM20649(_RESOLUTION, i2c)
    alti.initialize_device()
    accel.initialize_device()
    initialize_filesystem()
    neopixel[0] = _NEOPIXEL_OFF # type: ignore
    neopixel.write()
    return alti, accel

def read_fifo(accel : ICM20649, alti : BMP390) -> tuple[int, int]:
    neopixel[0] = _NEOPIXEL_GRN # type: ignore
    neopixel.write()
    accel_bytes = accel.read_fifo()
    alti_bytes = alti.read_fifo()
    return accel_bytes, alti_bytes

def write_files(accel_mv : memoryview, alti_mv : memoryview) -> None:
    neopixel[0] = _NEOPIXEL_BLU # type: ignore
    neopixel.write()
    gc.collect()
    with open('accel.bin', 'ab') as f:
        f.write(accel_mv)
    with open('alti.bin', 'ab') as f:
        f.write(alti_mv)

@micropython.native
def main_loop(alti : BMP390, accel : ICM20649) -> None:
    for _ in range(_DURATION_PERIODS):
        start_ms = time.ticks_ms()
        accel_bytes, alti_bytes = read_fifo(accel, alti)
        write_files(accel.mv[0 : accel_bytes], alti.mv[0 : alti_bytes])

        if shutdown_button.value() == 1:
            # Things keep running if the shutdown button is pushed -- the 
            # device only turns off if it runs the full duration
            return 

        neopixel[0] = _NEOPIXEL_OFF # type: ignore
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
        time.sleep_ms(10) # Give things a chance to settle
    
        machine.deepsleep()


# This lets us skip running the code (so we can drop into REPL / get the files)
if shutdown_button.value() == 0:
    alti, accel = init()
    main_loop(alti, accel)