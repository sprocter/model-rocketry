from machine import Pin, Signal, I2C
from micropython import const
from struct import unpack
from neopixel import NeoPixel
import time, gc, machine, os, vfs

############################
# Initialization / Startup #
############################

### Constants ###

# User-Modifiable #
_RESET_DATA = const(False) # True to wipe all launch history

_USE_LIGHTSLEEP = const(False) # True to use lightsleep instead of just
#                              # time.sleep. Note that setting this to True will
#                              # break USB output. This is intended to save 
#                              # power, use it when running on batteries

_DURATION_MINS = const(5) # This will be approximated, unless period_ms divides 
#                         # evenly into the duration

## Not User-Modifiable ##

# Timing #

_PERIOD_MS = const(2727) # Much higher than this and we risk overflowing the 
#                        # altimeter's FIFO buffer 😪
_DURATION_PERIODS = const(((_DURATION_MINS * 60) * 1000) // _PERIOD_MS)

# LED Control #
_NEOPIXEL_BRIGHTNESS = const(3) # 1-255
_NEOPIXEL_OFF = (0, 0, 0)
#                                            # LED Meaning:
_NEOPIXEL_RED = (_NEOPIXEL_BRIGHTNESS, 0, 0) # Initialization
_NEOPIXEL_GRN = (0, _NEOPIXEL_BRIGHTNESS, 0) # Reading FIFO buffers
_NEOPIXEL_BLU = (0, 0, _NEOPIXEL_BRIGHTNESS) # File I/O

# Altimeter Registers #
_ALTI_ADDR = const(0x77) # Default BMP290 I2C Addr: 119
_CALIB_COEFFS = const(0x31) # pg 28
_ALTI_FIFO_LENGTH_0 = const(0x12) # pg 33
_ALTI_FIFO_DATA = const(0x14) # pg 34
_ALTI_FIFO_CONFIG_1 = const(0x17) # pg 34
_ALTI_FIFO_CONFIG_2 = const(0x18) # pg 35
_ALTI_PWR_CTRL = const(0x1B) # pg 36
_ALTI_OSR = const(0x1C) # pg 37
_ALTI_ODR = const(0x1D) # pg 37
_ALTI_CONFIG = const(0x1F) # pg 39
_ALTI_CMD = const(0x7E) # pg 39

# Accelerometer Registers #
_ACCEL_ADDR = const(0x68) # Default ICM20649 I2C Addr: 104
_REG_BANK_SEL = const(0x7F) # All banks
_USER_CTRL = const(0x03) # Bank 0, pg 38
_LP_CONFIG = const(0x05) # Bank 0, pg 39
_PWR_MGMT_1 = const(0x06) # Bank 0, pg 39
_PWR_MGMT_2 = const(0x07) # Bank 0, pg 40
_FIFO_EN_2 = const(0x67) # Bank 0, pg 54
_FIFO_RST = const(0x68) # Bank 0, pg 55
_FIFO_MODE = const(0x69) # Bank 0, pg 55
_FIFO_COUNTH = const(0x70) # Bank 0, pg 55
_FIFO_R_W = const(0x72) # Bank 0, pg 56
_ODR_ALIGN_EN = const(0x09) # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_1 = const(0x10) # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_2 = const(0x11) # Bank 2, pg 65
_ACCEL_CONFIG = const(0x14) # Bank 2, pg 66
_ACCEL_CONFIG_2 = const(0x15) # Bank 2, pg 67
_MOD_CTRL_USR = const(0x54) # Bank 2, pg 70

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

# Connect to sensors #
i2c = I2C(1, scl=40, sda=41, freq=400000)

# Call to get the device-specific coefficients needed to decode altimeter data
def print_packed_coeffs() -> None:
    coeffs_packed = bytearray(21)
    i2c.readfrom_mem_into(_ALTI_ADDR, _CALIB_COEFFS, coeffs_packed)
    print("_PACKED_COEFFS = \"", str(hex(int.from_bytes(coeffs_packed)))[2:], "\"")

# Call to get the device-specific calibration values needed to correct 
# accelerometer data
def print_accel_calib_values() -> None:
    xs, ys, zs = [], [], []
    reading = bytearray(6)
    print("Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in five seconds.")
    time.sleep(5)
    print("Calibration beginning now, it will take 10 seconds...")
    for _ in range(1000):
        i2c.readfrom_mem_into(_ACCEL_ADDR, 0x2D, reading)
        raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
        xs.append(raw_accel_x / 1024)
        ys.append(raw_accel_y / 1024)
        zs.append(raw_accel_z / 1024)
        time.sleep_ms(10)

    print("_X_ERR = ", sum(xs) / len(xs))
    print("_Y_ERR = ", sum(ys) / len(ys))
    print("_Z_ERR = ", sum(zs) / len(zs) - 1)

def initialize_alti() -> None:
    # Reset the device
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_CMD, b'\xB6') 
    time.sleep_ms(5)

    # Turn on normal mode, the pressure sensor, and the temperature sensor
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_PWR_CTRL, b'\x33') # RR11RR11

    # Turn on FIFO, don't stop when FIFO is full, don't return "sensortime" #
    # frames, do store pressure data, do store temperature data
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_FIFO_CONFIG_1, b'\x19')  # RRR11001

    # Store filtered data, don't downsample
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_FIFO_CONFIG_2, b'\x08')  # RRR01000

    # Oversampling: Temperature 1 (000), Pressure 8 (011)
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_OSR, b'\x03')  # RR000011

    # Data rate: 25 Hz / 40ms sampling period
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_ODR, b'\x03')  # RRR00011

    # IIR Filter: 3 (010)
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_CONFIG, b'\x04') # RRR010R

    # Clear FIFO now that we've messed with the settings
    i2c.writeto_mem(_ALTI_ADDR, _ALTI_CMD, b'\xB0') 

def initialize_accel():
    # Begin with Bank 0 configuration
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b'\x00')

    # Reset the device to default settings
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x80') # 0b10000000
    time.sleep_ms(5)
    while i2c.readfrom_mem(_ACCEL_ADDR, _PWR_MGMT_1, 1) == 128:
        time.sleep_ms(5)

    # Turn low power off, temperature sensor off, use best available clock 
    # source
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x09') # 0b00001001
    time.sleep_ms(30) # Let the accelerometer wake up out of sleep

    # Turn off the gyroscope, turn on the accelerometer
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_2, b'\x07') # 0b00000111

    # Turn DMP Off, FIFO on, i2c master i/f module off, don't reset anything
    i2c.writeto_mem(_ACCEL_ADDR, _USER_CTRL, b'\x40') # 0b01000000
    
    # Now on to Bank 2 configuration
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b'\x20') # 0b00100000
    
    # Enable ODR start-time alignment
    i2c.writeto_mem(_ACCEL_ADDR, _ODR_ALIGN_EN, b'\x01')

    # Sample rate = 10, rate_hz = 1125/(1 + rate) = 1125/11 = 102.273
    # Split across two bytes, 12 bits
    # MSB, 0b00000000
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_1, b'\x00') 
    # LSB, 0b00001010
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_2, b'\x0A')
    
    # Low pass filter = 3, Accelerometer Full Scale 30g, DLPF Enabled
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG, b'\x3F') # 0b00111111

    # Average 4 samples (would be 1, but we're using the DLPF)
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG_2, b'\x00') # 0b0000000

    # Disable DMP in LP Accelerometer mode
    i2c.writeto_mem(_ACCEL_ADDR, _MOD_CTRL_USR, b'\x02')

    # Back to Bank 0
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL,  b'\x00')

    # Operate accelerometer in duty cycled mode
    i2c.writeto_mem(_ACCEL_ADDR, _LP_CONFIG, b'\x20') # 0b00100000

    # Turn FIFO on for accelerometer data
    i2c.writeto_mem(_ACCEL_ADDR, _FIFO_EN_2, b'\x10') # 0b00010000

    # Set FIFO to replace old data when full. Inshallah this won't happen
    i2c.writeto_mem(_ACCEL_ADDR, _FIFO_MODE, b'\x00')

    # Reset FIFO, step 1: Assert (Set FIFO size to zero)
    i2c.writeto_mem(_ACCEL_ADDR, _FIFO_RST, b'\x01')
    # Reset FIFO, step 2: De-assert
    i2c.writeto_mem(_ACCEL_ADDR, _FIFO_RST, b'\x00')

    # Finally, set the LP_EN flag -- this prevents writing to most registers, 
    # so disable for further configuration.
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x29') # 0b00101001

def initialize_filesystem() -> None:
    if _RESET_DATA:
        # Wipe all existing data
        vfs.umount('/')
        vfs.VfsLfs2.mkfs(bdev) # type: ignore
        vfs.mount(bdev, '/') # type: ignore
        os.mkdir("data")
        os.chdir("data")
        os.mkdir("1")
        os.chdir("1")
    else:
        os.chdir("data")
        # Make a new directory named as an integer one higher than the highest 
        # existing directory
        new_dir = str(int(sorted([int(i) for i in os.listdir()]).pop()) + 1)
        os.mkdir(new_dir)
        os.chdir(new_dir)

def init():
    initialize_filesystem()
    coeffs_packed = initialize_alti()
    initialize_accel()
    gc.collect()
    neopixel[0] = _NEOPIXEL_OFF # type: ignore
    neopixel.write()
    return coeffs_packed

def read_alti_fifo(alti_mv : memoryview) -> int:
    fifo_count_bytes = bytearray(2)
    i2c.readfrom_mem_into(_ALTI_ADDR, _ALTI_FIFO_LENGTH_0, fifo_count_bytes)
    fifo_count = fifo_count_bytes[1] << 8 | fifo_count_bytes[0]
    i2c.readfrom_mem_into(_ALTI_ADDR, _ALTI_FIFO_DATA, alti_mv[0:fifo_count])
    return fifo_count

def read_accel_fifo(accel_mv : memoryview) -> int:
    fifo_count_bytes = bytearray(2)
    i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_COUNTH, fifo_count_bytes)
    fifo_count = (fifo_count_bytes[0] & 15) << 8 | fifo_count_bytes[1]
    i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_R_W, accel_mv[0:fifo_count])
    return fifo_count

def read_fifo(accel_mv : memoryview, alti_mv : memoryview) -> tuple[int, int]:
    neopixel[0] = _NEOPIXEL_GRN # type: ignore
    neopixel.write()
    accel_bytes = read_accel_fifo(accel_mv)
    alti_bytes = read_alti_fifo(alti_mv)
    return accel_bytes, alti_bytes

def write_files(accel_mv : memoryview, alti_mv : memoryview) -> None:
    neopixel[0] = _NEOPIXEL_BLU # type: ignore
    neopixel.write()
    gc.collect()
    with open('accel.bin', 'ab') as f:
        f.write(accel_mv)
    with open('alti.bin', 'ab') as f:
        f.write(alti_mv)

def main_loop():
    accel_mv = memoryview(bytearray(4096)) # Shouldn't be larger than ~1500
    alti_mv = memoryview(bytearray(512)) # Shouldn't be larger than ~450
    for _ in range(_DURATION_PERIODS):
        start_ms = time.ticks_ms()
        accel_bytes, alti_bytes = read_fifo(accel_mv, alti_mv)
        write_files(accel_mv[0 : accel_bytes], alti_mv[0 : alti_bytes])
        
        if shutdown_button.value() == 1:
            # Things keep running if the shutdown button is pushed -- the 
            # device only turns off if it runs the full duration
            return 

        neopixel[0] = _NEOPIXEL_OFF # type: ignore
        neopixel.write()
        loop_time = time.ticks_diff(time.ticks_ms(), start_ms)

        if _USE_LIGHTSLEEP:
            machine.lightsleep(max(0, (_PERIOD_MS - loop_time)))
        else: 
            time.sleep_ms(max(0, (_PERIOD_MS - loop_time)))

    if _USE_LIGHTSLEEP:
        # Lightsleep is used when on battery power.
        # Since we're on battery power, and we've run for the full duration, we 
        # should try and save power further -- this is as close as we can get
        # to shutting everything off

        # Set altimeter to sleep mode, turn off pressure and temperature sensors
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_PWR_CTRL, b'\x00') # RR00RR00
        # Turn low power on and temperature sensor off, stop clock
        i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x6F') # 0b01101111
        # Turn off the gyroscope and accelerometer
        i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_2, b'\x3F') # 0b00111111
        
        time.sleep_ms(10) # Give things a chance to settle
    
        machine.deepsleep()


# This lets us skip running the code (so we can drop into REPL / get the files)
if shutdown_button.value() == 0:
    coeffs_packed = init()
    main_loop()