from machine import Pin, Signal, I2C
from micropython import const
from struct import unpack
from neopixel import NeoPixel
import time
import gc
import machine
import os
import vfs

# TODO:
#   1. Continue investigating / working around occasional slow file I/O
#   2. Sleep
#       1. Light sleep until deep is proven necessary
#       2. Add const option or USB power detection

############################
# Initialization / Startup #
############################

## Constants ##
_CURR_BARO_PRESSURE = const(1008)
_RESET_DATA = const(False)
_PERIOD_MS = const(2727) # Much higher than 2.5s and we risk overflowing the 
#                        # altimeter's FIFO buffer 😪
_DURATION_MINS = const(1)
_DURATION_PERIODS = const(((_DURATION_MINS * 60) * 1000) // _PERIOD_MS)


# LED Control #
_NEOPIXEL_BRIGHTNESS = const(1) # 1-255
_NEOPIXEL_OFF = (0, 0, 0)
#                                            # LED Meaning:
_NEOPIXEL_RED = (_NEOPIXEL_BRIGHTNESS, 0, 0) # Initialization
_NEOPIXEL_GRN = (0, _NEOPIXEL_BRIGHTNESS, 0) # Reading FIFO buffers
_NEOPIXEL_BLU = (0, 0, _NEOPIXEL_BRIGHTNESS) # File I/O

# Altimeter Registers #
_ALTI_ADDR = const(0x77) # Default BMP290 I2C Addr: 119
_CALIB_COEFFS = const(0x31) # pg 28
_DATA_0 = const(0x04) # pg 32
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


## Slow down the CPU ##
machine.freq(40000000) # Significantly lowers power consumption, see table on 
#                        page 66 of the esp32 datasheet

## Set up Globals ##

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

def initialize_alti() -> bytearray:
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

    # Get calibration coefficients
    coeffs_packed = bytearray(21)
    i2c.readfrom_mem_into(_ALTI_ADDR, _CALIB_COEFFS, coeffs_packed)
    return coeffs_packed

def initialize_accel():
    # Begin with Bank 0 configuration
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b'\x00')

    # Reset the device to default settings
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x80') # 0b10000000
    time.sleep_ms(5)
    while i2c.readfrom_mem(_ACCEL_ADDR, _PWR_MGMT_1, 1) == 128:
        print("waiting")
        time.sleep_ms(5)

    # Turn low power off, temperature sensor off, use best available clock source
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

    # Finally, set the LP_EN flag -- this prevents writing to most registers, so disable for further configuration.
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x29') # 0b00101001

    # Expected power consumption: 120uA

def initialize_filesystem() -> None:
    if _RESET_DATA:
        # Wipe all existing data
        vfs.umount('/')
        vfs.VfsLfs2.mkfs(bdev)
        vfs.mount(bdev, '/')
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

def print_alti_data(pressure_reading : memoryview, temp_reading : memoryview, coeffs_packed : bytearray):
    coeffs_unpacked = unpack("<HHbhhbbHHbbhbb", coeffs_packed)
    par_t1 = coeffs_unpacked[0] / (2 ** -8)
    par_t2 = coeffs_unpacked[1] / (2 ** 30)
    par_t3 = coeffs_unpacked[2] / (2 ** 48)

    par_p1 = (coeffs_unpacked[3] - (2 ** 14))/(2 ** 20)
    par_p2 = (coeffs_unpacked[4] - (2 ** 14))/(2 ** 29)
    par_p3 = coeffs_unpacked[5]/(2 ** 32)
    par_p4 = coeffs_unpacked[6]/(2 ** 37)
    par_p5 = coeffs_unpacked[7]/(2 ** -3)
    par_p6 = coeffs_unpacked[8]/(2 ** 6)
    par_p7 = coeffs_unpacked[9]/(2 ** 8)
    par_p8 = coeffs_unpacked[10]/(2 ** 15)
    par_p9 = coeffs_unpacked[11]/(2 ** 48)
    par_p10 = coeffs_unpacked[12]/(2 ** 48)
    par_p11 = coeffs_unpacked[13]/(2 ** 65)

    raw_pressure = pressure_reading[2] << 16 | pressure_reading[1] << 8 | pressure_reading[0]
    raw_temp = temp_reading[2] << 16 | temp_reading[1] << 8 | temp_reading[0]

    partial_data1 = raw_temp - par_t1
    partial_data2 = partial_data1 * par_t2
    temperature_c = partial_data2 + (partial_data1 ** 2) * par_t3
    temperature_f = (temperature_c * 1.8) + 32

    partial_data1 = par_p6 * temperature_c
    partial_data2 = par_p7 * (temperature_c ** 2)
    partial_data3 = par_p8 * (temperature_c ** 3)
    partial_out1 = par_p5 + partial_data1 + partial_data2 + partial_data3

    partial_data1 = par_p2 * temperature_c
    partial_data2 = par_p3 * (temperature_c ** 2)
    partial_data3 = par_p4 * (temperature_c ** 3)
    partial_out2 = raw_pressure * (par_p1 + partial_data1 + partial_data2 + partial_data3)

    partial_data1 = raw_pressure ** 2
    partial_data2 = par_p9 + par_p10 * temperature_c
    partial_data3 = partial_data1 * partial_data2
    partial_data4 = partial_data3 + (raw_pressure ** 3) * par_p11

    pressure_hpa = partial_out1 + partial_out2 + partial_data4
    alti_ft = ((1-((float(pressure_hpa/100)/_CURR_BARO_PRESSURE) ** .190284)) * 145366.45)

    print("Temperature (f): ", temperature_f)
    print("Pressure (hPa): ", pressure_hpa)
    print("Altitude (ft): ", alti_ft)
 
def print_accel_data(reading : memoryview):
    # Accelerometer Calibration Values
    _x_err = 0.029050755
    _y_err = -0.008103238
    _z_err = 0.044206185

    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
    
    # accelerometer "sensitivity scale factors" on page 13
    accel_x = (raw_accel_x / 1024) - _x_err
    accel_y = (raw_accel_y / 1024) - _y_err
    accel_z = (raw_accel_z / 1024) - _z_err

    print("Accel (x, y, z):", accel_x, accel_y, accel_z)

def init():
    initialize_filesystem()
    coeffs_packed = initialize_alti()
    initialize_accel()
    gc.collect()
    neopixel[0] = _NEOPIXEL_OFF # type: ignore
    neopixel.write()
    return coeffs_packed

def single_alti_reading(coeffs_packed : bytearray) -> None:
    alti_data = bytearray(6)
    alti_mv = memoryview(alti_data)
    i2c.readfrom_mem_into(_ALTI_ADDR, _DATA_0, alti_mv)
    print_alti_data(alti_mv[0:3], alti_mv[3:6], coeffs_packed)

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

def write_files(accel_mv : memoryview, alti_mv : memoryview, period : int) -> None:
    neopixel[0] = _NEOPIXEL_BLU # type: ignore
    neopixel.write()
    with open('accel.bin', 'ab') as f:
        f.write(accel_mv)
    with open('alti.bin', 'ab') as f:
        f.write(alti_mv)

def main_loop():
    accel_mv = memoryview(bytearray(4096)) # Shouldn't be larger than ~1500
    alti_mv = memoryview(bytearray(512)) # Shouldn't be larger than ~450
    for period in range(_DURATION_PERIODS):
        start_ms = time.ticks_ms()
        
        accel_bytes, alti_bytes = read_fifo(accel_mv, alti_mv)
        write_files(accel_mv[0 : accel_bytes], alti_mv[0 : alti_bytes], period)
        file_io_time = time.ticks_diff(time.ticks_ms(), start_ms)

        if shutdown_button.value() == 1:
            print("Got shutdown command. Quitting...")
            break

        neopixel[0] = _NEOPIXEL_OFF # type: ignore
        neopixel.write()
        end_ms = time.ticks_ms()
        loop_time = time.ticks_diff(end_ms, start_ms)
        print(period, ": File I/O time (ms): ", file_io_time)
        # print("Loop time (ms): ", loop_time)
        time.sleep_ms(max(0, (_PERIOD_MS - loop_time)))

coeffs_packed = init()
main_loop()