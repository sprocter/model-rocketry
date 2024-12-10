from machine import Pin, Signal, I2C
from micropython import const
from struct import unpack
from neopixel import NeoPixel
import time

# TODO:
#   1. Altimeter FIFO
#   2. Altimeter file output
#   3. Tera support for altimeter output
#   4. Cleanup

############################
# Initialization / Startup #
############################

## Constants ##
_CURR_BARO_PRESSURE = const(1013)

# LED Control #
_NEOPIXEL_BRIGHTNESS = const(1) # 1-255
_NEOPIXEL_RED = (_NEOPIXEL_BRIGHTNESS, 0, 0)
_NEOPIXEL_GRN = (0, _NEOPIXEL_BRIGHTNESS, 0)
_NEOPIXEL_BLU = (0, 0, _NEOPIXEL_BRIGHTNESS)
_NEOPIXEL_OFF = (0, 0, 0)

# Altimeter Registers #
_ALTI_ADDR = const(0x77) # Default BMP290 I2C Addr: 119
_CALIB_COEFFS = const(0x31) # pg 28
_DATA_0 = const(0x04) # pg 32
_PWR_CTRL = const(0x1B) # pg 36

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

# Slow things down for lower power consumption, see neat chart on
# page 1341 of the RP2350 datasheet
# machine.freq(65000000)

# Set up status LEDs 
neopixel_pwr_pin = Pin(38, Pin.OUT)
neopixel_pwr_pin.on()
neopixel_pin = Pin(39, Pin.OUT)
neopixel = NeoPixel(neopixel_pin, 1)
neopixel[0] = _NEOPIXEL_RED # type: ignore
neopixel.write()

# Set up save / shutdown button
shutdown_button = Signal(Pin(0, Pin.IN), invert = True)

i2c = I2C(1, scl=40, sda=41, freq=400000)

def initialize_alti() -> bytearray:

    # Turn on normal mode, the pressure sensor, and the temperature sensor
    pwr_ctrl = bytearray(1)
    i2c.readfrom_mem_into(_ALTI_ADDR, _PWR_CTRL, pwr_ctrl)
    pwr_ctrl[0] |= 0b00110011
    i2c.writeto_mem(_ALTI_ADDR, _PWR_CTRL, pwr_ctrl)

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

def print_alti_data(reading : memoryview, coeffs_packed : bytearray):
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

    raw_pressure = reading[2] << 16 | reading[1] << 8 | reading[0]
    raw_temp = reading[5] << 16 | reading[4] << 8 | reading[3]

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
    coeffs_packed = initialize_alti()
    initialize_accel()
    neopixel[0] = _NEOPIXEL_OFF # type: ignore
    neopixel.write()
    return coeffs_packed

def main_portion(coeffs_packed : bytearray):
    alti_data = bytearray(6)
    alti_mv = memoryview(alti_data)
    i2c.readfrom_mem_into(_ALTI_ADDR, _DATA_0, alti_mv)
    print_alti_data(alti_mv, coeffs_packed)

    return 
    accel_data = bytearray(4000) # Shouldn't be larger than ~1800
    accel_mv = memoryview(accel_data)
    fifo_count_bytes = bytearray(2)
    
    with open('accel.bin', 'wb') as f:
        pass # nuke the file 
    
    for i in range(10):
        start_ms = time.ticks_ms()
        neopixel[0] = _NEOPIXEL_GRN # type: ignore
        neopixel.write()
        remainder_count = 0
        i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_COUNTH, fifo_count_bytes)
        fifo_count = (fifo_count_bytes[0] & 15) << 8 | fifo_count_bytes[1]
        
        # Reading more than 1878 bytes reliably causes a timeout.
        # We do 12 fewer for a little breathing room
        if fifo_count > 1866:
            remainder_count = min(fifo_count - 1866, 1866)
            fifo_count = 1866
            i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_R_W, accel_mv[0:fifo_count])
            print("Taking out da trash (bytes): ", remainder_count)
            i2c.readfrom_mem(_ACCEL_ADDR, _FIFO_R_W, remainder_count)
        else:
            i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_R_W, accel_mv[0:fifo_count])
        
        neopixel[0] = _NEOPIXEL_BLU # type: ignore
        neopixel.write()
        with open('accel.bin', 'ab') as f:
            f.write(accel_mv[0 : fifo_count])
        
        neopixel[0] = _NEOPIXEL_OFF # type: ignore
        neopixel.write()
        end_ms = time.ticks_ms()
        if shutdown_button.value() == 1:
            print("Got shutdown command. Quitting...")
            break
        time.sleep_ms(3000 - time.ticks_diff(end_ms, start_ms))

coeffs_packed = init()
main_portion(coeffs_packed)