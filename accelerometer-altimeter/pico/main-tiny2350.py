from machine import Pin, PWM, Signal, I2C
from micropython import const
from struct import unpack
import time

# TODO: Lots, oy.
#   X. Implement pressure translation. See page 18 of the datasheet, and also https://github.com/pimoroni/icp10125-python/blob/main/icp10125/__init__.py
#       X And also altitude!
#   X. Implement configuration
#   3. Self-tests
#   4. Calibration
#   X. FIFO
#   6. General cleanup / organization.
#       * Classes?
#       * Methods, to be sure
#       * Constants. Lots of constants.
#   7. Bring over loop from 2040-w version
#   8. Bring over file i/o from 2040-w version
#   9. Port printing / decoding to "tera" implementation

############################
# Initialization / Startup #
############################

# Constants
_PWM_LED_DUTY_ON = 1024
_PWM_LED_DUTY_OFF = 0

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

_CURR_BARO_PRESSURE = const(1019)

# Slow things down for lower power consumption, see neat chart on
# page 1341 of the RP2350 datasheet
# machine.freq(65000000)

# Set up status LEDs 
# Red means loading, green means sensing, blue means writing to flash
red_led = PWM(Pin(18, Pin.OUT), freq=1000, duty_u16=_PWM_LED_DUTY_ON, invert=True)
grn_led = PWM(Pin(19, Pin.OUT), freq=1000, duty_u16=_PWM_LED_DUTY_OFF, invert=True)
blu_led = PWM(Pin(20, Pin.OUT), freq=1000, duty_u16=_PWM_LED_DUTY_OFF, invert=True)

# Set up save / shutdown button
shutdown_button = Signal(Pin(23, Pin.IN), invert = True)

i2c = I2C(id=0, scl=13, sda=12, freq=400000)

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

    # Sample rate = 10, rate_hz = 1125/(1 + rate) = 1125/11 = 112.5
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

def print_accel_data(reading : memoryview):
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading)
    # raw_gyro_x, raw_gyro_y, raw_gyro_z = unpack(">hhh", reading[6:12])
    
    # Accelerometer "sensitivity scale factors" on page 13
    accel_x = (raw_accel_x / 1024)
    accel_y = (raw_accel_y / 1024)
    accel_z = (raw_accel_z / 1024)

    # # Gyroscope "sensitivity scale factors" on page 12
    # gyro_x = (raw_gyro_x / 65.5)
    # gyro_y = (raw_gyro_y / 65.5)
    # gyro_z = (raw_gyro_z / 65.5)

    # raw_temp = unpack(">h", reading[12:14])[0]
    # temp_c = (raw_temp / 333.87) + 21.0
    # temp_f = temp_c * 1.8 + 32
    print("Accel (x, y, z):", accel_x, accel_y, accel_z)
    # print("Gyro (x, y, z):", gyro_x, gyro_y, gyro_z)
    # print("Temp (Accel):", temp_f)

def init():
    initialize_accel()
    red_led.duty_u16(_PWM_LED_DUTY_OFF)

def main_portion():
    grn_led.duty_u16(_PWM_LED_DUTY_ON)
    accel_data = bytearray(2000) # Shouldn't be larger than ~1800
    accel_mv = memoryview(accel_data)
    fifo_count_bytes = bytearray(2)
    for i in range(10):
        i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_COUNTH, fifo_count_bytes)
        fifo_count = (fifo_count_bytes[0] & 15) << 8 | fifo_count_bytes[1]
        print("FIFO Size (bytes): ", fifo_count)
        
        i2c.readfrom_mem_into(_ACCEL_ADDR, _FIFO_R_W, accel_mv[0:fifo_count])
        
        time.sleep_ms(3000)
    grn_led.duty_u16(_PWM_LED_DUTY_OFF)

init()
main_portion()