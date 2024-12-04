from machine import Pin, PWM, Signal, I2C
from micropython import const
from struct import unpack
import time
import gc

# TODO: Lots, oy.
#   X. Implement pressure translation. See page 18 of the datasheet, and also https://github.com/pimoroni/icp10125-python/blob/main/icp10125/__init__.py
#       X And also altitude!
#   X. Implement configuration
#   3. Self-tests
#   X. Calibration
#   X. FIFO
#   6. General cleanup / organization.
#       * Classes?
#       * Methods, to be sure
#       X Constants. Lots of constants.
#   X. Bring over loop from 2040-w version
#   X. Bring over file i/o from 2040-w version
#   X. Port printing / decoding to "tera" implementation

############################
# Initialization / Startup #
############################

## Constants ##

# LED Control #
_PWM_LED_DUTY_ON = const(1024)
_PWM_LED_DUTY_OFF = const(0)

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
    initialize_accel()
    red_led.duty_u16(_PWM_LED_DUTY_OFF)

def main_portion():
    accel_data = bytearray(4000) # Shouldn't be larger than ~1800
    accel_mv = memoryview(accel_data)
    fifo_count_bytes = bytearray(2)
    
    with open('accel.bin', 'wb') as f:
        pass # nuke the file 
    
    for i in range(10):
        start_ms = time.ticks_ms()
        grn_led.duty_u16(_PWM_LED_DUTY_ON)
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

        blu_led.duty_u16(_PWM_LED_DUTY_ON)
        with open('accel.bin', 'ab') as f:
            f.write(accel_mv[0 : fifo_count])
        blu_led.duty_u16(_PWM_LED_DUTY_OFF)
        
        grn_led.duty_u16(_PWM_LED_DUTY_OFF)
        end_ms = time.ticks_ms()
        if shutdown_button.value() == 1:
            print("Got shutdown command. Quitting...")
            break
        time.sleep_ms(3000 - time.ticks_diff(end_ms, start_ms))

init()
main_portion()