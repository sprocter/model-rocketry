from machine import Pin, PWM, Signal, I2C
from micropython import const
from struct import unpack
import time

# TODO: Lots, oy.
#   X. Implement pressure translation. See page 18 of the datasheet, and also https://github.com/pimoroni/icp10125-python/blob/main/icp10125/__init__.py
#       X And also altitude!
#   2. Implement configuration
#   3. Self-tests
#   4. Calibration
#   5. FIFO
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
_LP_CONFIG = const(0x05) # Bank 0, pg 39
_PWR_MGMT_1 = const(0x06) # Bank 0, pg 39
_PWR_MGMT_2 = const(0x07) # Bank 0, pg 40
_ACCEL_XOUT_H = const(0x2D) # Bank 0
_ODR_ALIGN_EN = const(0x09) # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_1 = const(0x10) # Bank 2, pg 65
_ACCEL_SMPLRT_DIV_2 = const(0x11) # Bank 2, pg 65
_ACCEL_CONFIG = const(0x14) # Bank 2, pg 66
_ACCEL_CONFIG_2 = const(0x15) # Bank 2, pg 67

_ALTI_ADDR = const(0x63) # Default ICP10124 I2C Addr: 99
_CURR_BARO_PRESSURE = const(1007)

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

alti_calib_data = list()
i2c = I2C(id=0, scl=13, sda=12, freq=400000)

# This process is described in IC10125 datasheet section 5.10 (pg 18)
def read_alti_calib():
    # 0xC59500669C is the command to set up OTP read
    i2c.writeto(_ALTI_ADDR, b'\xC5\x95\x00\x66\x9C') 
    for i in range(4):
        # 0xC7F7 is the command to incrementally read out OTP
        i2c.writeto(_ALTI_ADDR, b'\xC7\xF7')
        temp_data = bytearray(3)
        i2c.readfrom_into(_ALTI_ADDR, temp_data)
        alti_calib_data.append(unpack(">h", temp_data[:2])[0])
    return alti_calib_data

def initialize_accel():
    # Begin with Bank 0 configuration
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b'\x00')

    # Reset the device to default settings
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x80') # 0b10000001
    time.sleep_ms(5)
    while i2c.readfrom_mem(_ACCEL_ADDR, _PWR_MGMT_1, 1) == 128:
        print("waiting")
        time.sleep_ms(5)

    # Turn low power off, temperature sensor off, use best available clock source
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_1, b'\x09') # 0b00001001
    time.sleep_ms(30)

    # Turn off the gyroscope, turn on the accelerometer
    i2c.writeto_mem(_ACCEL_ADDR, _PWR_MGMT_2, b'\x07') # 0b00000111

    # Operate accelerometer in duty cycled mode
    # i2c.writeto_mem(_ACCEL_ADDR, _LP_CONFIG, b'\x20') # 0b00100000

    # Now on to Bank 2 configuration
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, b'\x20') # 0b00100000
    time.sleep_ms(5)
    
    i2c.writeto_mem(_ACCEL_ADDR, _ODR_ALIGN_EN, b'\x01')

    # Sample rate = 10, rate_hz = 1125/(1 + rate) = 1125/11 = 112.5
    # Split across two bytes, 12 bits
    # MSB, 0b00000000
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_1, b'\x00') 
    # LSB, 0b00001010
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_SMPLRT_DIV_2, b'\x0A')
    
    # Low pass filter = 3, Accelerometer Full Scale 30g, DLPF Enabled
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG, b'\x1F') # 0b00011111

    # Average 4 samples (would be 1, but we're using the DLPF)
    i2c.writeto_mem(_ACCEL_ADDR, _ACCEL_CONFIG_2, b'\x01') # 0b00000001

    #Back to Bank 0
    i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL,  b'\x00')

    time.sleep_ms(100)
    # Expected power consumption: 120uA

def print_accel_data(reading : bytearray):
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading[:6])
    print(unpack(">hhh", reading[:6]))
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

def print_alti_data(reading : bytearray):
    raw_temp = unpack(">h", reading[:2])[0]
    temp_c = -45 + ((175/(2 ** 16)) * raw_temp)
    temp_f = temp_c * 1.8 + 32
    print("Temp (f):", temp_f)

    # reading[2], reading[5], and reading[9] are CRC bytes, and reading [7] 
    # "must be disregarded" Section 5.5, pg 15 of the datasheet
    raw_press = reading[3] << 16 | reading[4] << 8 | reading[6]
    p_Pa = [45000.0, 80000.0, 105000.0]
    LUT_lower = 3.5 * (2**20)
    LUT_upper = 11.5 * (2**20)
    quadr_factor = 1 / 16777216.0
    offst_factor = 2048.0

    t = raw_temp - 32768.0
    s1 = LUT_lower + float(alti_calib_data[0] * t * t) * quadr_factor
    s2 = offst_factor * alti_calib_data[3] + float(alti_calib_data[1] * t * t) * quadr_factor
    s3 = LUT_upper + float(alti_calib_data[2] * t * t) * quadr_factor
    
    C = (s1 * s2 * (p_Pa[0] - p_Pa[1]) +
    s2 * s3 * (p_Pa[1] - p_Pa[2]) +
    s3 * s1 * (p_Pa[2] - p_Pa[0])) / \
    (s3 * (p_Pa[0] - p_Pa[1]) +
    s1 * (p_Pa[1] - p_Pa[2]) +
    s2 * (p_Pa[2] - p_Pa[0]))
    A = (p_Pa[0] * s1 - p_Pa[1] * s2 - (p_Pa[1] - p_Pa[0]) * C) / (s1 - s2)
    B = (p_Pa[0] - A) * (s1 + C)
    pressure_pa = A + B / (C + raw_press)
    altitude = (1-(((pressure_pa/100)/_CURR_BARO_PRESSURE) ** .190284)) * 145366.45
    # print("Pressure (Pa): ", pressure_pa)
    print("Altitude (ft): ", altitude)

def init():
    initialize_accel()
    read_alti_calib()
    red_led.duty_u16(_PWM_LED_DUTY_OFF)

def main_portion():
    grn_led.duty_u16(_PWM_LED_DUTY_ON)
    for i in range(40):
        # Read from the accelerometer
        temp_data = bytearray(14)
        i2c.readfrom_mem_into(_ACCEL_ADDR, _ACCEL_XOUT_H, temp_data)
        print_accel_data(temp_data)

        # Read from altimeter
        temp_data_2 = bytearray(9)
        # Request Low Power, Temp 1st
        i2c.writeto(_ALTI_ADDR, b'\x60\x9C') 
        time.sleep_ms(3)
        i2c.readfrom_into(_ALTI_ADDR, temp_data_2)
        print_alti_data(temp_data_2)

        print("")
        
        time.sleep_ms(100)
    grn_led.duty_u16(_PWM_LED_DUTY_OFF)

init()
main_portion()