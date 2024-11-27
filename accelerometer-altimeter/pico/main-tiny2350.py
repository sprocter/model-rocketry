from machine import Pin, Signal, I2C
from micropython import const
from struct import unpack
import time

# TODO: Lots, oy.
#   1. Implement pressure translation. See page 18 of the datasheet, and also https://github.com/pimoroni/icp10125-python/blob/main/icp10125/__init__.py
#       * And also altitude!
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

# Slow things down for lower power consumption, see neat chart on
# page 1341 of the RP2350 datasheet
# machine.freq(65000000)

# Set up status LEDs
red_led = Signal(Pin(18, Pin.OUT), invert = True)
grn_led = Signal(Pin(19, Pin.OUT), invert = True)
blu_led = Signal(Pin(20, Pin.OUT), invert = True)

red_led.off()
grn_led.off()
blu_led.off()

# Set up save / shutdown button
shutdown_button = Signal(Pin(23, Pin.IN), invert = True)

# Red LED indicates the system is initializing...
# red_led.on()

# Initialize sensors
i2c = I2C(id=0, scl=13, sda=12, freq=400000)

_ACCEL_ADDR = const(0x68) # Default ICM20649 I2C Addr: 104
_ALTI_ADDR = const(0x63) # Default ICP10124 I2C Addr: 99
_REG_BANK_SEL = const(0x7F) # All banks
_TEMP_OUT_H = const(0x39) # Bank 0
_ACCEL_XOUT_H = const(0x2D) # Bank 0
_GYRO_CONFIG_1 = const(0x01) # Bank 2, pg 61
_ACCEL_CONFIG = const(0x14) # Bank 2, pg 66

def print_accel_data(reading : bytearray):
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", reading[0:6])
    raw_gyro_x, raw_gyro_y, raw_gyro_z = unpack(">hhh", reading[6:12])
    
    # Accelerometer "sensitivity scale factors" on page 13
    accel_x = (raw_accel_x / 8192)
    accel_y = (raw_accel_y / 8192)
    accel_z = (raw_accel_z / 8192)

    # Gyroscope "sensitivity scale factors" on page 12
    gyro_x = (raw_gyro_x / 65.5)
    gyro_y = (raw_gyro_y / 65.5)
    gyro_z = (raw_gyro_z / 65.5)

    raw_temp = unpack(">h", reading[12:14])[0]
    temp_c = (raw_temp / 333.87) + 21.0
    temp_f = temp_c * 1.8 + 32
    print("Accel (x, y, z):", accel_x, accel_y, accel_z)
    print("Gyro (x, y, z):", gyro_x, gyro_y, gyro_z)
    print("Temp (Accel):", temp_f)

def print_alti_data(reading : bytearray):
    raw_temp = unpack(">h", reading[0:2])[0]
    temp_c = -45 + ((175/(2 ** 16)) * raw_temp)
    temp_f = temp_c * 1.8 + 32
    print("Temp (Alti):", temp_f)
    # pressure = (result[1] << 8) | (result[2] >> 8)

# red_led.off()
# grn_led.off()
# write_byte = bytearray(1)
# write_byte[0] = 0b00000000
# i2c.writeto_mem(_ACCEL_ADDR, _REG_BANK_SEL, write_byte)
temp_data = bytearray(14)
i2c.readfrom_mem_into(_ACCEL_ADDR, _ACCEL_XOUT_H, temp_data)
print_accel_data(temp_data)

# print(bin(int.from_bytes(i2c.readfrom_mem(_ACCEL_ADDR, _GYRO_CONFIG_1, 1))))
# print(bin(int.from_bytes(i2c.readfrom_mem(_ACCEL_ADDR, _ACCEL_CONFIG, 1))))
# print(temp_data)
# print(i2c.readfrom_mem(_ACCEL_ADDR, 0x00, 1))
# i2c.writeto(_ALTI_ADDR, bytes.fromhex("EFC8")) # Request "ID" register
# print(bin(int.from_bytes(i2c.readfrom(_ALTI_ADDR, 2))))
temp_data_2 = bytearray(9)
i2c.writeto(_ALTI_ADDR, bytes.fromhex("609C")) # Request Low Power, Temp 1st
time.sleep_ms(2)
i2c.readfrom_into(_ALTI_ADDR, temp_data_2)
print_alti_data(temp_data_2)


# i2c.readfrom_mem_into(_ALTI_ADDR, 0x00, temp_data)