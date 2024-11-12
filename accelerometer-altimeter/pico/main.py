from machine import Pin, I2C
from accelerometer import MPU6050, GYRO_FS_1000, ACCEL_FS_16
from altimeter import BME280
import struct
import altimeter
import utime
import time
from io import open

# TODO: 
# * Maybe? Move file-writing into the main loop, but shove it onto a second thread
# * Copy files to PC with https://docs.micropython.org/en/latest/reference/mpremote.html#examples
# * Maybe? Look into SD Cards, see https://electrocredible.com/raspberry-pi-pico-micro-sd-card-module-micropython/ 

mpu = MPU6050(bus=0, sda=Pin(4), scl=Pin(5), ofs=(638, -3813, 866, 53, 17, 3), gyro=GYRO_FS_1000, accel=ACCEL_FS_16, rate=1)
bme = BME280(i2c=I2C(1, sda=Pin(10), scl=Pin(11), freq=400000))

print("AccFact: " + str(mpu.__accfact) + ", GyroFact: " + str(mpu.__gyrofact))

pitch = 0
roll = 0
tick = -1
prev_time = utime.ticks_ms()
curr_sec_data = [None] * 100

start_time = time.ticks_ms()
while tick < 100:
        tick = tick + 1
        TICK_RATE_MS = 10
        CURR_BARO_PRESSURE = 1011
        ###
        # ALTIMETER READING AND PRINTING
        ###
        # print("(Altimeter) Temperature: " + str(bme.values[0]))
        # print("Pressure: " + str(bme.values[1]) + " hPa")
        
        # # From https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf except using current pressure instead of default, which is 1013.25
        # print("Altitude: " + str((1-((float(bme.values[1])/CURR_BARO_PRESSURE) ** .190284)) * 145366.45) + "ft above sea level")

        # print("Humidity: " + str(bme.values[2]))

        ###
        # ACCELEROMETER READING AND PRINTING
        ###
        if tick < 100:
            curr_sec_data[tick] = mpu.raw_data
        
        utime.sleep(TICK_RATE_MS / 1000)
end_time = time.ticks_ms()
# start_time = time.ticks_ms()
with open('testfile.txt', 'w') as f:
    f.write(str(curr_sec_data))
# end_time = time.ticks_ms()
print(time.ticks_diff(end_time, start_time))
print("Done!")