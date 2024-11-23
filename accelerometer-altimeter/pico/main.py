from machine import Pin, I2C
from accelerometer import MPU6050, GYRO_FS_1000, ACCEL_FS_16
from altimeter import BME280
import utime
import time
from io import open
import _thread
import machine

# TODO: 
# * Maybe? Move file-writing into the main loop, but shove it onto a second thread
# * Copy files to PC with https://docs.micropython.org/en/latest/reference/mpremote.html#examples
# * Maybe? Look into SD Cards, see https://electrocredible.com/raspberry-pi-pico-micro-sd-card-module-micropython/ 
#   * Looks like we only have a smidge over 9 minutes of data if we fill the pico (9040) with 100 accelerometer readings per second + 20 altimeter readings per second.

mpu = MPU6050(bus=0, sda=Pin(4), scl=Pin(5), ofs=(638, -3813, 866, 53, 17, 3), gyro=GYRO_FS_1000, accel=ACCEL_FS_16, rate=1)
bme = BME280(i2c=I2C(1, sda=Pin(10), scl=Pin(11), freq=400000))

# print("AccFact: " + str(mpu.__accfact) + ", GyroFact: " + str(mpu.__gyrofact))

# pitch = 0
# roll = 0
machine.freq(48000000)
# prev_time = utime.ticks_ms()
TICK_RATE_MS = 10

# create a global lock
# lock = _thread.allocate_lock()

def write_files_thd(accel_data, alti_data, seq_num):
    # global lock
    # lock.acquire()
    str_seq_num = str(seq_num)
    print("len(accel_data) = " + str(len(accel_data)))
    print("len(alti_data) = " + str(len(alti_data)))
    start_time = time.ticks_ms()
    with open('/data/accel' + str_seq_num + '.bin', 'wb') as f:
        f.write(accel_data)
    with open('/data/alti' + str_seq_num + '.bin', 'wb') as f:
        f.write(alti_data)
    end_time = time.ticks_ms()
    print(time.ticks_diff(end_time, start_time))
    # lock.release()
    return

# print(str(bme.rawer_data.hex())) # 8 bytes
# print(str(mpu.raw_data.hex())) # 14 bytes
# print(str(bme.calibration_data.hex()))

def main_portion():
    global lock
    accel_data = bytearray()
    alti_data = bytearray()
    tick = -1
    seq_num = 0
    # start_time = time.ticks_ms()
    while tick <= 3000:
        tick = tick + 1
        
        # CURR_BARO_PRESSURE = 1024
        
        """
        ###
        # ALTIMETER READING AND PRINTING
        ###
        # print("(Altimeter) Temperature: " + str(bme.values[0]))
        # print("Pressure: " + str(bme.values[1]) + " hPa")
        # # From https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf except using current pressure instead of default, which is 1013.25
        # print("Altitude: " + str((1-((float(bme.values[1])/CURR_BARO_PRESSURE) ** .190284)) * 145366.45) + "ft above sea level")
        # print("Humidity: " + str(bme.values[2]))
        # print("Raw bytes: " + str(bme.values[3]))
        # print("===========================")

        ###
        # ACCELEROMETER READING AND PRINTING
        ###
        # pitch, roll = mpu.angles
        # print("Pitch: {:.2f}, Roll: {:.2f} degrees".format(pitch, roll))
        # data = mpu.data
        # print("(Accelerometer) Temperature: {:.2f} *F".format(mpu.fahrenheit))
        # print(
        #     "Acceleration: X: {:.2f}, Y: {:.2f}, Z: {:.2f} g".format(
        #         data[0], data[1], data[2]
        #     )
        # )
        # print(
        #     "Gyroscope: X: {:.2f}, Y: {:.2f}, Z: {:.2f} */s".format(
        #         data[3], data[4], data[5]
        #     )
        # )
        # print("Raw bytes: " + bytes(data[6]).hex())
        # print("===========================")
        """

        accel_data += mpu.raw_data
        if tick % 5 == 0:
            alti_data += bme.rawer_data
        if tick % 1000 == 0 and tick != 0:
            seq_num = seq_num + 1
            _thread.start_new_thread(write_files_thd, (accel_data, alti_data, seq_num))
            accel_data = bytearray()
            alti_data = bytearray()
    
        utime.sleep_ms(TICK_RATE_MS)

main_portion()

# end_time = time.ticks_ms()
utime.sleep_ms(500) # give files time to finish writing
# lock.acquire() # avoid shutting down if the files are writing
print("Done!")




