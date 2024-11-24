from machine import Pin, I2C, Signal
from accelerometer import MPU6050, GYRO_FS_1000, ACCEL_FS_16
from altimeter import BME280
import utime
import time
import os
from io import open
import _thread
import machine
from micropython import const

# TODO: Spend some time on https://docs.micropython.org/en/latest/reference/speed_python.html#micropython-code-improvements -- probably ways to improve this code a bit
#   * Should also profile / work on main loop speed / power consumption. Memoryviews instead of bytearrays? Indexing instead of appending?

############################
# Initialization / Startup #
############################

# Slow things down for lower power consumption, see neat chart on
# page 1341 of the RP2350 datasheet
machine.freq(25000000)

# Something like, can't test this yet...
# red_led = Signal(Pin(18, Pin.OUT), invert = True)
# grn_led = Signal(Pin(19, Pin.OUT), invert = True)
# blu_led = Signal(Pin(20, Pin.OUT), invert = True)
red_led = Signal(Pin(20, Pin.OUT))
grn_led = Signal(Pin(19, Pin.OUT))
blu_led = Signal(Pin(18, Pin.OUT))

# shutdown_button = Signal(Pin(23, Pin.IN), invert = True)
shutdown_button = Signal(Pin(21, Pin.IN))

# Red LED indicates the system is still initializing...
red_led.on()

# Set global constants
_TICK_RATE_MS = const(10)
_DURATION_TICKS = const(3000)
_RESET_DATA = const(False)

os.chdir("/data")
if _RESET_DATA:
    for dir in os.listdir():
        os.chdir(dir)
        for file in os.listdir():
            os.remove(file)
        os.chdir("/data")
        os.rmdir(dir)
    os.mkdir("1")
    os.chdir("1")
else:
    new_dir = str(int(sorted([int(i) for i in os.listdir()]).pop()) + 1)
    os.mkdir(new_dir)
    os.chdir(new_dir)

# Initialize sensors
mpu = MPU6050(bus=0, sda=Pin(4), scl=Pin(5), ofs=(638, -3813, 866, 53, 17, 3), gyro=GYRO_FS_1000, accel=ACCEL_FS_16, rate=1)
bme = BME280(i2c=I2C(1, sda=Pin(10), scl=Pin(11), freq=400000))

def write_files_thd(accel_data, alti_data, seq_num):
    blu_led.on()
    str_seq_num = str(seq_num)
    print("len(accel_data) = " + str(len(accel_data)))
    print("len(alti_data) = " + str(len(alti_data)))
    start_time = time.ticks_ms()
    with open('accel' + str_seq_num + '.bin', 'wb') as f:
        f.write(accel_data)
    with open('alti' + str_seq_num + '.bin', 'wb') as f:
        f.write(alti_data)
    os.sync()
    end_time = time.ticks_ms()
    print(time.ticks_diff(end_time, start_time))
    blu_led.off()
    return

def main_portion():
    accel_data = bytearray()
    alti_data = bytearray()
    tick = 0
    seq_num = 0

    red_led.off()
    grn_led.on()

    while tick <= _DURATION_TICKS:
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

        if shutdown_button.value() == 1:
            # User requests that we wrap it up, so we write whatever we have to disk and bail out.
            print("Got shutdown command, saving state and then quitting...")
            _thread.start_new_thread(write_files_thd, (accel_data, alti_data, seq_num))
            return

        tick_mod_five = tick % 5
        tick_mod_thou = tick % 1000

        accel_data += mpu.raw_data
        if tick_mod_five == 0:
            alti_data += bme.rawer_data
        if tick_mod_thou == 0:
            seq_num = seq_num + 1
            _thread.start_new_thread(write_files_thd, (accel_data, alti_data, seq_num))
    
        utime.sleep_ms(_TICK_RATE_MS)
        
        if tick_mod_thou == 0:
            # wait until after sleep to (hopefully) avoid race 
            accel_data = bytearray()
            alti_data = bytearray()

main_portion()
grn_led.off()
utime.sleep_ms(500) # give files time to finish writing
print("Done!")




