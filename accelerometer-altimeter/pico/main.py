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
import micropython

############################
# Initialization / Startup #
############################

# Slow things down for lower power consumption, see neat chart on
# page 1341 of the RP2350 datasheet
machine.freq(65000000)

# Set up status LEDs
# red_led = Signal(Pin(18, Pin.OUT), invert = True)
# grn_led = Signal(Pin(19, Pin.OUT), invert = True)
# blu_led = Signal(Pin(20, Pin.OUT), invert = True)
red_led = Signal(Pin(20, Pin.OUT))
grn_led = Signal(Pin(19, Pin.OUT))
blu_led = Signal(Pin(18, Pin.OUT))

# Set up save / shutdown button
# shutdown_button = Signal(Pin(23, Pin.IN), invert = True)
shutdown_button = Signal(Pin(21, Pin.IN))

# Red LED indicates the system is initializing...
red_led.on()

# Set constants
_TICK_RATE_MS = const(10) # How often to take a gyro/accel reading
_TICKS_PER_ALTI = const(5) # How often (in ticks) to take an alti reading
_TICKS_PER_SAVE = const(1000) # How often (in ticks) to save to flash
_ACCEL_BYTES_PER_READ = const(14) # Bytes for a full read of the accel
_ACCEL_BYTES_PER_SAVE = const(_TICKS_PER_SAVE * _ACCEL_BYTES_PER_READ)
_ALTI_BYTES_PER_READ = const(8) # Bytes for a full read of the alti
_ALTI_BYTES_PER_SAVE = const((_TICKS_PER_SAVE // _TICKS_PER_ALTI) * _ALTI_BYTES_PER_READ) 
_ACCEL_BARRAY_SIZE = const(_ACCEL_BYTES_PER_SAVE * 2) # Memory for accel data
_ALTI_BARRAY_SIZE = const(_ALTI_BYTES_PER_SAVE * 2) # Memory for alti data
_DURATION_MINS = const(1)
_DURATION_TICKS = const(((_DURATION_MINS * 60) * 1000) // _TICK_RATE_MS)
_RESET_DATA = const(False) # True to wipe all recorded data and start over

# Make sure our constants aren't set wrong / going to break things
assert _TICKS_PER_SAVE % _TICKS_PER_ALTI == 0, "_TICKS_PER_SAVE must be a multiple of _TICKS_PER_ALTI"
assert ((_DURATION_MINS * 60) * 1000) % _TICK_RATE_MS == 0, "_DURATION_MINS must (after converting to ms) be a multiple of _TICK_RATE_MS"

# Prep filesystem
os.chdir("/data")
if _RESET_DATA:
    # Wipe all existing data
    for dir in os.listdir():
        os.chdir(dir)
        for file in os.listdir():
            os.remove(file)
        os.chdir("/data")
        os.rmdir(dir)
    os.mkdir("1")
    os.chdir("1")
else:
    # Make a new directory named as an integer one higher than the highest 
    # existing directory
    new_dir = str(int(sorted([int(i) for i in os.listdir()]).pop()) + 1)
    os.mkdir(new_dir)
    os.chdir(new_dir)

# Initialize sensors
mpu = MPU6050(bus=0, sda=Pin(4), scl=Pin(5), ofs=(638, -3813, 866, 53, 17, 3), gyro=GYRO_FS_1000, accel=ACCEL_FS_16, rate=1)
bme = BME280(i2c=I2C(1, sda=Pin(10), scl=Pin(11), freq=400000))

############################
# File I/O                 #
############################

# Runs as a thread so the main polling can continue while we save the data
def write_files_thd(accel_data : memoryview, alti_data : memoryview, seq_num : int) -> None:
    # Blue LED means saving to disk
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

@micropython.native
def main_portion():
    # These arrays are twice the size of the amount of data recorded between 
    # two saves. The idea is to:
    # 1. Initialize them only once, 
    # 2. First, write to the first half,
    # 3. Then, save the first half to flash while writing to the second half,
    # 4. Then, save the second half to flash while writing to the first half
    # 5. and keep cycling like that (steps 3 and 4) until we're done.
    accel_data = bytearray(_ACCEL_BARRAY_SIZE)
    alti_data = bytearray(_ALTI_BARRAY_SIZE)
    mv_accel_data = memoryview(accel_data) # Memoryviews for efficiency
    mv_alti_data = memoryview(alti_data)
    tick = -1 # The current tick, ie, current sample number
    seq_num = 0 # Tracks how many writes to flash we've done
    accel_index_nxt = 0 # See accel_index calculation below
    alti_index_nxt = 0 # See alti_index calculation below

    red_led.off() # Initialization is over at this point
    grn_led.on() # Green LED means in the main processing loop

    while tick <= _DURATION_TICKS:
        tick += 1
                
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

        # If true, the user requests that we wrap it up, so we write whatever we have to disk and bail out.
        if shutdown_button.value() == 1:
            print("Got shutdown command, saving state and then quitting...")
            seq_num += 1
            if seq_num & 1: # seq_num is odd, write the first half of the array
                _thread.start_new_thread(write_files_thd, (mv_accel_data[:_ACCEL_BYTES_PER_SAVE], mv_alti_data[:_ALTI_BYTES_PER_SAVE], seq_num))
            else: # seq_num is even, write the second half of the array
                _thread.start_new_thread(write_files_thd, (mv_accel_data[_ACCEL_BYTES_PER_SAVE:], mv_alti_data[_ALTI_BYTES_PER_SAVE:], seq_num)) 
            return

        # We write from the previous index...
        accel_index = accel_index_nxt % _ACCEL_BARRAY_SIZE
        # ... to the previous index + the number of bytes for one reading
        accel_index_nxt = (accel_index + _ACCEL_BYTES_PER_READ) 
        mv_accel_data[accel_index : accel_index_nxt] = mpu.raw_data

        if tick % _TICKS_PER_ALTI == 0:
            # We write from the previous index...
            alti_index = alti_index_nxt % _ALTI_BARRAY_SIZE
            # ... to the previous index + the number of bytes for one reading
            alti_index_nxt = (alti_index + _ALTI_BYTES_PER_READ) 
            mv_alti_data[alti_index : alti_index + _ALTI_BYTES_PER_READ] = bme.rawer_data

            if tick % _TICKS_PER_SAVE == 0 and tick > 0:
                seq_num += 1
                if seq_num & 1: # seq_num is odd, write the first half of the array
                    _thread.start_new_thread(write_files_thd, (mv_accel_data[:_ACCEL_BYTES_PER_SAVE], mv_alti_data[:_ALTI_BYTES_PER_SAVE], seq_num))
                else: # seq_num is even, write the second half of the array
                    _thread.start_new_thread(write_files_thd, (mv_accel_data[_ACCEL_BYTES_PER_SAVE:], mv_alti_data[_ALTI_BYTES_PER_SAVE:], seq_num)) 
        
        utime.sleep_ms(_TICK_RATE_MS)
        # machine.lightsleep(_TICK_RATE_MS)

main_portion()

grn_led.off()
utime.sleep_ms(500) # give files time to finish writing
# machine.lightsleep(_TICK_RATE_MS)
print("Done!")