from machine import Pin, I2C
from accelerometer import MPU6050
from altimeter import BME280
import altimeter
import utime

# TODO: Look into SD Cards, see https://electrocredible.com/raspberry-pi-pico-micro-sd-card-module-micropython/ 

mpu_i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
bme_i2c = I2C(1, sda=Pin(10), scl=Pin(11), freq=400000)

mpu = MPU6050()
bme = BME280(i2c=bme_i2c) 

pitch = 0
roll = 0
prev_time = utime.ticks_ms()

while True:
        TICK_RATE_MS = 5000
        CURR_BARO_PRESSURE = 1019
        ###
        # ALTIMETER READING AND PRINTING
        ###
        print("(Altimeter) Temperature: " + str(bme.values[0]))
        print("Pressure: " + str(bme.values[1]) + " hPa")
        
        # From https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf except using current pressure instead of default, which is 1013.25
        print("Altitude: " + str((1-((float(bme.values[1])/CURR_BARO_PRESSURE) ** .190284)) * 145366.45) + "ft above sea level")

        print("Humidity: " + str(bme.values[2]))

        ###
        # ACCELEROMETER READING AND PRINTING
        ###
        data = mpu.get_mpu6050_data(mpu_i2c)
        curr_time = utime.ticks_ms()
        dt = (curr_time - prev_time) / TICK_RATE_MS

        tilt_x, tilt_y, tilt_z = mpu.calculate_tilt_angles(data["accel"])
        pitch, roll = mpu.complementary_filter(pitch, roll, data["gyro"], dt)

        prev_time = curr_time

        print("(Accelerometer) Temperature: {:.2f} *C".format(data["temp"]))
        print(
            "Tilt angles: X: {:.2f}, Y: {:.2f}, Z: {:.2f} degrees".format(
                tilt_x, tilt_y, tilt_z
            )
        )
        print("Pitch: {:.2f}, Roll: {:.2f} degrees".format(pitch, roll))
        print(
            "Acceleration: X: {:.2f}, Y: {:.2f}, Z: {:.2f} g".format(
                data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]
            )
        )
        print(
            "Gyroscope: X: {:.2f}, Y: {:.2f}, Z: {:.2f} */s".format(
                data["gyro"]["x"], data["gyro"]["y"], data["gyro"]["z"]
            )
        )

        utime.sleep(TICK_RATE_MS / 1000)