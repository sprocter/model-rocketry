from machine import Pin, I2C
import utime
import math

# (incredibly) lightly adapted from https://how2electronics.com/interfacing-mpu6050-with-raspberry-pi-pico-micropython/


# TODO:
# * Implement calibration, see guide: https://shillehtek.com/blogs/news/how-to-calibrate-mpu6050
# * Tweak configuration from defaults (and honestly use like constants or something) see implementation in https://github.com/shillehbean/youtube-channel/blob/main/imu.py and full description in MPU-6000/MPU-6050 Register Map and Descriptions

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
TEMP_OUT_H = 0x41
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050:

    def init_mpu6050(self, i2c, address=0x68):
        i2c.writeto_mem(address, PWR_MGMT_1, b"\x00")
        utime.sleep_ms(100)
        i2c.writeto_mem(address, SMPLRT_DIV, b"\x07")
        i2c.writeto_mem(address, CONFIG, b"\x00")
        i2c.writeto_mem(address, GYRO_CONFIG, b"\x00")
        i2c.writeto_mem(address, ACCEL_CONFIG, b"\x00")


    def read_raw_data(self, i2c, addr, address=0x68):
        high = i2c.readfrom_mem(address, addr, 1)[0]
        low = i2c.readfrom_mem(address, addr + 1, 1)[0]
        value = high << 8 | low
        if value > 32768:
            value = value - 65536
        return value


    def get_mpu6050_data(self, i2c):
        temp = self.read_raw_data(i2c, TEMP_OUT_H) / 340.0 + 36.53
        accel_x = self.read_raw_data(i2c, ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_raw_data(i2c, ACCEL_XOUT_H + 2) / 16384.0
        accel_z = self.read_raw_data(i2c, ACCEL_XOUT_H + 4) / 16384.0
        gyro_x = self.read_raw_data(i2c, GYRO_XOUT_H) / 131.0
        gyro_y = self.read_raw_data(i2c, GYRO_XOUT_H + 2) / 131.0
        gyro_z = self.read_raw_data(i2c, GYRO_XOUT_H + 4) / 131.0

        return {
            "temp": temp,
            "accel": {
                "x": accel_x,
                "y": accel_y,
                "z": accel_z,
            },
            "gyro": {
                "x": gyro_x,
                "y": gyro_y,
                "z": gyro_z,
            },
        }


    def calculate_tilt_angles(self, accel_data):
        x, y, z = accel_data["x"], accel_data["y"], accel_data["z"]

        tilt_x = math.atan2(y, math.sqrt(x * x + z * z)) * 180 / math.pi
        tilt_y = math.atan2(-x, math.sqrt(y * y + z * z)) * 180 / math.pi
        tilt_z = math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi

        return tilt_x, tilt_y, tilt_z


    def complementary_filter(self, pitch, roll, gyro_data, dt, alpha=0.98):
        pitch += gyro_data["x"] * dt
        roll -= gyro_data["y"] * dt

        pitch = (
            alpha * pitch
            + (1 - alpha)
            * math.atan2(
                gyro_data["y"],
                math.sqrt(
                    gyro_data["x"] * gyro_data["x"] + gyro_data["z"] * gyro_data["z"]
                ),
            )
            * 180
            / math.pi
        )
        roll = (
            alpha * roll
            + (1 - alpha)
            * math.atan2(
                -gyro_data["x"],
                math.sqrt(
                    gyro_data["y"] * gyro_data["y"] + gyro_data["z"] * gyro_data["z"]
                ),
            )
            * 180
            / math.pi
        )

        return pitch, roll

def runStandalone():

    i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

    mpu = MPU6050()

    pitch = 0
    roll = 0
    prev_time = utime.ticks_ms()

    while True:
        TICK_RATE_MS = 1000

        data = mpu.get_mpu6050_data(i2c)
        curr_time = utime.ticks_ms()
        dt = (curr_time - prev_time) / TICK_RATE_MS

        tilt_x, tilt_y, tilt_z = mpu.calculate_tilt_angles(data["accel"])
        pitch, roll = mpu.complementary_filter(pitch, roll, data["gyro"], dt)

        prev_time = curr_time

        print("Temperature: {:.2f} *C".format(data["temp"]))
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

# runStandalone()