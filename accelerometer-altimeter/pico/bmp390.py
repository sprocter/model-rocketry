from machine import I2C
import time

# Altimeter Registers #
_ALTI_ADDR = const(0x77)  # Default BMP390 I2C Addr: 119
_CALIB_COEFFS = const(0x31)  # pg 28
_ALTI_FIFO_LENGTH_0 = const(0x12)  # pg 33
_ALTI_FIFO_DATA = const(0x14)  # pg 34
_ALTI_FIFO_CONFIG_1 = const(0x17)  # pg 34
_ALTI_FIFO_CONFIG_2 = const(0x18)  # pg 35
_ALTI_PWR_CTRL = const(0x1B)  # pg 36
_ALTI_OSR = const(0x1C)  # pg 37
_ALTI_ODR = const(0x1D)  # pg 37
_ALTI_CONFIG = const(0x1F)  # pg 39
_ALTI_CMD = const(0x7E)  # pg 39


class BMP390:

    def __init__(self, resolution: int, i2c: I2C):
        # Resolution #
        if resolution == 1:
            # Temperature 1 (001), Pressure 8 (100)
            self._ALTI_OVERSAMPLE = b"\x03"
            # 25 Hz / 40ms sampling period
            self._ALTI_DATARATE = b"\x03"
        elif resolution == 2:
            # Temperature 1 (000), Pressure 8 (011)
            self._ALTI_OVERSAMPLE = b"\x03"
            # 50 Hz / 20ms sampling period
            self._ALTI_DATARATE = b"\x02"
        elif resolution == 3:
            # Temperature 1 (000), Pressure 2 (001)
            self._ALTI_OVERSAMPLE = b"\x01"
            # 100 Hz / 10ms sampling period
            self._ALTI_DATARATE = b"\x01"

        # Initialize buffer to the size of the sensor's FIFO
        self.mv = memoryview(bytearray(512))
        self.i2c = i2c

    # Call to get the device-specific coefficients needed to decode altimeter data
    def print_packed_coeffs(self) -> None:
        coeffs_packed = bytearray(21)
        self.i2c.readfrom_mem_into(_ALTI_ADDR, _CALIB_COEFFS, coeffs_packed)
        print('_PACKED_COEFFS = "', str(hex(int.from_bytes(coeffs_packed)))[2:], '"')

    def initialize_device(self) -> None:
        i2c = self.i2c
        # Reset the device
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_CMD, b"\xb6")
        time.sleep_ms(5)

        # Turn on normal mode, the pressure sensor, and the temperature sensor
        # RR11RR11
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_PWR_CTRL, b"\x33")

        # Turn on FIFO, don't stop when FIFO is full, don't return "sensortime"
        # frames, do store pressure data, do store temperature data
        # RRR11001
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_FIFO_CONFIG_1, b"\x19")

        # Store filtered data, don't downsample
        # RRR01000
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_FIFO_CONFIG_2, b"\x08")

        # Oversampling: See Resolution section of constants
        # RR000001
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_OSR, self._ALTI_OVERSAMPLE)

        # Data rate: See Resolution section of constants
        # RRR00001
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_ODR, self._ALTI_DATARATE)

        # IIR Filter: 2 (010R)
        # RRR010R
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_CONFIG, b"\x04")

        # Clear FIFO now that we've messed with the settings
        i2c.writeto_mem(_ALTI_ADDR, _ALTI_CMD, b"\xb0")

    @micropython.native
    def read_fifo(self) -> int:
        fifo_count_bytes = bytearray(2)
        self.i2c.readfrom_mem_into(_ALTI_ADDR, _ALTI_FIFO_LENGTH_0, fifo_count_bytes)
        fifo_count = fifo_count_bytes[1] << 8 | fifo_count_bytes[0]
        # print("Alti FIFO: ", fifo_count)
        self.i2c.readfrom_mem_into(_ALTI_ADDR, _ALTI_FIFO_DATA, self.mv[0:fifo_count])
        return fifo_count

    def shutdown(self) -> None:
        # Set altimeter to sleep mode, turn off pressure and temperature sensors
        self.i2c.writeto_mem(_ALTI_ADDR, _ALTI_PWR_CTRL, b"\x00")  # RR00RR00
