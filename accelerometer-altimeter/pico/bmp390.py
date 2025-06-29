"""A driver for a model rocket altimeter

This script contains initialization and usage functions for the Bosch BMP390: https://www.adafruit.com/product/4816

To get the device-specific coefficients, call print_packed_coeffs() after the driver has been initialized.

Note that none of this code is designed to be modified by the user.
"""

from machine import I2C
import time

MIN_RESOLUTION = const(1)
MAX_RESOLUTION = const(3)

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
    """A driver for a model rocket altimeter

    This class stores all data in a memoryview object the size of the BMP390's FIFO buffer -- 512 bytes. The basic workflow is to instantiate the driver, use it to initialize the device, wait a while (but not so long the FIFO buffer fills up / overflows), then read the FIFO buffer (which clears it) into the memoryview object, write that to a file or otherwise store it, then wait a while, read the FIFO buffer again, etc.
    """

    def __init__(self, resolution: int, i2c: I2C):
        """Initializes the driver.

        This method initializes globals according to the user's desired resolution: higher values mean more data readings -- thus more power consumption and more disk usage.

        :param int resolution: The amount of data to record, should be in the range [BMP390.MIN_RESOLUTION .. BMP390.MAX_RESOLUTION] (inclusive). Values less than BMP390.MIN_RESOLUTION will be interpreted as BMP390.MIN_RESOLUTION, values greater than BMP390.MAX_RESOLUTION will be interpreted as BMP390.MAX_RESOLUTION.
        :param I2C i2c: An initialized / connected I2C object allowing communication with the device.
        """

        if resolution < MIN_RESOLUTION:
            resolution = MIN_RESOLUTION
        elif resolution > MAX_RESOLUTION:
            resolution = MAX_RESOLUTION

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

    def print_packed_coeffs(self) -> None:
        """Prints the device-specific coeffecients needed to decode the data

        This will print the device's coefficients to the terminal. These coefficients will be a 42 character hex string. That string should be copied and pasted into the sibling python script (which should be located in ../tera).

        This only needs to be done once when you have a new device.
        """

        coeffs_packed = bytearray(21)
        self.i2c.readfrom_mem_into(_ALTI_ADDR, _CALIB_COEFFS, coeffs_packed)
        print('_PACKED_COEFFS = "', str(hex(int.from_bytes(coeffs_packed)))[2:], '"')

    def initialize_device(self) -> None:
        """This initializes the device when power is initially connected.

        This should be called when the device first receives power -- it will leave the device running and recording data at a rate determined by the user-selected resolution value.
        """

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
        """Read from the device's FIFO buffer.

        This checks how many bytes are available to read from the device,  reads those bytes into memory, then returns the number of bytes read so they can be written to disk.

        :return: The number of bytes read from the FIFO
        :rtype: int
        """
        fifo_count_bytes = bytearray(2)
        self.i2c.readfrom_mem_into(_ALTI_ADDR, _ALTI_FIFO_LENGTH_0, fifo_count_bytes)
        fifo_count = fifo_count_bytes[1] << 8 | fifo_count_bytes[0]
        # print("Alti FIFO: ", fifo_count)
        self.i2c.readfrom_mem_into(_ALTI_ADDR, _ALTI_FIFO_DATA, self.mv[0:fifo_count])
        return fifo_count

    def shutdown(self) -> None:
        """Shuts the device off. Call to save power."""
        # Set altimeter to sleep mode, turn off pressure and temperature sensors
        self.i2c.writeto_mem(_ALTI_ADDR, _ALTI_PWR_CTRL, b"\x00")  # RR00RR00
