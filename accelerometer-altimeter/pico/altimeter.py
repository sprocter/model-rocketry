from machine import Pin, I2C
import time
import utime
from ustruct import unpack, unpack_from
from array import array

# adapted from:
# * Main control flow, algorithms for getting data and making it readable: https://how2electronics.com/bme280-raspberry-pi-pico-w-web-server-weather-station/ 
# * Configuration options: 
#   * https://github.com/adafruit/Adafruit_CircuitPython_BME280/tree/main/adafruit_bme280
#   * https://cdn-learn.adafruit.com/assets/assets/000/115/588/original/bst-bme280-ds002.pdf?1664822559


# BME280 default address.
BME280_I2CADDR = 0x76

#####################################################
# Configuration options.                            #
# See 5.3/5.4 of the BME Data sheet for explanation #
#####################################################

# Humidity Oversample Modes
BME280_HUM_OFF = 0b000
BME280_HUM_OSMPL_1 = 0b001
BME280_HUM_OSMPL_2 = 0b010
BME280_HUM_OSMPL_4 = 0b011
BME280_HUM_OSMPL_8 = 0b100
BME280_HUM_OSMPL_16 = 0b101

# Temperature Oversample Modes
BME280_TEMP_OFF = 0b000
BME280_TEMP_OSMPL_1 = 0b001
BME280_TEMP_OSMPL_2 = 0b010
BME280_TEMP_OSMPL_4 = 0b011
BME280_TEMP_OSMPL_8 = 0b100
BME280_TEMP_OSMPL_16 = 0b101

# Pressure Oversample Modes
BME280_PRESS_OFF = 0b000
BME280_PRESS_OSMPL_1 = 0b001
BME280_PRESS_OSMPL_2 = 0b010
BME280_PRESS_OSMPL_4 = 0b011
BME280_PRESS_OSMPL_8 = 0b100
BME280_PRESS_OSMPL_16 = 0b101

# Register Settings
BME280_MODE_SLEEP = 0b00
BME280_MODE_FORCED = 0b01
BME280_MODE_NORMAL = 0b11

# Inactive Duration (only relevant in normal mode)
BME280_TSB_0_5MS = 0b000
BME280_TSB_10MS = 0b110
BME280_TSB_20MS = 0b111
BME280_TSB_62_5MS = 0b001
BME280_TSB_125MS = 0b010
BME280_TSB_250MS = 0b011
BME280_TSB_500MS = 0b100
BME280_TSB_1000MS = 0b101

# IIR Filter Coefficient
BME280_IIR_OFF = 0b000
BME280_IIR_COEF2 = 0b001
BME280_IIR_COEF4 = 0b010
BME280_IIR_COEF8 = 0b011
BME280_IIR_COEF16 = 0b100

# Configuration Register Addresses
BME280_REGISTER_CONTROL_HUM = 0xF2
BME280_REGISTER_CONTROL = 0xF4
BME280_REGISTER_CONFIG = 0xF5

class BME280:

    def __init__(
        self, 
        hum_osmpl=BME280_HUM_OSMPL_1,
        temp_osmpl=BME280_TEMP_OSMPL_1,
        press_osmpl=BME280_PRESS_OSMPL_16,
        mode=BME280_MODE_NORMAL,
        inactive_dur=BME280_TSB_0_5MS,
        iir=BME280_IIR_COEF16,
        address=BME280_I2CADDR,
        i2c=None,
        **kwargs
    ):
        self.hum_osmpl=hum_osmpl
        self.temp_osmpl=temp_osmpl
        self.press_osmpl=press_osmpl
        self.mode=mode
        self.inactive_dur=inactive_dur
        self.iir=iir
        self.address = address
        if i2c is None:
            raise ValueError("An I2C object is required.")
        self.i2c = i2c

        # load calibration data
        dig_88_a1 = self.i2c.readfrom_mem(self.address, 0x88, 26)
        dig_e1_e7 = self.i2c.readfrom_mem(self.address, 0xE1, 7)
        (
            self.dig_T1,
            self.dig_T2,
            self.dig_T3,
            self.dig_P1,
            self.dig_P2,
            self.dig_P3,
            self.dig_P4,
            self.dig_P5,
            self.dig_P6,
            self.dig_P7,
            self.dig_P8,
            self.dig_P9,
            _,
            self.dig_H1,
        ) = unpack("<HhhHhhhhhhhhBB", dig_88_a1)

        self.dig_H2, self.dig_H3 = unpack("<hB", dig_e1_e7)
        e4_sign = unpack_from("<b", dig_e1_e7, 3)[0]
        self.dig_H4 = (e4_sign << 4) | (dig_e1_e7[4] & 0xF)

        e6_sign = unpack_from("<b", dig_e1_e7, 5)[0]
        self.dig_H5 = (e6_sign << 4) | (dig_e1_e7[4] >> 4)

        self.dig_H6 = unpack_from("<b", dig_e1_e7, 6)[0]

        # temporary data holders which stay allocated
        self._l1_barray = bytearray(1)
        self._l8_barray = bytearray(8)
        self._l3_resultarray = array("i", [0, 0, 0])

        # Write our configuration options
        self._l1_barray[0] = self.hum_osmpl
        self.i2c.writeto_mem(self.address, BME280_REGISTER_CONTROL_HUM, self._l1_barray)
        self._l1_barray[0] = self.temp_osmpl << 5 | self.press_osmpl << 2 | self.mode
        self.i2c.writeto_mem(self.address, BME280_REGISTER_CONTROL,  self._l1_barray)
        self._l1_barray[0] = self.inactive_dur << 5 | self.iir << 2 | 0
        self.i2c.writeto_mem(self.address, BME280_REGISTER_CONFIG,  self._l1_barray)

        self.t_fine = 0

        time.sleep_ms(100)  # Wait a bit

    @property
    def calibration_data(self):
        # load calibration data
        p1 = bytearray(26)
        p2 = bytearray(7)
        self.i2c.readfrom_mem_into(self.address, 0x88, p1)
        self.i2c.readfrom_mem_into(self.address, 0xE1, p2)
        return p1 + p2

    @property
    def rawer_data(self):
        # burst readout from 0xF7 to 0xFE, recommended by datasheet
        self.i2c.readfrom_mem_into(self.address, 0xF7, self._l8_barray)
        return self._l8_barray

    def read_raw_data(self, result):
        """Reads the raw (uncompensated) data from the sensor.

        Args:
            result: array of length 3 or alike where the result will be
            stored, in temperature, pressure, humidity order
        Returns:
            None
        """

        # burst readout from 0xF7 to 0xFE, recommended by datasheet
        self.i2c.readfrom_mem_into(self.address, 0xF7, self._l8_barray)
        readout = self._l8_barray
        # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
        # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
        # humidity(0xFD): (msb << 8) | lsb
        raw_hum = (readout[6] << 8) | readout[7]

        result[0] = raw_temp
        result[1] = raw_press
        result[2] = raw_hum

    def read_compensated_data(self, result=None):
        """Reads the data from the sensor and returns the compensated data.

        Args:
            result: array of length 3 or alike where the result will be
            stored, in temperature, pressure, humidity order. You may use
            this to read out the sensor without allocating heap memory

        Returns:
            array with temperature, pressure, humidity. Will be the one from
            the result parameter if not None
        """
        self.read_raw_data(self._l3_resultarray)
        raw_temp, raw_press, raw_hum = self._l3_resultarray
        # temperature
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (
            ((((raw_temp >> 4) - self.dig_T1) * ((raw_temp >> 4) - self.dig_T1)) >> 12)
            * self.dig_T3
        ) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8

        # pressure
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            pressure = 0
        else:
            p = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)

        # humidity
        h = self.t_fine - 76800
        h = (
            (((raw_hum << 14) - (self.dig_H4 << 20) - (self.dig_H5 * h)) + 16384) >> 15
        ) * (
            (
                (
                    (
                        (
                            ((h * self.dig_H6) >> 10)
                            * (((h * self.dig_H3) >> 11) + 32768)
                        )
                        >> 10
                    )
                    + 2097152
                )
                * self.dig_H2
                + 8192
            )
            >> 14
        )
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = 0 if h < 0 else h
        h = 419430400 if h > 419430400 else h
        humidity = h >> 12

        if result:
            result[0] = temp
            result[1] = pressure
            result[2] = humidity
            return result

        return array("i", (temp, pressure, humidity))

    @property
    def values(self):
        """human readable values"""

        t, p, h = self.read_compensated_data()

        p = p // 256
        pi = p // 100
        pd = p - pi * 100

        hi = h // 1024
        hd = h * 100 // 1024 - hi * 100
        return (
            "{}*C".format(t / 100),
            #"{}.{:02d} hPa".format(pi, pd),
            "{}.{:02d}".format(pi, pd),
            "{}.{:02d} %".format(hi, hd),
            self._l8_barray.hex()
        )

def _Run_Standalone():
    """Used to test the altimeter directly, not API.
    """
    i2c = I2C(1, sda=Pin(10), scl=Pin(11), freq=400000)
    while True:
        TICK_RATE_MS = 500
        bme = BME280(i2c=i2c)
        print("Temperature: " + str(bme.values[0]))
        print("Pressure: " + str(bme.values[1]))
        print("Humidity: " + str(bme.values[2]))
        print("Altitude: " + str((1-((float(bme.values[1])/1022) ** .190284)) * 145366.45) + "ft above sea level")
        
        utime.sleep(TICK_RATE_MS / 1000)

# _Run_Standalone()

