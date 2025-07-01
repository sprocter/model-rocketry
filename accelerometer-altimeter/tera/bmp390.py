"""A module for decoding data from a model rocket altimeter

This script contains methods to read data from the Bosch BMP390: https://www.adafruit.com/product/4816

To get the device-specific coefficients, call print_packed_coeffs() on the device after the driver has been initialized.

Note that none of this code is designed to be modified by the user.
"""

from decimal import Decimal
from typing import Generator, Dict
from struct import unpack

from more_itertools import peekable

_PACKED_COEFFS = "996e384df93a1a52140601684e9d6003fabd0f05f5"
"""Coefficients necessary to interpret the altimeter readings.

To get this string, call `BMP390.print_packed_coeffs()` on the device when it's connected to the BMP290
"""


class BMP390:
    """A class for decoding data from a model rocket altimeter

    After instantiation, this class has three globals -- altitudes, speeds, and temperatures -- which are dictionaries that map timestamps to decoded sensor readings. The timestamps' units are seconds; they start at 0.0 and increase by the amount of time between samples as specified by the samplerate provided when the class is initialized. Altitudes are provided in feet, speeds are in miles per hour, and temperatures in fahrenheit. Altitudes relative to launch are also available via the `relative_altitudes` property, and have the same data structure and units as the globals.
    """

    def __init__(self, samplerate_hz: int, barometric_pressure: int = 1015):
        """Iniitalizes the class.

        This method initializes the device-specific coefficients (stored in the `_PACKED_COEFFS` global) and the various internal globals used when decoding the binary-format senor readings.

        :param int samplerate_hz: The number of readings stored by the sensor per second.
        :param int barometric_pressure: The atmopsheric pressure at the date and location of the launch in millibar. If not provided, a reasonable default is used -- relative altitudes will still be correct.
        """
        coeffs_unpacked = unpack("<HHbhhbbHHbbhbb", bytes.fromhex(_PACKED_COEFFS))
        self.par_t1 = coeffs_unpacked[0] / (2**-8)
        self.par_t2 = coeffs_unpacked[1] / (2**30)
        self.par_t3 = coeffs_unpacked[2] / (2**48)

        self.par_p1 = (coeffs_unpacked[3] - (2**14)) / (2**20)
        self.par_p2 = (coeffs_unpacked[4] - (2**14)) / (2**29)
        self.par_p3 = coeffs_unpacked[5] / (2**32)
        self.par_p4 = coeffs_unpacked[6] / (2**37)
        self.par_p5 = coeffs_unpacked[7] / (2**-3)
        self.par_p6 = coeffs_unpacked[8] / (2**6)
        self.par_p7 = coeffs_unpacked[9] / (2**8)
        self.par_p8 = coeffs_unpacked[10] / (2**15)
        self.par_p9 = coeffs_unpacked[11] / (2**48)
        self.par_p10 = coeffs_unpacked[12] / (2**48)
        self.par_p11 = coeffs_unpacked[13] / (2**65)

        self.barometric_pressure = barometric_pressure

        self.samplerate_hz = samplerate_hz

        self.altitudes: Dict[float, float] = {}
        self.speeds: Dict[float, float] = {}
        self.temperatures: Dict[float, float] = {}

        self.timestamp_increment = Decimal(1 / samplerate_hz)
        self.timestamp = peekable(self._timestamps())

    def _timestamps(self) -> Generator[float]:
        """Generator for timestamps

        This generator provides timestamps as parts of a second, derived from the sample rate specified when the class was instantiated.
        :return: The generator
        :rtype: Generator[float]
        """
        cur_timestamp = Decimal(0)
        while True:
            yield float(cur_timestamp.quantize(Decimal("0.0001")))
            cur_timestamp += self.timestamp_increment

    def store_reading(self, reading: bytes) -> None:
        """Decodes a single reading and stores it in memory

        This method decodes the single provided reading and updates the various globals (altitudes, speeds, and temperatures). Note that this method should be passed just the reading -- not an arbitrary frame (which may or may not contain data in addition to a header).

        :param reading bytes: A six-byte sensor reading. The first three bytes are the raw temperature reading, the second three bytes are the raw pressure reading.
        """
        raw_temp = reading[3] << 16 | reading[2] << 8 | reading[1]
        raw_pressure = reading[6] << 16 | reading[5] << 8 | reading[4]

        partial_data1 = raw_temp - self.par_t1
        partial_data2 = partial_data1 * self.par_t2
        temperature_c = partial_data2 + (partial_data1**2) * self.par_t3
        temperature_f = (temperature_c * 1.8) + 32

        partial_data1 = self.par_p6 * temperature_c
        partial_data2 = self.par_p7 * (temperature_c**2)
        partial_data3 = self.par_p8 * (temperature_c**3)
        partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3

        partial_data1 = self.par_p2 * temperature_c
        partial_data2 = self.par_p3 * (temperature_c**2)
        partial_data3 = self.par_p4 * (temperature_c**3)
        partial_out2 = raw_pressure * (
            self.par_p1 + partial_data1 + partial_data2 + partial_data3
        )

        partial_data1 = raw_pressure**2
        partial_data2 = self.par_p9 + self.par_p10 * temperature_c
        partial_data3 = partial_data1 * partial_data2
        partial_data4 = partial_data3 + (raw_pressure**3) * self.par_p11

        pressure_hpa = partial_out1 + partial_out2 + partial_data4
        alti_ft = (
            1 - ((float(pressure_hpa / 100) / self.barometric_pressure) ** 0.190284)
        ) * 145366.45

        if len(self.altitudes) > 0:
            prev_alti = list(self.altitudes.values())[-1]
            change = abs(alti_ft - prev_alti)
            miles_per_hour = change * self.samplerate_hz * 0.681818
        else:
            miles_per_hour = 0.0

        cur_idx = next(self.timestamp)
        self.altitudes[cur_idx] = alti_ft
        self.speeds[cur_idx] = miles_per_hour
        self.temperatures[cur_idx] = temperature_f

    @property
    def relative_altitudes(self) -> Dict[float, float]:
        """Returns a map from timestamps to altitudes relative to launch

        This returns a map derived from / structurally identical to the altitudes recorded by the sensor, but uses launch height as 0 instead of sea level.

        :return: A mappring from timestamps to height above liftoff
        :rtype: Dict[float, float]
        """
        return {k: v - self.altitudes[0.0] for k, v in self.altitudes.items()}
