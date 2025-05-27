from decimal import Decimal
from typing import Generator, Dict
from struct import unpack

from more_itertools import peekable

class BMP390:

    def __init__(
            self, 
            packed_coeffs : str, 
            barometric_pressure : int,
            samplerate_hz : int
        ):
        coeffs_packed = bytes.fromhex(packed_coeffs)
        coeffs_unpacked = unpack("<HHbhhbbHHbbhbb", coeffs_packed)
        self.par_t1 = coeffs_unpacked[0] / (2 ** -8)
        self.par_t2 = coeffs_unpacked[1] / (2 ** 30)
        self.par_t3 = coeffs_unpacked[2] / (2 ** 48)

        self.par_p1 = (coeffs_unpacked[3] - (2 ** 14))/(2 ** 20)
        self.par_p2 = (coeffs_unpacked[4] - (2 ** 14))/(2 ** 29)
        self.par_p3 = coeffs_unpacked[5]/(2 ** 32)
        self.par_p4 = coeffs_unpacked[6]/(2 ** 37)
        self.par_p5 = coeffs_unpacked[7]/(2 ** -3)
        self.par_p6 = coeffs_unpacked[8]/(2 ** 6)
        self.par_p7 = coeffs_unpacked[9]/(2 ** 8)
        self.par_p8 = coeffs_unpacked[10]/(2 ** 15)
        self.par_p9 = coeffs_unpacked[11]/(2 ** 48)
        self.par_p10 = coeffs_unpacked[12]/(2 ** 48)
        self.par_p11 = coeffs_unpacked[13]/(2 ** 65)

        self.barometric_pressure = barometric_pressure

        self.samplerate_hz = samplerate_hz

        self.altitudes : Dict[float, float] = {}
        self.speeds : Dict[float, float] = {}
        self.temperatures : Dict[float, float] = {}

        self.timestamp = peekable(self._timestamps())


    def _timestamps(self) -> Generator[float]:
        cur_timestamp = Decimal(0)
        while True:
            yield float(cur_timestamp.quantize(Decimal('0.0001')))
            cur_timestamp += Decimal(1 / self.samplerate_hz)

    def store_reading(self, reading : bytes) -> None:
        raw_temp = reading[3] << 16 | reading[2] << 8 | reading[1]
        raw_pressure = reading[6] << 16 | reading[5] << 8 | reading[4]

        partial_data1 = raw_temp - self.par_t1
        partial_data2 = partial_data1 * self.par_t2
        temperature_c = partial_data2 + (partial_data1 ** 2) * self.par_t3
        temperature_f = (temperature_c * 1.8) + 32

        partial_data1 = self.par_p6 * temperature_c
        partial_data2 = self.par_p7 * (temperature_c ** 2)
        partial_data3 = self.par_p8 * (temperature_c ** 3)
        partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3

        partial_data1 = self.par_p2 * temperature_c
        partial_data2 = self.par_p3 * (temperature_c ** 2)
        partial_data3 = self.par_p4 * (temperature_c ** 3)
        partial_out2 = raw_pressure * (self.par_p1 + partial_data1 + partial_data2 + partial_data3)

        partial_data1 = raw_pressure ** 2
        partial_data2 = self.par_p9 + self.par_p10 * temperature_c
        partial_data3 = partial_data1 * partial_data2
        partial_data4 = partial_data3 + (raw_pressure ** 3) * self.par_p11

        pressure_hpa = partial_out1 + partial_out2 + partial_data4
        alti_ft = ((1-((float(pressure_hpa/100)/self.barometric_pressure) ** .190284)) * 145366.45)

        if len(self.altitudes) > 0:
            prev_alti = list(self.altitudes.values())[-1]
            change = abs(alti_ft - prev_alti)
            miles_per_hour = change * self.samplerate_hz * .681818
        else :
            miles_per_hour = 0.0

        cur_idx = next(self.timestamp)
        self.altitudes[cur_idx] = alti_ft
        self.speeds[cur_idx] = miles_per_hour
        self.temperatures[cur_idx] = temperature_f

    @property
    def relative_altitudes(self):
        return {k: v - self.altitudes[0.0] for k, v in self.altitudes.items()}