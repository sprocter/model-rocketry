"""
A quick, minimal driver for the PA1010D GPS Module

A few modifications by Sam Procter, 2025: The original (1) used I2C, but it reads one character at a time so was too slow for what I needed. It also used functionality in Micropython's regular expressions module that is not available on the ESP32S3, so I rewrote those portions. I adapted / used the send_command function from (2), which is for CircuitPython rather than MicroPython.

Adapted from
1. GPS Driver, (c) 2022 by Mike Bell licensed under MIT
   Original code: https://github.com/MichaelBell/pico-uPython/
2. Adafruit GPS, (c) 2017 Tony DiCola for Adafruit Industries and 2021
   James Carr licensed under MIT
   Original code: https://github.com/adafruit/Adafruit_CircuitPython_GPS

--------------------------------------------------------------------------------
(Modifications to 1 & 2 and new portions) Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""


from machine import UART
import time, gc, machine
import re

GGA_DECODE = re.compile(
    r"\$GNGGA,(\d\d)(\d\d)(\d\d)\.(\d\d\d),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d),(\d+),[^,]+,([0-9.]+),"
)
RMC_DECODE = re.compile(
    r"\$GNRMC,(\d\d)(\d\d)(\d\d)\.(\d\d\d),([AV]),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),([0-9.]+),([0-9.]+),(\d\d)(\d\d)(\d\d),"
)

class PA1010:

    def __init__(self, uart):
        self.uart = uart

    def initialize(self):
        # Increase the baud rate to the maximum
        self.send_command("PMTK251,115200")
        # Re initialize the UART to use the higher baud rate
        self.uart.init(baudrate=115200, tx=6, rx=7)
        # Get a "recommended minimum" every update, and "fix data" every fifth update
        self.send_command("PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Get a new fix every 100 ms
        self.send_command("PMTK220,100")
        # Clear the buffer
        self.uart.read()

    def send_command(self, command, add_checksum=True):
        """Send a command string
        If add_checksum is True (the default) a NMEA checksum will automatically be computed and added.
        """
        if type(command) is not bytes:
            command = command.encode("ascii")

        buf = bytearray()
        buf += b"$"
        buf += command
        if add_checksum:
            checksum = 0
            # `for char in command` iterates through char ordinals
            for char in command:
                checksum ^= char
            buf += b"*"  # Delimits checksum value
            buf += bytes(f"{checksum:02x}".upper(), "ascii")
        buf += b"\r\n"
        print(f'Writing "{buf}"')
        self.uart.write(buf)

    def _decode_sentence(self, buf):
        m = GGA_DECODE.match(buf)
        if m is not None:
            # print("GGA parse ok!")
            self._set_data_from_gga(m)
        else:
            m = RMC_DECODE.match(buf)
            if m is not None:
                # print("RMC parse ok!")
                self._set_data_from_rmc(m)

    def _set_data_from_rmc(self, m):
        # Modifications by Sam Procter, 2025:
        # Micropython for ESP32S3 (and others, I believe) does not support
        # regex's match.groups() functionality, so I commented it out and
        # replaced it with hardcoded match.group() calls.

        # self.hour, self.minute, self.second, self.milli = [
        #     int(x) for x in m.groups()[:4]
        # ]
        self.hour = int(m.group(1))
        self.minute = int(m.group(2))
        self.second = int(m.group(3))
        self.milli = int(m.group(4))
        self.valid = m.group(5) == "A"
        # self.lat, self.latNS, self.lon, self.lonEW = m.groups()[5:9]
        self.lat = m.group(6)
        self.latNS = m.group(7)
        self.lon = m.group(8)
        self.lonEW = m.group(9)
        self.speed = float(m.group(10))
        self.heading = float(m.group(11))
        # self.day, self.month, self.year = [int(x) for x in m.groups()[11:14]]
        self.day = int(m.group(12))
        self.month = int(m.group(13))
        self.year = int(m.group(14))
        self.year += 2000

    def _set_data_from_gga(self, m):
        # self.hour, self.minute, self.seconds, self.millis = [int(x) for x in m.groups()[:4]]
        # self.lat, self.latNS, self.lon, self.lonEW = m.groups()[4:8]
        self.satellites = int(m.group(10))
        self.altitude = float(m.group(11))

gps = PA1010(UART(1, baudrate=9600, tx=6, rx=7))
gps.initialize()

# sentences = [ # Sentences copied from datasheet
#     '$GNRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W\n,A*2C',
#     '$GNRMC,155503.000,A,5606.1725,N,01404.0622,E,0.04,0.00,110918,,,D*75\n','$GNGGA,165006.000,2241.9107,N,12017.2383,E,1,14,0.79,22.6,M,18.5,M,,*42\n',
#     '$GNRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W\n,A*2C',
#     '$GNRMC,155503.000,A,5606.1725,N,01404.0622,E,0.04,0.00,110918,,,D*75\n','$GNGGA,165006.000,2241.9107,N,12017.2383,E,1,14,0.79,22.6,M,18.5,M,,*42\n',
#     '$GNRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W\n,A*2C',
#     '$GNRMC,155503.000,A,5606.1725,N,01404.0622,E,0.04,0.00,110918,,,D*75\n','$GNGGA,165006.000,2241.9107,N,12017.2383,E,1,14,0.79,22.6,M,18.5,M,,*42\n','$GNRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W\n,A*2C',
#     '$GNRMC,155503.000,A,5606.1725,N,01404.0622,E,0.04,0.00,110918,,,D*75\n',
#     '$GNGGA,165006.000,2241.9107,N,12017.2383,E,1,14,0.79,22.6,M,18.5,M,,*42\n']

# for i in range(len(sentences)):
for i in range(20):
    time.sleep_ms(100)
    start_ts = time.ticks_us()
    uart_out = gps.uart.readline()
    uart_ts = time.ticks_us()
    gps._decode_sentence(uart_out.decode('ascii'))
    # gps._decode_sentence(sentences[i])
    parsed_ts = time.ticks_us()
    print(f"UART read time: {time.ticks_diff(uart_ts, start_ts)}")
    print(f"Parse time (fastgps): {time.ticks_diff(parsed_ts, uart_ts)}")
    gc.collect()

gps.uart.deinit()