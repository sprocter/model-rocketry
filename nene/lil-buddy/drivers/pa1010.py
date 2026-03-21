"""
A quick, minimal driver for the PA1010D GPS Module

A few modifications by Sam Procter, 2025: The original (1) used I2C, but it reads one character at a time so was too slow for what I needed. It also used functionality in Micropython's regular expressions module that is not available on the ESP32S3, so I rewrote those portions. I also only need data from GGA messages, so I removed the RMC functionality. I adapted / used the send_command function from (2), which is for CircuitPython rather than MicroPython.

Adapted from
1. GPS Driver, (c) 2022 by Mike Bell licensed under MIT
   Original code: https://github.com/MichaelBell/pico-uPython/
2. Adafruit GPS, (c) 2017 Tony DiCola for Adafruit Industries and 2021
   James Carr licensed under MIT
   Original code: https://github.com/adafruit/Adafruit_CircuitPython_GPS

--------------------------------------------------------------------------------
(Modifications to 1 & 2 and new portions) Copyright (C) 2025-2026 Sam Procter

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

class PA1010:

    def __init__(self, uart_tx, uart_rx):
        self.tx = uart_tx
        self.rx = uart_rx
        self.buffer = b""

        # Set placeholder values to avoid attr checks in speed-critical code
        self.altitude = 0.0
        self.lat = 0.0
        self.lon = 0.0

        self.uart = UART(1, baudrate=9600, tx=uart_tx, rx=uart_rx)

    def initialize(self):
        # Get "fix data" every update
        self._send_command("PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Increase the baud rate to the maximum
        self._send_command("PMTK251,115200")
        # Re initialize the UART to use the higher baud rate
        self.uart.init(baudrate=115200, tx=self.tx, rx=self.rx)
        # The UART seems to need a hot sec to reinitialize...
        time.sleep_ms(100)
        # Get a new fix every 100 ms
        self._send_command("PMTK220,100")

    def read_raw(self):
        self.buffer = self.uart.readline()

    def decode_reading(self, reading: bytearray) -> None:
        if reading is not None:
            m = GGA_DECODE.match(reading)
            if m is not None:
                self._set_data_from_gga(m)

    def clear_buffer(self) -> None:
        self.uart.read()

    def _send_command(self, command, add_checksum=True):
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
        self.uart.write(buf)
        # Wait for the writes to complete
        self.uart.flush()

    def _set_data_from_gga(self, m):
        self.hour = int(m.group(1))
        self.minute = int(m.group(2))
        self.second = int(m.group(3))
        self.milli = int(m.group(4))
        self.lat = m.group(5)
        self.latNS = m.group(6)
        self.lon = m.group(7)
        self.lonEW = m.group(8)
        self.valid = (m.group(9) == b"1") or (m.group(9) == b"2") 
        self.satellites = int(m.group(10))
        self.altitude = float(m.group(11))
