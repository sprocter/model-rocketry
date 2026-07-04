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
(Modifications to 1 & 2 and new portions) Copyright (C) 2025-2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import UART
import time, gc, machine
import re

_GGA_DECODE = re.compile(
    r"\$GNGGA,(\d\d)(\d\d)(\d\d)\.(\d\d\d),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d),(\d+),[^,]+,([0-9.]+),"
)

_RMC_DECODE = re.compile(
    r"\$GNRMC,(\d\d)(\d\d)(\d\d)\.(\d\d\d),([AV]),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),([0-9.]+),([0-9.]+),(\d\d)(\d\d)(\d\d),"
)


class GPS:

    def __init__(self, uart_tx:int, uart_rx:int, use_pmtk_cmds:bool):
        self.tx = uart_tx
        self.rx = uart_rx
        self.buffer = b""
        self.init_done = False  # We switch NMEA sentences once we have a fix

        # Set placeholder values to avoid attr checks in speed-critical code
        self.altitude = 0.0
        self.lat = 0.0
        self.lon = 0.0

        # True if we should use PMTK commands, false if we use PCAS
        self.pmtk = use_pmtk_cmds

        self.uart = UART(1, baudrate=9600, tx=uart_tx, rx=uart_rx)

    def initialize(self) -> None:
        # Increase the baud rate to the maximum
        if(self.pmtk):
            self._send_command("PMTK251,115200")
        else:
            self._send_command("PCAS01,5")
        # Re initialize the UART to use the higher baud rate
        self.uart.init(baudrate=115200, tx=self.tx, rx=self.rx)
        # The UART seems to need a hot sec to reinitialize...
        # Get a new fix as fast as we can
        if(self.pmtk):
            self._send_command("PMTK220,100") # MT3333 can do 10Hz
        else:
            self._send_command("PCAS02,200") # L76K can only do 5Hz
        time.sleep_ms(100)
        # Get "recommended minimum" every update
        if(self.pmtk):
            self._send_command("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        else:
            self._send_command("PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0")
        time.sleep_ms(200)

    def read_raw(self):
        self.buffer = self.uart.readline()

    def decode_reading(self, reading: bytearray) -> None:
        if self.init_done and reading is not None:
            m = _GGA_DECODE.match(reading)
            if m is not None:
                self._set_data_from_gga(m)
        elif not self.init_done and reading is not None:
            m = _RMC_DECODE.match(reading)
            if m is not None:
                self._set_data_from_rmc(m)

    def clear_buffer(self) -> None:
        self.uart.read()

    def _send_command(self, command, add_checksum=True) -> None:
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

    def _set_data_from_rmc(self, m) -> None:
        if (m.group(5) != b"A"):
            return
        self.hour = int(m.group(1))
        self.minute = int(m.group(2))
        self.second = int(m.group(3))
        self.milli = int(m.group(4))
        
        self.day = int(m.group(12))
        self.month = int(m.group(13))
        self.year = 2000+int(m.group(14))
        # Switch to just getting "fix data" every update
        if(self.pmtk):
            self._send_command("PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        else:
            self._send_command("PCAS03,1,0,0,0,0,0,0,0,0,0,,,0,0")
        self.init_done = True

    def _set_data_from_gga(self, m) -> None:
        self.hour = int(m.group(1))
        self.minute = int(m.group(2))
        self.second = int(m.group(3))
        self.milli = int(m.group(4))
        self.lat = m.group(5)
        self.latNS = m.group(6)
        self.lon = m.group(7)
        self.lonEW = m.group(8)
        self.valid = (m.group(9) == b"1") or (m.group(9) == b"2")
        # # We don't use the satillite count for anything. 
        # self.satellites = int(m.group(10))
        self.altitude = float(m.group(11))
