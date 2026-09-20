"""A script for a model rocket locator

--------------------------------------------------------------------------------
Copyright (C) 2025-2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C, SPI, Pin, Signal
from sh1107 import SH1107_I2C
from max17048 import MAX17048
from sx127x import SX1276
from struct import unpack
from binascii import unhexlify
from micropython import schedule
import time, json, cryptolib

MODE_INITIAL = const(0)
MODE_BATTERY = const(1)
MODE_LORA = const(2)
MODE_C = const(3)


def listen_for_msgs(payload):
    schedule(display_msg, payload)


def display_msg(message, rssi):
    lat_str = "  -- --.----"
    lon_str = " --- --.----"
    alt_str = "----.---"
    rssi_str = "---.--"

    while True:
        if message is not None:
            rssi_str = str(rssi)

            msg_elems = unpack(">fHHBHHB", message)
            print(msg_elems)
            alt_str = f"{msg_elems[0]:.3f}"
            lat_str = (
                chr(msg_elems[3])
                + " "
                + f"{msg_elems[1]:04}"
                + "."
                + f"{msg_elems[2]:04}"
            )
            lat_str = (
                lat_str[: lat_str.index(".") - 2]
                + " "
                + lat_str[lat_str.index(".") - 2 :]
            )
            lon_str = (
                chr(msg_elems[6]) + f"{msg_elems[4]:05}" + "." + f"{msg_elems[5]:04}"
            )
            lon_str = (
                lon_str[: lon_str.index(".") - 2]
                + " "
                + lon_str[lon_str.index(".") - 2 :]
            )

        line1 = f"Lat:{lat_str}"
        line2 = f"Lon:{lon_str}"
        line3 = f"Alt:{alt_str:>11}m"
        line4 = f"RSSI:{rssi_str:>8}dBm"
        display_small_text(line1, line2, line3, line4)
        received = modem.recv()  # Hang until we get a message
        decrypted = decryptor.decrypt(received)
        message = decrypted[:14]  # Last two bytes are random padding, discard
        rssi = received.rssi


def ignore_msgs(payload):
    pass


def display_small_text(line1: str, line2: str, line3: str, line4: str) -> None:
    display.fill(0)
    display.text(line1, 0, 0)
    display.text(line2, 0, 16)
    display.text(line3, 0, 32)
    display.text(line4, 0, 48)
    display.invert()
    display.show()
    time.sleep(1)
    display.invert()
    display.show()


def display_big_text(line1: str, line2: str) -> None:
    display.fill(0)
    display.large_text(line1, 0, 0, 2)
    display.large_text(line2, 0, 32, 2)
    display.invert()
    display.show()
    time.sleep(1)
    display.invert()
    display.show()


def handleA(button_A):
    global last_time, display, charger, mode
    # debounce
    if time.ticks_diff(time.ticks_ms(), last_time) < 400:
        return
    last_time = time.ticks_ms()
    print("Button A pressed")
    mode = MODE_BATTERY
    voltage = charger.voltage
    charge = charger.charge_percent
    rate = charger.charge_rate
    line1 = f"Voltage: {voltage:.2f}v"
    line2 = f"SoC:    {charge:.2f}%"
    line3 = f"Rate:   {rate:.1f}%/hr"
    line4 = f" "
    display_small_text(line1, line2, line3, line4)


def handleB(button_B):
    global last_time, mode
    # debounce
    if time.ticks_diff(time.ticks_ms(), last_time) < 400:
        return
    last_time = time.ticks_ms()
    mode == MODE_LORA
    display_msg(None, None)  # Last two bytes are random padding, discard


def handleC(button_C):
    global last_time, mode
    # debounce
    if time.ticks_diff(time.ticks_ms(), last_time) < 400:
        return
    last_time = time.ticks_ms()
    mode = MODE_C
    display_big_text("Button", "C")


with open("/config.json", "r") as f:
    config = json.loads(f.read())

last_time = time.ticks_ms()
mode = MODE_INITIAL

button_A = Pin(9, Pin.IN, Pin.PULL_UP)
button_B = Pin(6, Pin.IN, Pin.PULL_UP)
button_C = Pin(5, Pin.IN, Pin.PULL_UP)

i2c = I2C(scl=4, sda=3)

display = SH1107_I2C(128, 64, i2c, address=0x3C, rotate=180)
display.contrast(0xFF)
charger = MAX17048(i2c)

lora_cfg = {
    "freq_khz": int(float(config["lora"]["freq"]) * 1000),
    "sf": 12,
    "bw": 500,  # kHz
    "coding_rate": 8,
    "preamble_len": 12,
    "output_power": 22,  # We don't transmit, so this can be whatever
    "crc_en": False,
    "rx_boost": True,
    "lna_boost_hf": True,
}

modem = SX1276(
    spi=SPI(
        1,
        baudrate=2_000_000,
        polarity=0,
        phase=0,
        miso=Pin(37),
        mosi=Pin(35),
        sck=Pin(36),
    ),
    cs=Pin(11),
    dio0=Pin(10),
    reset=Pin(8),
    lora_cfg=lora_cfg,
)

decryptor = cryptolib.aes(unhexlify(config["lora"]["key"]), 1)

button_A.irq(trigger=Pin.IRQ_FALLING, handler=handleA)
button_B.irq(trigger=Pin.IRQ_FALLING, handler=handleB)
button_C.irq(trigger=Pin.IRQ_FALLING, handler=handleC)

while True:
    if mode == MODE_INITIAL:
        display_big_text("Await", "Input")
        time.sleep(9)
    elif mode == MODE_BATTERY:
        voltage = charger.voltage
        charge = charger.charge_percent
        rate = charger.charge_rate
        line1 = f"Voltage: {voltage:.2f}v"
        line2 = f"SoC:    {charge:.2f}%"
        line3 = f"Rate:   {rate:.1f}%/hr"
        line4 = f" "
        display_small_text(line1, line2, line3, line4)
        time.sleep(9)
    elif mode == MODE_LORA:
        time.sleep(9)
    elif mode == MODE_C:
        display_big_text("Button", "C")
        time.sleep(9)
