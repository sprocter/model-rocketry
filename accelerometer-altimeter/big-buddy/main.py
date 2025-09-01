"""A script for a model rocket altimeter / accelerometer

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

from machine import I2C, Pin, Signal
from sh1107 import SH1107_I2C
from max17048 import MAX17048
from ulora import LoRa, ModemConfig, SPIConfig
import time, json

MODE_INITIAL = const(0)
MODE_BATTERY = const(1)
MODE_LORA = const(2)
MODE_C = const(3)


def listen_for_msgs(payload):
    print("From:", payload.header_from)
    print("Received:", payload.message)
    formatted_msg = "".join([chr(s) for s in payload.message])
    print("Formatted:", formatted_msg)
    print("RSSI: {}; SNR: {}".format(payload.rssi, payload.snr))
    show(formatted_msg, str(payload.rssi) + "dB")


def ignore_msgs(payload):
    pass


def show(line1: str, line2: str) -> None:
    display.fill(0)
    display.large_text(line1, 0, 0, 2)
    display.large_text(line2, 0, 32, 2)
    display.invert()
    display.show()
    time.sleep(1)
    display.invert()
    display.show()


def handleA(button_A):
    global last_time, display, charger, lora, mode
    # debounce
    if time.ticks_diff(time.ticks_ms(), last_time) < 400:
        return
    last_time = time.ticks_ms()
    print("Button A pressed")
    lora.on_recv = ignore_msgs
    lora.set_mode_idle()
    mode = MODE_BATTERY
    charge = charger.charge_percent
    rate = charger.charge_rate
    show(f"{charge:.1f}%", f"{rate:.1f}%/hr")


def handleB(button_B):
    global last_time, lora, mode
    # debounce
    if time.ticks_diff(time.ticks_ms(), last_time) < 400:
        return
    last_time = time.ticks_ms()
    print("Button B pressed")
    lora.on_recv = listen_for_msgs
    lora.set_mode_rx()
    mode = MODE_LORA


def handleC(button_C):
    global last_time, lora, mode
    # debounce
    if time.ticks_diff(time.ticks_ms(), last_time) < 400:
        return
    last_time = time.ticks_ms()
    lora.on_recv = ignore_msgs
    lora.set_mode_idle()
    mode = MODE_C
    show("Button", "C")


with open("/secrets.json", "r") as f:
    secrets = json.loads(f.read())

last_time = time.ticks_ms()
mode = MODE_INITIAL

button_A = Pin(9, Pin.IN, Pin.PULL_UP)
button_B = Pin(6, Pin.IN, Pin.PULL_UP)
button_C = Pin(5, Pin.IN, Pin.PULL_UP)

i2c = I2C(scl=4, sda=3)

display = SH1107_I2C(128, 64, i2c, address=0x3C, rotate=180)
display.contrast(0xFF)
charger = MAX17048(i2c)

lora = LoRa(
    spi_channel=SPIConfig.esp32s3_1,
    interrupt=10,
    this_address=secrets["bigbuddy-addr"],
    cs_pin=11,
    reset_pin=8,
    freq=915.0,
    tx_power=5,
    modem_config=ModemConfig.USLegalLongRange,
    receive_all=False,
    acks=False,
    crypto=None,
)

button_A.irq(trigger=Pin.IRQ_FALLING, handler=handleA)
button_B.irq(trigger=Pin.IRQ_FALLING, handler=handleB)
button_C.irq(trigger=Pin.IRQ_FALLING, handler=handleC)

while True:
    if mode == MODE_INITIAL:
        show("Await", "Input")
        time.sleep(9)
    elif mode == MODE_BATTERY:
        charge = charger.charge_percent
        rate = charger.charge_rate
        show(f"{charge:.1f}%", f"{rate:.1f}%/hr")
        time.sleep(9)
    elif mode == MODE_LORA:
        time.sleep(9)
    elif mode == MODE_C:
        show("Button", "C")
        time.sleep(9)
