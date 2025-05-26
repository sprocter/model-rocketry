from io import open
from json import dump
from os import chdir, scandir, listdir, DirEntry
from struct import unpack
from decimal import Decimal
from itertools import pairwise
from math import sqrt
from typing import Generator

from more_itertools import peekable
from whittaker_eilers import WhittakerSmoother # See https://towardsdatascience.com/the-perfect-way-to-smooth-your-noisy-data-4f3fe6b44440/

from bokeh.embed import json_item
from bokeh.io import curdoc
from bokeh.models import LinearAxis, Range1d
from bokeh.plotting import figure, output_file, save

from bmp390 import BMP390

# To get these values, call `print_accel_calib_values` on the pico python file 
# when it's connected to the ICM 20649
_X_ERR = 0.029050755
_Y_ERR = -0.008103238
_Z_ERR = 0.044206185

# This value has to be retrieved from your favorite weather app and written 
# down at the time and location of the launch
_CURR_BARO_PRESSURE = 1020

# To get this string, call `print_packed_coeffs` on the pico python file when
# it's connected to the BMP290
_PACKED_COEFFS = "996e384df93a1a52140601684e9d6003fabd0f05f5" 

# X in 1125/(1+X), which gives the accelerometer's sample rate in Hz
_ACCEL_SAMPLERATE_NUM = 10

# The altimeter's sample rate in Hz
_ALTI_SAMPLERATE_HZ = 25


def accel_timestamps() -> Generator[float]:
    cur_timestamp = Decimal(0)
    while True:
        yield float(cur_timestamp.quantize(Decimal('0.0001')))
        cur_timestamp += Decimal(1)/(Decimal(1125)/Decimal(1 + _ACCEL_SAMPLERATE_NUM))

def decode_accel_reading(accel_reading : bytes) -> tuple[float, float, float]:
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", accel_reading)
    accel_x = (raw_accel_x / 1024) - _X_ERR
    accel_y = (raw_accel_y / 1024) - _Y_ERR
    accel_z = (raw_accel_z / 1024) - _Z_ERR
    return (accel_x, accel_y, accel_z)

def read_raw_data_from_files() -> tuple[bytes, bytes]:
    accel_data = bytes()
    alti_data = bytes()
    for file in sorted(listdir()): 
        if file.startswith('accel'):
            with open(file, 'rb') as f:
                accel_data = f.read()
        if file.startswith('alti'):
            with open(file, 'rb') as f:
                alti_data = f.read()
    return accel_data, alti_data

def decode_raw_data(accel_data : bytes, alti_data : bytes, altimeter : BMP390) -> tuple[list, list, list, list]:
    accel_timestamp = peekable(accel_timestamps())
    
    accel_idx = 0
    alti_idx = 0
    alti_idx_mod = 0 

    xs = []
    ys = []
    zs = []
    accel_ts = []

    while accel_idx < len(accel_data)//6:
        alti_idx_start = alti_idx_mod + alti_idx * 7
        alti_idx_end = alti_idx_mod + (alti_idx + 1) * 7
            
        if (accel_timestamp.peek() < altimeter.timestamp.peek()):
            accel_ts.append(next(accel_timestamp))
            [x.append(y) for x, y in zip([xs, ys, zs], decode_accel_reading(accel_data[accel_idx * 6 : (accel_idx + 1) * 6]))]
            accel_idx += 1
        else:
            # Check for Control frame or Empty frame
            if (alti_data[alti_idx_start] & 192 == 64) or (alti_data[alti_idx_start] & 255 == 128): 
                alti_idx_mod += 2
                continue
            altimeter.store_reading(alti_data[alti_idx_start : alti_idx_end])
            alti_idx += 1

    return xs, ys, zs, accel_ts

def get_total_accel(xs : list[float], ys : list[float], zs : list[float]):
    accel = []
    for a in zip(xs, ys, zs):
        accel.append(sqrt(a[0]**2 + a[1]**2 +a[2]**2))
    return accel

def smooth_data(accels:list[float]):
    accel_smoother = WhittakerSmoother(lmbda=2e4, order=2, data_length=len(accels))
    smoothed_accels = accel_smoother.smooth_optimal(accels).get_optimal().get_smoothed() 
    return smoothed_accels

def smooth(raw_data : list[float]):
    smoother = WhittakerSmoother(lmbda=2e4, order=2, data_length=len(raw_data))
    smoothed = smoother.smooth_optimal(raw_data).get_optimal().get_smoothed()
    return smoothed

def write_bokeh_files(temps : list, altis : list, alti_ts : list, speeds : list, accel_ts : list, accels : list, launch_name : str) -> None:
    # apply theme to current document
    curdoc().theme = "dark_minimal"

    output_file(filename="launch-" + launch_name + ".html", title="Launch " + launch_name + ": Acceleration and Altitude")

    # create a new plot with a title and axis labels
    p = figure(
        title="Acceleration and Altitude, Launch " + launch_name,
        sizing_mode="stretch_both",
        height=1000,
        width=1000,
        y_range=(0, max(speeds)),
        x_axis_label="Time (seconds)",
        y_axis_label="Velocity (mph)",
        active_scroll="wheel_zoom")
        
    p.extra_y_ranges['altitude'] = Range1d(min(altis), max(altis)) # type: ignore
    ax2 = LinearAxis(
        axis_label="Altitude (Feet Above Ground Level)",
        y_range_name="altitude",
    )
    ax2.axis_label_text_color = "red"
    p.add_layout(ax2, 'left')

    p.extra_y_ranges['acceleration'] = Range1d(min(accels), max(accels)) # type: ignore
    ax3 = LinearAxis(
        axis_label="Acceleration (g)",
        y_range_name="acceleration",
    )
    ax3.axis_label_text_color = "lightgreen"
    p.add_layout(ax3, 'right')


    p.extra_y_ranges['temperature'] = Range1d(min(temps), max(temps)) # type: ignore
    ax4 = LinearAxis(
        axis_label="Temperature (f)",
        y_range_name="temperature",
    )
    ax4.axis_label_text_color = "lightblue"
    p.add_layout(ax4, 'right')

    p.line(accel_ts, accels, legend_label="Acceleration (g)", color="lightgreen", line_width=2) # type: ignore
    p.line(alti_ts, speeds, legend_label="Velocity (mph)", color="white", line_width=2) # type: ignore
    p.line(alti_ts, temps, legend_label="Temperature (f)", color="lightblue", line_width=2, y_range_name="temperature") # type: ignore
    p.line(alti_ts, altis, legend_label="Altitude (ft)", color="red", line_width=2, y_range_name="altitude") # type: ignore

    p.legend.click_policy="hide"

    save(p)
    with open("launch-" + launch_name + ".json", "w") as json_file:
        dump(json_item(p, "accel-alti"), json_file) # type: ignore

def main():
    chdir('data')
    altimeter = BMP390(_PACKED_COEFFS, _CURR_BARO_PRESSURE, _ALTI_SAMPLERATE_HZ)
    for launch in scandir():
        if DirEntry.is_file(launch):
            continue
        chdir(launch)
        accel_data, alti_data = read_raw_data_from_files()
        chdir('..')
        xs, ys, zs, accel_ts = decode_raw_data(accel_data, alti_data, altimeter)
        # if len(altis) < 5:
        #     continue
        accels = get_total_accel(xs, ys, zs)

        smoothed_accels = smooth_data(accels)

        write_bokeh_files(
            smooth(list(altimeter.temperatures.values())), 
            smooth(list(altimeter.altitudes.values())), 
            list(altimeter.altitudes.keys()), # timestamps
            smooth(list(altimeter.speeds.values())), 
            accel_ts, 
            smoothed_accels, 
            launch.name)

main()