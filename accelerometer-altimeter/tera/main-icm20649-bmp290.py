from io import open
from json import dump
from os import chdir, scandir, listdir, DirEntry
from struct import unpack
from decimal import Decimal

from more_itertools import peekable

from bokeh.embed import json_item
from bokeh.io import curdoc
from bokeh.models import LinearAxis, Range1d
from bokeh.plotting import figure, output_file, save

# To get these values, call `print_accel_calib_values` on the pico python file 
# when it's connected to the ICM 20649
_X_ERR = 0.029050755
_Y_ERR = -0.008103238
_Z_ERR = 0.044206185

# This value has to be retrieved from your favorite weather app and written 
# down at the time and location of the launch
_CURR_BARO_PRESSURE = 1023

# To get this string, call `print_packed_coeffs` on the pico python file when
# it's connected to the BMP290
_PACKED_COEFFS = "996e384df93a1a52140601684e9d6003fabd0f05f5" 

def accel_timestamps() -> float:
    cur_timestamp = Decimal(0)
    while True:
        yield float(cur_timestamp.quantize(Decimal('0.0001')))
        cur_timestamp += Decimal(1)/(Decimal(1125)/Decimal(11))

def alti_timestamps() -> float:
    cur_timestamp = Decimal(0)
    while True:
        yield float(cur_timestamp.quantize(Decimal('0.0001')))
        cur_timestamp += Decimal(.04)

def decode_accel_reading(accel_reading : bytes) -> tuple[float, float, float]:
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", accel_reading)
    accel_x = (raw_accel_x / 1024) - _X_ERR
    accel_y = (raw_accel_y / 1024) - _Y_ERR
    accel_z = (raw_accel_z / 1024) - _Z_ERR
    return (accel_x, accel_y, accel_z)

def decode_alti_reading(alti_reading : bytes) -> tuple[float, float]:
    coeffs_packed = bytes.fromhex(_PACKED_COEFFS)
    coeffs_unpacked = unpack("<HHbhhbbHHbbhbb", coeffs_packed)
    par_t1 = coeffs_unpacked[0] / (2 ** -8)
    par_t2 = coeffs_unpacked[1] / (2 ** 30)
    par_t3 = coeffs_unpacked[2] / (2 ** 48)

    par_p1 = (coeffs_unpacked[3] - (2 ** 14))/(2 ** 20)
    par_p2 = (coeffs_unpacked[4] - (2 ** 14))/(2 ** 29)
    par_p3 = coeffs_unpacked[5]/(2 ** 32)
    par_p4 = coeffs_unpacked[6]/(2 ** 37)
    par_p5 = coeffs_unpacked[7]/(2 ** -3)
    par_p6 = coeffs_unpacked[8]/(2 ** 6)
    par_p7 = coeffs_unpacked[9]/(2 ** 8)
    par_p8 = coeffs_unpacked[10]/(2 ** 15)
    par_p9 = coeffs_unpacked[11]/(2 ** 48)
    par_p10 = coeffs_unpacked[12]/(2 ** 48)
    par_p11 = coeffs_unpacked[13]/(2 ** 65)

    raw_temp = alti_reading[3] << 16 | alti_reading[2] << 8 | alti_reading[1]
    raw_pressure = alti_reading[6] << 16 | alti_reading[5] << 8 | alti_reading[4]

    partial_data1 = raw_temp - par_t1
    partial_data2 = partial_data1 * par_t2
    temperature_c = partial_data2 + (partial_data1 ** 2) * par_t3
    temperature_f = (temperature_c * 1.8) + 32

    partial_data1 = par_p6 * temperature_c
    partial_data2 = par_p7 * (temperature_c ** 2)
    partial_data3 = par_p8 * (temperature_c ** 3)
    partial_out1 = par_p5 + partial_data1 + partial_data2 + partial_data3

    partial_data1 = par_p2 * temperature_c
    partial_data2 = par_p3 * (temperature_c ** 2)
    partial_data3 = par_p4 * (temperature_c ** 3)
    partial_out2 = raw_pressure * (par_p1 + partial_data1 + partial_data2 + partial_data3)

    partial_data1 = raw_pressure ** 2
    partial_data2 = par_p9 + par_p10 * temperature_c
    partial_data3 = partial_data1 * partial_data2
    partial_data4 = partial_data3 + (raw_pressure ** 3) * par_p11

    pressure_hpa = partial_out1 + partial_out2 + partial_data4
    alti_ft = ((1-((float(pressure_hpa/100)/_CURR_BARO_PRESSURE) ** .190284)) * 145366.45)

    return (alti_ft, temperature_f)

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

def decode_raw_data(accel_data : bytes, alti_data : bytes) -> tuple[list, list, list, list, list, list, list]:
    accel_timestamp = peekable(accel_timestamps())
    alti_timestamp = peekable(alti_timestamps())
    
    accel_idx = 0
    alti_idx = 0
    alti_idx_mod = 0 

    xs = []
    ys = []
    zs = []
    accel_ts = []
    temps = []
    altis = []
    alti_ts = []

    while accel_idx < len(accel_data)//6:
        alti_idx_start = alti_idx_mod + alti_idx * 7
        alti_idx_end = alti_idx_mod + (alti_idx + 1) * 7
            
        if (accel_timestamp.peek() < alti_timestamp.peek()):
            accel_ts.append(next(accel_timestamp))
            [x.append(y) for x, y in zip([xs, ys, zs], decode_accel_reading(accel_data[accel_idx * 6 : (accel_idx + 1) * 6]))]
            accel_idx += 1
        else:
            # Check for Control frame or Empty frame
            if (alti_data[alti_idx_start] & 192 == 64) or (alti_data[alti_idx_start] & 255 == 128): 
                alti_idx_mod += 2
                continue
            alti_ts.append(next(alti_timestamp))
            [x.append(y) for x, y in zip([altis, temps], decode_alti_reading(alti_data[alti_idx_start : alti_idx_end]))]
            alti_idx += 1

    return xs, ys, zs, accel_ts, temps, altis, alti_ts

def write_bokeh_files(xs : list, ys : list, zs : list, accel_ts : list, temps : list, altis : list, alti_ts : list, launch_name : str) -> None:
    # apply theme to current document
    curdoc().theme = "dark_minimal"

    output_file(filename="launch-" + launch_name + ".html", title="Launch " + launch_name + ": Acceleration and Altitude")

    # create a new plot with a title and axis labels
    p = figure(
        title="Acceleration and Altitude, Launch " + launch_name,
        sizing_mode="stretch_both",
        height=1000,
        width=1000,
        y_range=(min(min(xs), min(ys), min(zs)),max(max(xs), max(ys), max(zs))),
        x_axis_label="Time (seconds)",
        y_axis_label="Acceleration (g)",
        active_scroll="wheel_zoom")
        
    p.extra_y_ranges['altitude'] = Range1d(min(altis), max(altis))
    ax2 = LinearAxis(
        axis_label="Altitude (ft)",
        y_range_name="altitude",
    )
    ax2.axis_label_text_color = "red"
    p.add_layout(ax2, 'left')

    p.extra_y_ranges['temperature'] = Range1d(min(temps), max(temps))
    ax3 = LinearAxis(
        axis_label="Temperature (f)",
        y_range_name="temperature",
    )
    ax3.axis_label_text_color = "lightblue"
    p.add_layout(ax3, 'right')

    # add a line renderer with legend and line thickness
    p.line(accel_ts, xs, legend_label="Acceleration (g): X", color="yellow", line_width=2)
    p.line(accel_ts, ys, legend_label="Acceleration (g): Y", color="white", line_width=2)
    p.line(accel_ts, zs, legend_label="Acceleration (g): Z", color="orange", line_width=2)
    p.line(alti_ts, temps, legend_label="Temperature (f)", color="lightblue", line_width=2, y_range_name="temperature")
    p.line(alti_ts, altis, legend_label="Altitude (ft)", color="red", line_width=2, y_range_name="altitude")

    save(p)
    with open("launch-" + launch_name + ".json", "w") as json_file:
        dump(json_item(p, "accel-alti"), json_file)

def main():
    chdir('data')
    for launch in scandir():
        if DirEntry.is_file(launch):
            continue
        chdir(launch)
        accel_data, alti_data = read_raw_data_from_files()
        chdir('..')
        xs, ys, zs, accel_ts, temps, altis, alti_ts = decode_raw_data(accel_data, alti_data)
        write_bokeh_files(xs, ys, zs, accel_ts, temps, altis, alti_ts, launch.name)

main()