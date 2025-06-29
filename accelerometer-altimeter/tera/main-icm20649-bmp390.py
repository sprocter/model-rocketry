"""A script for processing model-rocketry sensor data into a human-readable form

This script processes raw sensor data (in binary format) from a commonly-available pressure sensor (the `Bosch BMP390`_) and accelerometer (the InvenSense ICM20649). These devices are small and light enough to go, along with a small battery and computer, onto a model rocket to record data during launches.

This code is designed to run on a full python installation (3.13+). Please see the sister script (located in `../pico`) for the code that should run on the smaller devices aboard the rocket itself. It requires the `Bokeh`_ and `Whittaker Smoother`_ libraries.

Full instructions for constructing your own altimeter / accelerometer and using this script are available at https://samprocter.com/hobbies/model-rockets/ accelerometer-altimeter-instructions/.

High-level usage instructions follow:

To use this script, place each launch's sensor readings in their own subdirectory; subdirectory names will be used as launch names. Place those subdirectories in a launches directory ("launches" here is an example -- the name is not used). Place the launches directory,  this script, and the device-specific drivers in a top-level directory. Then, change the final line of this script (the call to `main_loop(...)`) to the name of the launches directory, and run it. After running, there will be a html and json file for each launch in the launches directory. The html files are designed to be viewed by users, while the json files are designed to be archived and viewed later using `bokeh's embed feature`_.

Example:

Before invocation of this script:

│   main-icm20649-bmp390.py (this file)
│   bmp390.py
│   icm20649.py
|
├───launches
│   │
│   ├───C6-7
│   │       accel.bin
│   │       alti.bin
│   │
│   ├───D12-5
│   │       accel.bin
│   │       alti.bin
│   │
│   └───F15-8
│           accel.bin
│           alti.bin

After invocation of this script:

│   main-icm20649-bmp390.py (this file)
│   bmp390.py
│   icm20649.py
|
├───launches
│   │
│   │   C6-7.html
│   │   C6-7.json
│   │   D12-5.html
│   │   D12-5.json
│   │   F15-8.html
│   │   F15-8.json
│   │
│   ├───C6-7
│   │       accel.bin
│   │       alti.bin
│   │
│   ├───D12-5
│   │       accel.bin
│   │       alti.bin
│   │
│   └───F15-8
│           accel.bin
│           alti.bin

.. _Bosch BMP390: https://www.adafruit.com/product/4816
.. _InvenSense ICM20649: https://www.adafruit.com/product/4464
.. _Bokeh: https://docs.bokeh.org/en/3.7.3/docs/first_steps.html#first-steps-installing
.. _Whittaker Smoother: https://pypi.org/project/whittaker-eilers/
.. _bokeh's embed feature: https://docs.bokeh.org/en/3.7.3/docs/user_guide/output/embed.html#json-items
"""

from io import open
from json import dump
from os import chdir, scandir, listdir, DirEntry

# See https://towardsdatascience.com/the-perfect-way-to-smooth-your-noisy-data-4f3fe6b44440/
from whittaker_eilers import WhittakerSmoother

from bokeh.embed import json_item
from bokeh.io import curdoc
from bokeh.models import LinearAxis, Range1d
from bokeh.plotting import figure, output_file, save

from bmp390 import BMP390
from icm20649 import ICM20649

_CURR_BARO_PRESSURE = 1020
"""Barometric pressure at launch site and time.

This value has to be retrieved from your favorite weather app and written down at the time and location of the launch if you want accurate absolute altitudes.
"""

_RESOLUTION = 1
"""int (1-3): The 'resolution' of the sensor readings.

Higher values mean more sensor readings, more current draw, and more disk usage. Should be set to the same value as was used on the device when the readings were taken -- we use this information to accurately rebuild the sensor readings' timestamps.
"""

if _RESOLUTION == 1:
    # X in 1125/(1+X), which gives the accelerometer's sample rate in Hz
    _ACCEL_SAMPLERATE_NUM = 10
    # The altimeter's sample rate in Hz
    _ALTI_SAMPLERATE_HZ = 25
elif _RESOLUTION == 2:
    _ACCEL_SAMPLERATE_NUM = 2
    _ALTI_SAMPLERATE_HZ = 50
elif _RESOLUTION == 3:
    _ACCEL_SAMPLERATE_NUM = 1
    _ALTI_SAMPLERATE_HZ = 100
else:
    print("Resolution must be between 1 and 3.")
    exit()


def read_raw_data_from_files() -> tuple[bytes, bytes]:
    """Reads the sensor data from predictably named files.

    This reads every file in the current directory that begins with `accel` or `alti` and populates bytes objects with their contents. The formats of these files are specific to the accelerometer and altimeter devices being used.

    :return: A tuple containing accelerometer and altimeter data, in raw binary form
    :rtype: tuple[bytes, bytes]
    """

    accel_data = bytes()
    alti_data = bytes()
    for file in sorted(listdir()):
        if file.startswith("accel"):
            with open(file, "rb") as f:
                accel_data = f.read()
        if file.startswith("alti"):
            with open(file, "rb") as f:
                alti_data = f.read()
    return accel_data, alti_data


def decode_raw_data(
    accel_data: bytes, alti_data: bytes, accelerometer: ICM20649, altimeter: BMP390
) -> None:
    """Decodes, converts, and stores the raw sensor readings.

    This walks over the supplied raw sensor data and, using the supplied altimeter and accelerometer objects, converts it to human-readable, timestamped form. After this method completes, those driver objects will have the complete set of sensor readings in human-readable form.

    :param bytes accel_data: Raw accelerometer data in a format understandable to the accelerometer driver
    :param bytes alti_data: Raw altimeter data in a format understandable to the altimeter driver
    :param ICM20649 accelerometer: The accelerometer driver
    :param BMP390 altimeter: The altimeter driver
    """

    accel_idx = 0
    alti_idx = 0
    alti_idx_mod = 0

    while accel_idx < len(accel_data) // 6:
        alti_idx_start = alti_idx_mod + alti_idx * 7
        alti_idx_end = alti_idx_mod + (alti_idx + 1) * 7

        if accelerometer.timestamp.peek() < altimeter.timestamp.peek():
            accelerometer.store_reading(accel_data[accel_idx * 6 : (accel_idx + 1) * 6])
            accel_idx += 1
        else:
            # Check for Control frame or Empty frame
            if (alti_data[alti_idx_start] & 192 == 64) or (
                alti_data[alti_idx_start] & 255 == 128
            ):
                alti_idx_mod += 2
                continue
            altimeter.store_reading(alti_data[alti_idx_start:alti_idx_end])
            alti_idx += 1


def smooth(raw_data: list[float]) -> list[float]:
    """Smooths the supplied data

    Smooths the supplied data using a `Whittaker Smoother`_. Most settings are either reasonable defaults or, where automatically determinable, optimal.

    :param list[float] raw_data: A data set (typically sensor readings) that should be smoothed.
    :return: The smoothed version of the data.
    :rtype: list[float]

    .. _Whittaker Smoother: https://towardsdatascience.com/the-perfect-way-to-smooth-your-noisy-data-4f3fe6b44440/
    """
    smoother = WhittakerSmoother(lmbda=2e4, order=2, data_length=len(raw_data))
    smoothed = smoother.smooth_optimal(raw_data).get_optimal().get_smoothed()
    return smoothed


def write_bokeh_files(
    accelerometer: ICM20649, altimeter: BMP390, launch_name: str
) -> None:
    """Writes the data to html and json

    This writes the stored data to html and json formats using the `Bokeh`_ library. Files will be written to launch-NAME.html and launch-NAME.json where NAME is the value of the launch_name parameter. The html files are standalone, and intended to be distributed to viewed by end-users. The json files are intended to be archivable, and rendered using BokehJS; see the `bokeh documentation on this feature`_ for details.

    Accelerometer and altimeter data should be decoded and stored prior to invocation of this method; this should be done by calling `decode_raw_data(...)`.

    :param ICM20649 accelerometer: The accelerometer driver
    :param BMP390 altimeter: The altimeter driver
    :param str launch_name: The name of the launch

    .. _Bokeh: https://bokeh.org/
    .. _bokeh documentation on this feature: https://docs.bokeh.org/en/3.7.3/docs/user_guide/output/embed.html#json-items
    """

    # Smooth data and cache it and timestamps in local objects
    accels = smooth(list(accelerometer.accel.values()))
    accel_ts = list(accelerometer.accel.keys())
    temps = smooth(list(altimeter.temperatures.values()))
    altis = smooth(list(altimeter.relative_altitudes.values()))
    speeds = smooth(list(altimeter.speeds.values()))
    alti_ts = list(altimeter.altitudes.keys())

    # apply theme to current document
    curdoc().theme = "dark_minimal"

    output_file(
        filename="launch-" + launch_name + ".html",
        title="Launch " + launch_name + ": Acceleration and Altitude",
    )

    # create a new plot with a title and axis labels
    p = figure(
        title="Acceleration and Altitude, Launch " + launch_name,
        sizing_mode="stretch_both",
        height=1000,
        width=1000,
        y_range=(0, max(speeds)),
        x_axis_label="Time (seconds)",
        y_axis_label="Velocity (mph)",
        active_scroll="wheel_zoom",
    )

    p.extra_y_ranges["altitude"] = Range1d(min(altis), max(altis))  # type: ignore
    ax2 = LinearAxis(
        axis_label="Altitude (Feet Above Ground Level)",
        y_range_name="altitude",
    )
    ax2.axis_label_text_color = "red"
    p.add_layout(ax2, "left")

    p.extra_y_ranges["acceleration"] = Range1d(min(accels), max(accels))  # type: ignore
    ax3 = LinearAxis(
        axis_label="Acceleration (g)",
        y_range_name="acceleration",
    )
    ax3.axis_label_text_color = "lightgreen"
    p.add_layout(ax3, "right")

    p.extra_y_ranges["temperature"] = Range1d(min(temps), max(temps))  # type: ignore
    ax4 = LinearAxis(
        axis_label="Temperature (f)",
        y_range_name="temperature",
    )
    ax4.axis_label_text_color = "lightblue"
    p.add_layout(ax4, "right")

    p.line(accel_ts, accels, legend_label="Acceleration (g)", color="lightgreen", line_width=2)  # type: ignore
    p.line(alti_ts, speeds, legend_label="Velocity (mph)", color="white", line_width=2)  # type: ignore
    p.line(alti_ts, temps, legend_label="Temperature (f)", color="lightblue", line_width=2, y_range_name="temperature")  # type: ignore
    p.line(alti_ts, altis, legend_label="Altitude (ft)", color="red", line_width=2, y_range_name="altitude")  # type: ignore

    p.legend.click_policy = "hide"

    save(p)
    with open("launch-" + launch_name + ".json", "w") as json_file:
        dump(json_item(p, "accel-alti"), json_file)  # type: ignore


def main_loop(dir_name: str) -> None:
    """Processes all available data into html and json files

    This looks for subfolders in the provided directory, and processes sensor readings from each of those subfolders into human-readable and archivable formats. See the example in the module documentation for usage.

    :param str dir_name: The name of the directory containing subfolders with accelerometer and altimeter sensor readings.
    """
    chdir(dir_name)
    for launch in scandir():
        if DirEntry.is_file(launch):
            continue
        chdir(launch)
        accel_data, alti_data = read_raw_data_from_files()
        chdir("..")
        altimeter = BMP390(_CURR_BARO_PRESSURE, _ALTI_SAMPLERATE_HZ)
        accelerometer = ICM20649(_ACCEL_SAMPLERATE_NUM)
        decode_raw_data(accel_data, alti_data, accelerometer, altimeter)
        write_bokeh_files(accelerometer, altimeter, launch.name)


main_loop("data")
