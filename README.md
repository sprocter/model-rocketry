# model-rocketry
This is a repository for model rocketry projects. Currently, there is only one: an altimeter and accelerometer based on commonly-available parts.

## Altimeter and Accelerometer

If you've ever launched a model rocket, you've probably also wondered how high it went, how fast it traveled, and what sort of g-forces it experienced. There are some commercially-available altimeters and accelerometers, but I wanted to make my own. 

The [project website](https://samprocter.com/hobbies/model-rockets) has all the information you're probably looking for:

* [Hardware assembly, software installation, and usage instructions](https://samprocter.com/hobbies/model-rockets/accelerometer-altimeter-instructions/)
* [Software design considerations](https://samprocter.com/hobbies/model-rockets/accelerometer-altimeter-software/), in particular power-saving techniques / optimizations
* [Hardware selection considerations](https://samprocter.com/hobbies/model-rockets/accelerometer-altimeter-hardware/), as well as tradeoffs and ideas for improvement
* [Sample output](https://samprocter.com/hobbies/model-rockets/accelerometer-altimeter-data/), in the form of graphs generated from actual launches
* [Photos and discussion](https://samprocter.com/hobbies/model-rockets/accelerometer-altimeter-rockets/) of the types of rockets this has been tested on

### High-Level Instructions

If you don't want to read the [full instructions](https://samprocter.com/hobbies/model-rockets/accelerometer-altimeter-instructions/), the short version is:

1. Install micropython on an [ESP32s3](https://www.adafruit.com/product/5426).
2. Copy the files in the `pico` folder to that computer and connect it to a [BMP390](https://www.adafruit.com/product/4816) pressure sensor, [ICM20649](https://www.adafruit.com/product/4464) accelerometer, and A27 battery.
3. Put the device in a model rocket and launch it.
4. Connect the device to a USB power source (battery, computer, car, etc.) and hold the `boot` button until the blue LED stays lit.
5. Connect to the ESP32s3's wifi network and copy the folders (via FTP) in the `data` folder -- they should be numbered, i.e. `1`, `2`...
6. Copy the files in the `tera` folder to the directory that you copied the numbered folders to.
7. Install the dependencies: Python, and (via pip) `bokeh`, `more-itertools`, and `whittaker-eilers`
8. Run the `main-icm20649-bmp390.py` script.
9. HTML files with the recorded data will be generated in the current directory.