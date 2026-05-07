
# Development Environment Setup

## Prerequisites

This guide assumes that the following repositories are cloned into `~/git/`. If your setup differs, you may have to modify some paths.

1. [Micropython](https://github.com/micropython/micropython)
2. [ESP-IDF](https://github.com/espressif/esp-idf.git)
    * Note: Be sure to clone a version supported by Micropython, e.g., `git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git`
3. [ulab](https://github.com/v923z/micropython-ulab)
4. [This repository](https://github.com/sprocter/model-rocketry/)

You'll also need to either download a few files from some other repositories (listed below).

## Copy / Modify Build Files

1. Python modules, copy into `micropython/ports/esp32/modules`
    1. Third-party libraries
        1. [micropySX126X](https://github.com/ehong-tl/micropySX126X): `_sx126x.py`, `sx126x.py`, and `sx1262.py`
        2. [micropython-fusion](https://github.com/micropython-IMU/micropython-fusion): `deltat.py`, `fusion.py`, and `orientate.py`
        3. [FTP-Server-for-ESP8266-ESP32-and-PYBD](https://github.com/robert-hh/FTP-Server-for-ESP8266-ESP32-and-PYBD/tree/master): `uftpd.py`
            * Delete the last line: we don't want the FTP server to auto-start upon import.
    2. All the files in the `model-rocketry/nene/lil-buddy/drivers` directory
2. Build files, copy from `model-rocketry/nene/lil-buddy/micropython-tweaks` into `micropython/ports/esp32`
    1. FeatherS3D-makefile
    2. XiaoEsp32s3Plus-makefile
    3. partitions_nene.csv
    4. nene.cmake
3. Nene-specific port customizations
    1. From the `micropython/ports/esp32/boards` directory, copy the generic board to a new one called NENE, e.g. `cp -r ESP32_GENERIC_S3/ NENE`
    2. In the newly-created `micropython/ports/esp32/boards/NENE` directory, replace the following two files with the versions from `model-rocketry/nene/lil-buddy/micropython-tweaks`:
        1. sdkconfig.board
        2. mpconfigboard.h
4. Tweak ulab for speed and space usage by modifying `ulab/code/ulab.h`
    1. Disable complex number support
        * Change line 36 to `#define ULAB_SUPPORTS_COMPLEX               (0)`
    2. Disable SciPy
        * Change line 42 to `#define ULAB_HAS_SCIPY                      (0)`
    3. Disable function pointers in iterations
        * Change line 296 to `#define ULAB_VECTORISE_USES_FUN_POINTER (0)`

## Building

Using the build instructions in `micropython/ports/esp32/README.md` 

1. Follow the steps in the section titled "Setting up ESP-IDF and the build environment"
2. Follow the first step in the section titled "Building the firmware" to build the cross-compiler and then change to the `ports/esp32` directory. 
3. Instead of running `make` as the standard instructions suggest, instead run `make --makefile=FeatherS3D-makefile` (or, for the Xiao Esp32S3+, `make --makefile=XiaoEsp32s3Plus-makefile`):
    1. `make --makefile=FeatherS3D-makefile submodules`
    2. `make --makefile=FeatherS3D-makefile`

## Deploying

This section assumes you're using Linux and the device is at `/dev/ttyACM0` -- if you're using Windows, or using a different port, you'll need to change the ports in the commands.

If this is the first time you've used this board, you'll need to erase it first with `esptool --chip esp32s3 --port /dev/ttyACM0 erase-flash`

In the directory micropython/ports/esp32, run `esptool --chip esp32s3 --port /dev/ttyACM0 write-flash --flash-mode dio 0x0 build-NENE/bootloader/bootloader.bin 0x8000 build-NENE/partition_table/partition-table.bin 0x10000 build-NENE/micropython.bin `