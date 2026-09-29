
# Development Environment Setup

## Prerequisites

This guide assumes that the following repositories are cloned into `~/git/`. If your setup differs, you may have to modify some paths.

1. [Micropython](https://github.com/micropython/micropython)
2. [ESP-IDF](https://github.com/espressif/esp-idf.git)
    * Note: Be sure to clone a version supported by Micropython, e.g., `git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git`
3. [ulab](https://github.com/v923z/micropython-ulab)
4. [FTP Server](https://github.com/robert-hh/FTP-Server-for-ESP8266-ESP32-and-PYBD)
5. [This repository](https://github.com/sprocter/model-rocketry/)

You'll also need to either download a few files or clone a few other repositories (listed below).

## Modify Build Files

1. Nene-specific port customizations
    1. From the `micropython/ports/esp32/boards` directory, copy the generic board to a new one called NENE, e.g. `cp -r ESP32_GENERIC_S3/ NENE`
    2. In the newly-created `micropython/ports/esp32/boards/NENE` directory, replace the following two files with the versions from `model-rocketry/nene/lil-buddy/micropython-tweaks`:
        1. sdkconfig.board
        2. mpconfigboard.h
    3. Copy `partitions_nene.csv` into `micropython/ports/esp32/boards/NENE`
    4. Modify `micropython/ports/esp32/boards/NENE/mpconfigboard.cmake`: add `${MICROPY_BOARD_DIR}/sdkconfig.board` below line 5 / as a new line 6.
2. Tweak ulab for speed and space usage by modifying `ulab/code/ulab.h`
    1. Disable complex number support
        * Change line 36 to `#define ULAB_SUPPORTS_COMPLEX               (0)`
    2. Disable SciPy
        * Change line 42 to `#define ULAB_HAS_SCIPY                      (0)`
    3. Disable function pointers in iterations
        * Change line 296 to `#define ULAB_VECTORISE_USES_FUN_POINTER (0)`
3. Tweak the FTP server to not auto-start upon import.
    * In the directory `FTP-Server-for-ESP8266-ESP32-and-PYBD`, Delete the last line of `uftpd.py`

## Building

Using the build instructions in `micropython/ports/esp32/README.md` 

1. Follow the steps in the section titled "Setting up ESP-IDF and the build environment"
2. Follow the first step in the section titled "Building the firmware" to build the cross-compiler and then change to the `ports/esp32` directory. 
3. Instead of running `make` as the standard instructions suggest, run with the following options:
    1. `BOARD=NENE`
    2. `FROZEN_MANIFEST=~/git/model-rocketry/nene/lil-buddy/micropython-tweaks/manifest.py`
    3. (If building for the Xiao ESP32S3+) `BOARD_VARIANT=SPIRAM_OCT`
4. Examples (for the FeatherS3D)
    1. `make BOARD=NENE FROZEN_MANIFEST=~/git/model-rocketry/nene/lil-buddy/micropython-tweaks/manifest.py submodules`
    2. `make BOARD=NENE FROZEN_MANIFEST=~/git/model-rocketry/nene/lil-buddy/micropython-tweaks/manifest.py`

## Deploying

This section assumes you're using Linux and the device is at `/dev/ttyACM0` -- if you're using Windows, or using a different port, you'll need to change the ports in the commands.

If this is the first time you've used this board, you'll need to erase it first with `esptool --chip esp32s3 --port /dev/ttyACM0 erase-flash`

In the directory micropython/ports/esp32, run (for the FeatherS3D)`esptool --chip esp32s3 --port /dev/ttyACM0 write-flash --flash-mode dio 0x0 build-NENE/bootloader/bootloader.bin 0x8000 build-NENE/partition_table/partition-table.bin 0x10000 build-NENE/micropython.bin `

In the directory micropython/ports/esp32, run (for the Xiao ESP32S3+)`esptool --chip esp32s3 --port /dev/ttyACM0 write-flash --flash-mode dio 0x0 build-NENE-SPIRAM_OCT/bootloader/bootloader.bin 0x8000 build-NENE-SPIRAM_OCT/partition_table/partition-table.bin 0x10000 build-NENE-SPIRAM_OCT/micropython.bin `
