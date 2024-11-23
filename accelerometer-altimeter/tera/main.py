from collections import namedtuple
from struct import unpack, unpack_from
from math import atan, sqrt, pi
from io import open
from os import chdir, scandir, listdir, DirEntry
from itertools import batched

# TODO: import filters from accelerometer

D = namedtuple('D', ('acc_x', 'acc_y', 'acc_z', 'temp_f', 'gyro_x', 'gyro_y', 'gyro_z'))
A   = namedtuple('A', ('roll', 'pitch'))

CURR_BARO_PRESSURE = 1006

def set_globals(alti_calib : bytes):
    global accfact, gyrofact
    accfact = 2048.0
    gyrofact = 32.8

    global dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6
    alti_calib1 = alti_calib[0:26]
    alti_calib2 = alti_calib[26:33]
    (
    dig_T1,
    dig_T2,
    dig_T3,
    dig_P1,
    dig_P2,
    dig_P3,
    dig_P4,
    dig_P5,
    dig_P6,
    dig_P7,
    dig_P8,
    dig_P9,
    _,
    dig_H1,
    ) = unpack("<HhhHhhhhhhhhBB", alti_calib1)
    dig_H2, dig_H3 = unpack_from("<hB", alti_calib2)
    e4_sign = unpack_from("<b", alti_calib2, 3)[0]
    dig_H4 = (e4_sign << 4) | (alti_calib2[4] & 0xF)
    e6_sign = unpack_from("<b", alti_calib2, 5)[0]
    dig_H5 = (e6_sign << 4) | (alti_calib2[4] >> 4)
    dig_H6 = unpack_from("<b", alti_calib2, 6)[0]

def decode_accel_data(accel_reading : bytearray):
    global accfact 
    global gyrofact
    accel_data = unpack(">hhhhhhh", accel_reading)
    ret = []
    ax = accel_data[0] / accfact
    ay = accel_data[1] / accfact
    az = accel_data[2] / accfact
    temp_f = (accel_data[3] / 340 + 36.53) * 1.8 + 32
    gx = accel_data[4] / gyrofact
    gy = accel_data[5] / gyrofact
    gz = accel_data[6] / gyrofact

    z2 = az**2
    R2D = 180/pi
    roll  = atan(ax/sqrt(ay**2+z2))*R2D
    pitch = atan(ay/sqrt(ax**2+z2))*R2D

    # return (D(ax, ay, az, temp_f, gx, gy, gz), A(roll, pitch))
    return (ax, ay, az, temp_f, gx, gy, gz, roll, pitch)

def _get_raw_alti_data(readout : bytearray):
    # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
    raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
    # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
    raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
    # humidity(0xFD): (msb << 8) | lsb
    raw_hum = (readout[6] << 8) | readout[7]
    return raw_press, raw_temp, raw_hum

def _get_compensated_alti_data(alti_reading : bytearray):
    global dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6
    raw_press, raw_temp, raw_hum = _get_raw_alti_data(alti_reading)
    var1 = ((raw_temp >> 3) - (dig_T1 << 1)) * (dig_T2 >> 11)
    var2 = (
        ((((raw_temp >> 4) - dig_T1) * ((raw_temp >> 4) - dig_T1)) >> 12)
        * dig_T3
    ) >> 14
    t_fine = var1 + var2
    temp = (t_fine * 5 + 128) >> 8

    # pressure
    var1 = t_fine - 128000
    var2 = var1 * var1 * dig_P6
    var2 = var2 + ((var1 * dig_P5) << 17)
    var2 = var2 + (dig_P4 << 35)
    var1 = ((var1 * var1 * dig_P3) >> 8) + ((var1 * dig_P2) << 12)
    var1 = (((1 << 47) + var1) * dig_P1) >> 33
    if var1 == 0:
        pressure = 0
    else:
        p = 1048576 - raw_press
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (dig_P8 * p) >> 19
        pressure = ((p + var1 + var2) >> 8) + (dig_P7 << 4)

    # humidity
    h = t_fine - 76800
    h = (
        (((raw_hum << 14) - (dig_H4 << 20) - (dig_H5 * h)) + 16384) >> 15
    ) * (
        (
            (
                (
                    (
                        ((h * dig_H6) >> 10)
                        * (((h * dig_H3) >> 11) + 32768)
                    )
                    >> 10
                )
                + 2097152
            )
            * dig_H2
            + 8192
        )
        >> 14
    )
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * dig_H1) >> 4)
    h = 0 if h < 0 else h
    h = 419430400 if h > 419430400 else h
    humidity = h >> 12
    return temp, pressure, humidity

def get_alti_values(alti_reading : bytearray):
    """human readable values"""
    global CURR_BARO_PRESSURE

    t, p, h = _get_compensated_alti_data(alti_reading)

    p = p // 256
    pi = p // 100
    pd = p - pi * 100

    hi = h // 1024
    hd = h * 100 // 1024 - hi * 100
    return (
        str((t / 100) * 1.8 + 32),
        #"{}.{:02d} hPa".format(pi, pd),
        #"{}.{:02d}".format(pi, pd),
        str((1-((float(p/100)/CURR_BARO_PRESSURE) ** .190284)) * 145366.45),
        # "{}.{:02d}".format(hi, hd)
        str(h/1024)
    )

set_globals(bytes.fromhex("0a6ea6673200928e71d6d00b7f1deefff9ffb42de8d18813004b6d01001329031e"))

accel_data = list() 
alti_data = list()
chdir('data')
for launch in scandir():
    if DirEntry.is_file(launch):
        continue
    chdir(launch)
    #TODO: Test with multidigit files, ie alti12.bin
    for file in sorted(listdir()): 
        if file.startswith('accel'):
            with open(file, 'rb') as f:
                while(bytes := f.read(14)):
                    accel_data.append(bytes)
        if file.startswith('alti'):
            with open(file, 'rb') as f:
                while(bytes := f.read(8)):
                    alti_data.append(bytes)
    chdir('..')
    i = 0
    with open('launch'+launch.name+'.csv', 'wt') as f:
        f.write("acc_x, acc_y, acc_z, acc_temp_f, gyro_x, gyro_y, gyro_z, roll, pitch, alti_temp_f, alti_ft, humid_pct\n")
        for chunk in batched(accel_data, 5):
            if(len(chunk) < 5):
                break
            f.write(','.join(str(x) for x in decode_accel_data(chunk[0])) + ",")
            f.write(','.join(str(x) for x in get_alti_values(alti_data[i])) + "\n")
            f.write(','.join(str(x) for x in decode_accel_data(chunk[1])) + "\n")
            f.write(','.join(str(x) for x in decode_accel_data(chunk[2])) + "\n")
            f.write(','.join(str(x) for x in decode_accel_data(chunk[3])) + "\n")
            f.write(','.join(str(x) for x in decode_accel_data(chunk[4])) + "\n")
            i += 1
    accel_data.clear()
    alti_data.clear()
