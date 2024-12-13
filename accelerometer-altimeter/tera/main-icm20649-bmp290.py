from struct import unpack
from io import open
from os import chdir, scandir, listdir, DirEntry
from more_itertools import peekable
# TODO: import filters from accelerometer

x_err = 0.029050755
y_err = -0.008103238
z_err = 0.044206185
_CURR_BARO_PRESSURE = 1033

def accel_timestamps():
    cur_timestamp = 0
    while True:
        yield cur_timestamp
        cur_timestamp += 1/(1125/11)

def alti_timestamps():
    cur_timestamp = 0
    while True:
        yield cur_timestamp
        cur_timestamp += .04

def decode_accel_data(accel_reading : bytes, timestamp : float) -> tuple[float, float, float, float]:
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", accel_reading)
    accel_x = (raw_accel_x / 1024) - x_err
    accel_y = (raw_accel_y / 1024) - y_err
    accel_z = (raw_accel_z / 1024) - z_err
    return (timestamp, accel_x, accel_y, accel_z)

def decode_alti_data(alti_reading : bytes) -> tuple[float, float]:
    coeffs_packed = bytes.fromhex("996e384df93a1a52140601684e9d6003fabd0f05f5")
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

accel_data = bytes()
alti_data = bytes()
chdir('data')
for launch in scandir():
    if DirEntry.is_file(launch):
        continue
    chdir(launch)
    #TODO: Test with multidigit files, ie alti12.bin
    for file in sorted(listdir()): 
        if file.startswith('accel'):
            with open(file, 'rb') as f:
                accel_data = f.read()
        if file.startswith('alti'):
            with open(file, 'rb') as f:
                alti_data = f.read()
    chdir('..')
    accel_idx = 0
    alti_idx = 0
    alti_idx_mod = 0 
    accel_timestamp = peekable(accel_timestamps())
    alti_timestamp = peekable(alti_timestamps())
    with open('launch'+launch.name+'.csv', 'wt') as f:
        #f.write("acc_x, acc_y, acc_z, acc_temp_f, gyro_x, gyro_y, gyro_z, roll, pitch, alti_temp_f, alti_ft, humid_pct\n")
        f.write("time, acc_x, acc_y, acc_z, altitude_ft, temp_f\n")
        while accel_idx < len(accel_data)//6:
            if accel_timestamp.peek() < alti_timestamp.peek():
                f.write(','.join(str(x) for x in decode_accel_data(accel_data[accel_idx * 6 : (accel_idx + 1) * 6], next(accel_timestamp))) + "\n")
                accel_idx += 1
            else:
                alti_idx_start = alti_idx_mod + alti_idx * 7
                alti_idx_end = alti_idx_mod + (alti_idx + 1) * 7
                if alti_data[alti_idx_start] & 192 == 64: # Control frame 
                    alti_idx_mod += 2
                    continue
                if alti_data[alti_idx_start] & 255 == 128: # Empty frame
                    alti_idx_mod += 2
                    continue
                f.write(str(next(alti_timestamp)) + ', , , ,' + ','.join(str(x) for x in decode_alti_data(alti_data[alti_idx_start : alti_idx_end]))+ "\n")
                alti_idx += 1            
