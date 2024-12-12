from struct import unpack
from io import open
from os import chdir, scandir, listdir, DirEntry
from more_itertools import peekable
# TODO: import filters from accelerometer

x_err = 0.029050755
y_err = -0.008103238
z_err = 0.044206185

def accel_timestamps(entries):
    cur_timestamp, entry = 0, 0
    while entry < entries:
        yield cur_timestamp
        cur_timestamp += 1/(1125/11)

def decode_accel_data(accel_reading : bytes, timestamp : float):
    raw_accel_x, raw_accel_y, raw_accel_z = unpack(">hhh", accel_reading)
    accel_x = (raw_accel_x / 1024) - x_err
    accel_y = (raw_accel_y / 1024) - y_err
    accel_z = (raw_accel_z / 1024) - z_err
    return (timestamp, accel_x, accel_y, accel_z)

# accel_data = list() 
accel_data = bytes()
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
                accel_data = f.read()
        if file.startswith('alti'):
            with open(file, 'rb') as f:
                while(bytes := f.read(8)):
                    alti_data.append(bytes)
    chdir('..')
    i = 0
    accel_timestamp = peekable(accel_timestamps(len(accel_data)//6))
    with open('launch'+launch.name+'.csv', 'wt') as f:
        #f.write("acc_x, acc_y, acc_z, acc_temp_f, gyro_x, gyro_y, gyro_z, roll, pitch, alti_temp_f, alti_ft, humid_pct\n")
        f.write("time, acc_x, acc_y, acc_z\n")
        for i in range(len(accel_data)//6):
            f.write(','.join(str(x) for x in decode_accel_data(accel_data[i * 6 : (i + 1) * 6], next(accel_timestamp))) + "\n")
            i += 1
