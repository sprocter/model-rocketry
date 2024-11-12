from collections import namedtuple

accfact = 0
gyrofact = 0

D = namedtuple('D', ('acc_x', 'acc_y', 'acc_z', 'temp_f', 'gyro_x', 'gyro_y', 'gyro_z'))

def set_globals():
    global accfact 
    global gyrofact 
    accfact = 2048.0
    gyrofact = 32.8

def decode_accel_data(accel_data : tuple) -> namedtuple:
    global accfact 
    global gyrofact 
    ret = []
    ret[0] = accel_data[0] / accfact
    ret[1] = accel_data[1] / accfact
    ret[2] = accel_data[2] / accfact
    ret[3] = (accel_data[3] / 340 + 36.53) * 1.8 + 32
    ret[4] = accel_data[4] / gyrofact
    ret[5] = accel_data[5] / gyrofact
    ret[6] = accel_data[6] / gyrofact
    return D(ret[0], ret[1], ret[2], ret[3], ret[4], ret[5], ret[6])
    