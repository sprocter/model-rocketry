import json, vfs, os, esp32, machine, time
from random import randint
from machine import I2C


def generate_secrets():
    secrets = {
        "wifi-ssid": "YourWiFiNameHere",
        "wifi-key": "YourWifiPasswordHere",
        "bigbuddy-addr": randint(0, 256),
        "lilbuddy-addr": randint(0, 256),
    }

    with open("/secrets.json", "w") as f:
        json.dump(secrets, f)


def reset_filesystem():
    vfs.umount("/")
    vfs.VfsLfs2.mkfs(bdev)  # type: ignore


def reset_nvs():
    p = esp32.Partition.find(esp32.Partition.TYPE_DATA, label="nvs")[0]

    # p.info()[3] is partition size
    for x in range(int(p.info()[3] / 4096)):
        p.writeblocks(x, bytearray(4096))

    machine.reset()


def print_filesystem_space():
    stat = os.statvfs("/")
    size = stat[1] * stat[2]
    free = stat[0] * stat[3]
    used = size - free

    KB = 1024
    MB = 1024 * 1024

    print("Size : {:,} bytes, {:,} KB, {} MB".format(size, size / KB, size / MB))
    print("Used : {:,} bytes, {:,} KB, {} MB".format(used, used / KB, used / MB))
    print("Free : {:,} bytes, {:,} KB, {} MB".format(free, free / KB, free / MB))


def print_adxl375_offsets():
    from adxl375 import ADXL375

    i2c = I2C(sda=41, scl=40)
    accelerometer = ADXL375(i2c)
    accelerometer.initialize()
    xs, ys, zs = [], [], []
    print("Please ensure X, Y, and Z ERR constants are set to 0 in the ADXL375 Driver")
    print("Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in five seconds.")
    time.sleep(5)
    print("Calibration beginning now, it will take 10 seconds...")
    for _ in range(250):
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        xs.append(accel_reading[0])
        ys.append(accel_reading[1])
        zs.append(accel_reading[2])
        time.sleep_ms(40)

    print("_X_ERR = ", sum(xs) / len(xs))
    print("_Y_ERR = ", sum(ys) / len(ys))
    print("_Z_ERR = ", sum(zs) / len(zs) - 1)

def print_icm20649_offsets():
    from icm20649 import ICM20649

    i2c = I2C(scl=9, sda=8)
    accelerometer = ICM20649(i2c)
    accelerometer.initialize()
    acc_xs, acc_ys, acc_zs = [], [], []
    gyro_xs, gyro_ys, gyro_zs = [], [], []
    print("Please ensure X, Y, and Z ERR constants are set to 0 in the ICM20649 Driver")
    print("Set the device face-up on a flat surface and hold it still.")
    print("Calibration begins in three seconds.")
    time.sleep(3)
    print("Calibration beginning now, it will take 10 seconds...")
    for _ in range(5):
        accelerometer.read_raw()
        accel_reading = accelerometer.decode_reading(accelerometer.buffer)
        acc_xs.append(accel_reading[0])
        acc_ys.append(accel_reading[1])
        acc_zs.append(accel_reading[2])
        gyro_xs.append(accel_reading[3])
        gyro_ys.append(accel_reading[4])
        gyro_zs.append(accel_reading[5])
        print(f"X: {accel_reading[0]}")
        print(f"Y: {accel_reading[1]}")
        print(f"Z: {accel_reading[2]}")
        time.sleep_ms(40)

    print(f"_ACC_X_ERR = const({sum(acc_xs) / len(acc_xs)})")
    print(f"_ACC_Y_ERR = const({sum(acc_ys) / len(acc_ys)})")
    print(f"_ACC_Z_ERR = const({sum(acc_zs) / len(acc_zs) - 9.80665})")
    print(f"_GYRO_X_ERR = const({sum(gyro_xs) / len(gyro_xs)})")
    print(f"_GYRO_Y_ERR = const({sum(gyro_ys) / len(gyro_ys)})")
    print(f"_GYRO_Z_ERR = const({sum(gyro_zs) / len(gyro_zs)})")

def kalman_test():
    from kalman import KalmanFilter
    
    init_timestamp = time.ticks_us()

    delta_t  = .04 # timestep, seconds
    alti_err_std_dev = 1 # meters
    accel_err_std_dev = 4 # meters per second squared

    kf = KalmanFilter(delta_t, alti_err_std_dev, accel_err_std_dev)

    u = [0] * 141
    z = [0] * 141

    u[0] = 9.642589648825
    u[1] = 9.68089736572
    u[2] = 9.709627712092
    u[3] = 9.661743801472
    u[4] = 9.67132019133
    u[5] = 9.690474147844
    u[6] = 9.642589648825
    u[7] = 9.68089736572
    u[8] = 9.71920410195
    u[9] = 9.661743801472
    u[10] = 9.690474147844
    u[11] = 9.67132019133
    u[12] = 9.690474147844
    u[13] = 9.67132019133
    u[14] = 9.700050733835
    u[15] = 9.68089736572
    u[16] = 9.709627712092
    u[17] = 9.709627712092
    u[18] = 9.709627712092
    u[19] = 9.690474147844
    u[20] = 9.700050733835
    u[21] = 9.642589648825
    u[22] = 9.661743801472
    u[23] = 9.709627712092
    u[24] = 9.690474147844
    u[25] = 9.690474147844
    u[26] = 9.67132019133
    u[27] = 9.690474147844
    u[28] = 9.661743801472
    u[29] = 9.709627712092
    u[30] = 9.700050733835
    u[31] = 9.67132019133
    u[32] = 9.67132019133
    u[33] = 9.690474147844
    u[34] = 9.72878127634
    u[35] = 9.68089736572
    u[36] = 9.709627712092
    u[37] = 9.68089736572
    u[38] = 9.661743801472
    u[39] = 9.700050733835
    u[40] = 9.68089736572
    u[41] = 9.661743801472
    u[42] = 9.661743801472
    u[43] = 9.700050733835
    u[44] = 9.661743801472
    u[45] = 9.68089736572
    u[46] = 9.747934644455
    u[47] = 9.709627712092
    u[48] = 9.690474147844
    u[49] = 9.67132019133
    u[50] = 9.690474147844
    u[51] = 9.68089736572
    u[52] = 9.72878127634
    u[53] = 9.68089736572
    u[54] = 9.700050733835
    u[55] = 9.71920410195
    u[56] = 9.700050733835
    u[57] = 9.68089736572
    u[58] = 9.661743801472
    u[59] = 9.709627712092
    u[60] = 9.996931371945
    u[61] = 9.403169899592
    u[62] = 9.451054006345
    u[63] = 16.3750853301
    u[64] = 52.2306493926
    u[65] = 108.1687812709
    u[66] = 165.5721639523
    u[67] = 219.5087389523
    u[68] = 167.3342816642
    u[69] = 33.68037456196
    u[70] = 7.1334664168544
    u[71] = 2.4408311335282
    u[72] = -0.1065994426847
    u[73] = -1.064280028294
    u[74] = -1.332430712298
    u[75] = -1.36116105867
    u[76] = -0.891897557796
    u[77] = -0.7961295011964
    u[78] = -0.815283104671
    u[79] = -0.8535903312335
    u[80] = -1.0068192374835
    u[81] = -1.0930104727325
    u[82] = -0.958935228797
    u[83] = -0.958935228797
    u[84] = -0.8440135491095
    u[85] = -0.7865526602325
    u[86] = -0.815283104671
    u[87] = -0.7673990959845
    u[88] = -0.7865526602325
    u[89] = -0.824859886795
    u[90] = -0.74824543367
    u[91] = -0.6333237539825
    u[92] = -0.5471325383468
    u[93] = -0.4705180852218
    u[94] = -0.422634056922
    u[95] = -0.3364427824464
    u[96] = -0.269405150672
    u[97] = -0.2789819524093
    u[98] = -0.2885587737599
    u[99] = -0.3077123772345
    u[100] = -0.3172891789718
    u[101] = -0.3077123772345
    u[102] = -0.3077123772345
    u[103] = -0.2981355754972
    u[104] = -0.3555964251476
    u[105] = -0.2119443206349
    u[106] = -0.1640602727218
    u[107] = -0.039561791297
    u[108] = 0.0753598883905
    u[109] = 0.02747585224538
    u[110] = -0.010831373336455
    u[111] = 0.01789904658542
    u[112] = -0.00125456669583
    u[113] = 0.10409030340905
    u[114] = 0.14239752997155
    u[115] = 0.14239752997155
    u[116] = 0.14239752997155
    u[117] = 0.2764728229403
    u[118] = 0.2477423981151
    u[119] = 0.2381655963778
    u[120] = 0.2477423981151
    u[121] = 0.1998583698153
    u[122] = 0.1807047663407
    u[123] = 0.1232439166903
    u[124] = 0.0753598883905
    u[125] = 0.10409030340905
    u[126] = 0.113667114953
    u[127] = 0.10409030340905
    u[128] = 0.2094351715526
    u[129] = 0.13282072823425
    u[130] = 0.1998583698153
    u[131] = 0.3243568708534
    u[132] = 0.343510474328
    u[133] = 0.343510474328
    u[134] = 0.2094351715526
    u[135] = 0.2764728229403
    u[136] = 0.2956264460282
    u[137] = 0.3722408991532
    u[138] = 0.2381655963778
    u[139] = 0.2573192194657
    u[140] = 0.2285887946405

    z[0] = 0
    z[1] = 0
    z[2] = 0
    z[3] = 0
    z[4] = 0
    z[5] = 0
    z[6] = 0
    z[7] = 0
    z[8] = 0
    z[9] = 0
    z[10] = 0
    z[11] = 0
    z[12] = 0
    z[13] = 0
    z[14] = 0
    z[15] = 0
    z[16] = 0
    z[17] = 0
    z[18] = 0
    z[19] = 0
    z[20] = 0
    z[21] = 0
    z[22] = 0
    z[23] = 0
    z[24] = 0
    z[25] = 0
    z[26] = 0
    z[27] = 0
    z[28] = 0
    z[29] = 0
    z[30] = 0
    z[31] = 0
    z[32] = 0
    z[33] = 0
    z[34] = 0.08981773248
    z[35] = 0.08981773248
    z[36] = 0.08981773248
    z[37] = 0
    z[38] = 0.08981773248
    z[39] = 0
    z[40] = 0
    z[41] = 0
    z[42] = 0
    z[43] = 0
    z[44] = 0
    z[45] = 0
    z[46] = 0.08981773248
    z[47] = 0
    z[48] = 0
    z[49] = 0
    z[50] = 0
    z[51] = 0
    z[52] = 0
    z[53] = 0
    z[54] = 0
    z[55] = -0.0871389062400001
    z[56] = 0
    z[57] = 0
    z[58] = 0
    z[59] = 0
    z[60] = 0
    z[61] = 0
    z[62] = 0
    z[63] = 0
    z[64] = -0.0871389062400001
    z[65] = -0.0871389062400001
    z[66] = 0.264095484
    z[67] = 1.65854060976
    z[68] = 2.8786336128
    z[69] = 4.70881027248
    z[70] = 6.62876772288
    z[71] = 7.76175870048
    z[72] = 7.67458285248
    z[73] = 9.42026206368
    z[74] = 9.76885511808
    z[75] = 10.90448810208
    z[76] = 12.03744189408
    z[77] = 13.08589903008
    z[78] = 13.69594525728
    z[79] = 14.22149482848
    z[80] = 15.52876374048
    z[81] = 16.49008221408
    z[82] = 17.45136289248
    z[83] = 18.41003935968
    z[84] = 18.84844904928
    z[85] = 19.72259106528
    z[86] = 20.68122973728
    z[87] = 21.64254760128
    z[88] = 22.60386790368
    z[89] = 23.47800991968
    z[90] = 23.91377699328
    z[91] = 24.78791900928
    z[92] = 25.40060663328
    z[93] = 26.27474864928
    z[94] = 27.06175322208
    z[95] = 27.41038163328
    z[96] = 28.19738681568
    z[97] = 29.07152883168
    z[98] = 29.68421706528
    z[99] = 30.47121980928
    z[100] = 31.16844066528
    z[101] = 32.04258268128
    z[102] = 32.04258268128
    z[103] = 32.74244737248
    z[104] = 33.26799755328
    z[105] = 33.79354590528
    z[106] = 34.05500029728
    z[107] = 34.66768792128
    z[108] = 35.19323810208
    z[109] = 35.71614505728
    z[110] = 36.15455779488
    z[111] = 36.41600913888
    z[112] = 36.94155992928
    z[113] = 37.29279316128
    z[114] = 37.72852609728
    z[115] = 38.16693517728
    z[116] = 38.60270468928
    z[117] = 38.77962280128
    z[118] = 39.12825304128
    z[119] = 39.47948810208
    z[120] = 39.83072255328
    z[121] = 40.17935279328
    z[122] = 40.26648901728
    z[123] = 40.61772529728
    z[124] = 40.96635553728
    z[125] = 41.14327669728
    z[126] = 41.40472804128
    z[127] = 41.57904316128
    z[128] = 41.66618243328
    z[129] = 41.93027639328
    z[130] = 42.10459151328
    z[131] = 42.27890663328
    z[132] = 42.45582779328
    z[133] = 42.45582779328
    z[134] = 42.63013986528
    z[135] = 42.80441840928
    z[136] = 42.71728218528
    z[137] = 42.80441840928
    z[138] = 42.80441840928
    z[139] = 42.89159730528
    z[140] = 42.97873352928

    setup_timestamp = time.ticks_us()

    kf.predict(u[0])

    update_times = []
    predict_times = []

    for i in range(1, len(z)):
        loop_start_timestamp = time.ticks_us()
        kf.update(z[i])
        # print(f"Timestep {i}: Altitude (e) {kf.x[0][0]}, Altitude (m) {z[i]}, Velocity (e) {kf.x[1][0]}")
        print(f"{i * delta_t}, {z[i]}, {kf.x[0][0]}, {kf.x[1][0]}, {u[i]}")
        update_timestamp = time.ticks_us()
        kf.predict(u[i])
        predict_timestamp = time.ticks_us()
        update_times.append(time.ticks_diff(update_timestamp, loop_start_timestamp))
        predict_times.append(time.ticks_diff(predict_timestamp, update_timestamp))

    # kf.update(z[30])
    # print(f"After Update:")
    # print(f"x = {kf.x}")
    # print(f"P = {kf.P}\n")

    # kf.predict(u[30])
    # print(f"After Prediction:")
    # print(f"x = {kf.x}")
    # print(f"P = {kf.P}")

    # setup_time = time.ticks_diff(setup_timestamp, init_timestamp)    
  
    # print(f"Setup Time: {setup_time}us")
    # print(f"Average Prediction Time: {sum(predict_times) / len(predict_times)}us")
    # print(f"Average Update Time: {sum(update_times) / len(update_times)}us")

def get_alti():
    from bmp581 import BMP581
    from machine import I2C
    i2c = I2C(scl=9, sda=8)
    alti = BMP581(i2c)
    alti.initialize()
    alti.read_raw()
    print(alti.decode_reading(alti.buffer))


generate_secrets()
