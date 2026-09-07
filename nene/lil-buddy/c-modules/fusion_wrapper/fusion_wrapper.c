/*
A wrapper for x-io Technologies' Fusion library

This module makes the Fusion library[1], developed by x-io Technologies, accessible as a micropython module. It is customized for the Nene flight computer / is not a general-purpose wrapper. The copyright statement below applies only to this file; see the Fusion library for the license terms that apply to it.

1: https://github.com/xioTechnologies/Fusion

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
*/

#include "py/runtime.h"
#include "Fusion.h"
#include <math.h>

static FusionAhrs fusion;
static FusionBias bias;
static FusionRemapAlignment remap;
static float declination;

// Update the AHRS with a new reading, return Euler angles
static mp_obj_t update(
        mp_obj_t gyro_obj,
        mp_obj_t acc_obj,
        mp_obj_t mag_obj) {
    mp_obj_t *gyro = NULL;
    size_t gyro_len = 0;
    mp_obj_get_array(gyro_obj, &gyro_len, &gyro);
    FusionVector gyroscope;
    gyroscope.axis.x = mp_obj_get_float(gyro[0]);
    gyroscope.axis.y = mp_obj_get_float(gyro[1]);
    gyroscope.axis.z = mp_obj_get_float(gyro[2]);
    FusionVector gyro_remapped = FusionRemap(gyroscope, remap);

    mp_obj_t *acc = NULL;
    size_t acc_len = 0;
    mp_obj_get_array(acc_obj, &acc_len, &acc);
    FusionVector accelerometer;
    // The library wants gravity in g, we track in m/s, so we convert here
    accelerometer.axis.x = mp_obj_get_float(acc[0]) * 0.101971621298;
    accelerometer.axis.y = mp_obj_get_float(acc[1]) * 0.101971621298;
    accelerometer.axis.z = mp_obj_get_float(acc[2]) * 0.101971621298;
    FusionVector acc_remapped = FusionRemap(accelerometer, remap);

    mp_obj_t *mag = NULL;
    size_t mag_len = 0;
    mp_obj_get_array(mag_obj, &mag_len, &mag);
    FusionVector magnetometer;
    magnetometer.axis.x = mp_obj_get_float(mag[0]);
    magnetometer.axis.y = mp_obj_get_float(mag[1]);
    magnetometer.axis.z = mp_obj_get_float(mag[2]);
    FusionVector mag_remapped = FusionRemap(magnetometer, remap);

    // Update bias algorithm
    FusionVector gyro_biased = FusionBiasUpdate(&bias, gyro_remapped);

    FusionAhrsUpdate(&fusion, gyro_biased, acc_remapped, mag_remapped);

    return mp_const_none;
}

static mp_obj_t get_euler() {
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusion));

    float raw_roll = euler.angle.roll;
    float raw_pitch = euler.angle.pitch;
    float raw_yaw = euler.angle.yaw;

    float roll = 0.0f;
    float pitch = 0.0f;
    float heading = 0.0f;

    if(raw_yaw < declination){
        roll = 360 + raw_yaw - declination;
    } else {
        roll = raw_yaw - declination;
    }

    pitch = -1 * raw_pitch;
    heading = raw_roll;

    float pitch_rad = pitch * (M_PI / 180.0);
    float heading_rad = heading * (M_PI / 180.0);
    float tilt_rad = acos(cos(heading_rad) * cos(pitch_rad));
    float tilt = (180.0 / M_PI) * tilt_rad;

    mp_obj_t ret_list[] = {
        mp_obj_new_float(roll),
        mp_obj_new_float(pitch),
        mp_obj_new_float(heading),
        mp_obj_new_float(tilt),
    };

    return mp_obj_new_list(4, ret_list);
}

static mp_obj_t get_lin_acc() {
    FusionVector lin_acc = FusionAhrsGetLinearAcceleration(&fusion);
    mp_obj_t lin_acc_list[] = {
        mp_obj_new_float(lin_acc.axis.x),
        mp_obj_new_float(lin_acc.axis.y),
        mp_obj_new_float(lin_acc.axis.z),
    };
    return mp_obj_new_list(3, lin_acc_list);
}

static mp_obj_t get_states() {
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&fusion);
    mp_obj_t state_list[] = {
        mp_obj_new_float(states.accelerationError),
        mp_obj_new_bool(states.accelerometerIgnored),
        mp_obj_new_float(states.accelerationRecoveryTrigger),
        mp_obj_new_float(states.magneticError),
        mp_obj_new_bool(states.magnetometerIgnored),
        mp_obj_new_float(states.magneticRecoveryTrigger),
    };
    return mp_obj_new_list(6, state_list);
}

static mp_obj_t get_flags() {
    FusionAhrsFlags flags = FusionAhrsGetFlags(&fusion);
    mp_obj_t flag_list[] = {
        mp_obj_new_bool(flags.startup),
        mp_obj_new_bool(flags.overrangeRecovery),
        mp_obj_new_bool(flags.accelerationRecovery),
        mp_obj_new_bool(flags.magneticRecovery),
    };
    return mp_obj_new_list(4, flag_list);
}

// Initialize the AHRS Fusion Algorithm
static mp_obj_t init_ahrs(size_t n_args, const mp_obj_t *args) {
    mp_float_t sample_rate = mp_obj_get_float(args[0]);
    mp_float_t tgt_gain = mp_obj_get_float(args[1]);
    mp_float_t gyro_range = mp_obj_get_float(args[2]);
    mp_float_t acc_rej = mp_obj_get_float(args[3]);
    mp_float_t mag_rej = mp_obj_get_float(args[4]);
    mp_float_t rej_timeout = mp_obj_get_float(args[5]);
    mp_float_t decl = mp_obj_get_float(args[6]);
    mp_int_t alignment = mp_obj_get_int(args[7]);
    
    FusionAhrsInitialise(&fusion);

    const FusionAhrsSettings settings = {
        .sampleRate = sample_rate, // 45
        .convention = FusionConventionNed,
        .gain = tgt_gain, // 0.5
        .gyroscopeRange = gyro_range, // 500
        .accelerationRejection = acc_rej, // 10
        .magneticRejection = mag_rej, // 10
        .rejectionTimeout = rej_timeout, // 5
    };

    FusionAhrsSetSettings(&fusion, &settings);

    FusionBiasInitialise(&bias);

    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = sample_rate;

    FusionBiasSetSettings(&bias, &biasSettings);

    if(alignment == 0){
        remap = FusionRemapAlignmentPXPYNZ;
    } else if(alignment == 1) {
        remap = FusionRemapAlignmentPZPYNX;
    } else if(alignment == 2) {
        remap = FusionRemapAlignmentPXPYPZ; // TODO: Xiao alignment, TBD
    } else {
        remap = FusionRemapAlignmentPXPYPZ; // TODO: Set better default?
    }
    declination = decl;

    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(init_ahrs_obj, 8, 8, init_ahrs);
static MP_DEFINE_CONST_FUN_OBJ_0(get_euler_obj, get_euler);
static MP_DEFINE_CONST_FUN_OBJ_0(get_states_obj, get_states);
static MP_DEFINE_CONST_FUN_OBJ_0(get_flags_obj, get_flags);
static MP_DEFINE_CONST_FUN_OBJ_0(get_lin_acc_obj, get_lin_acc);
static MP_DEFINE_CONST_FUN_OBJ_3(update_obj, update);

static const mp_rom_map_elem_t fusion_wrapper_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fusion_wrapper) },
    { MP_ROM_QSTR(MP_QSTR_init_ahrs), MP_ROM_PTR(&init_ahrs_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_euler), MP_ROM_PTR(&get_euler_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_states), MP_ROM_PTR(&get_states_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_flags), MP_ROM_PTR(&get_flags_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_lin_acc), MP_ROM_PTR(&get_lin_acc_obj) },
    { MP_ROM_QSTR(MP_QSTR_update), MP_ROM_PTR(&update_obj) },
};
static MP_DEFINE_CONST_DICT(fusion_wrapper_globals, fusion_wrapper_globals_table);

const mp_obj_module_t fusion_wrapper = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&fusion_wrapper_globals,
};

MP_REGISTER_MODULE(MP_QSTR_fusion_wrapper, fusion_wrapper);