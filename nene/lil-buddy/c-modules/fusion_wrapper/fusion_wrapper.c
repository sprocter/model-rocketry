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

static FusionAhrs fusion;

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

    mp_obj_t *acc = NULL;
    size_t acc_len = 0;
    mp_obj_get_array(acc_obj, &acc_len, &acc);
    FusionVector accelerometer;
    accelerometer.axis.x = mp_obj_get_float(acc[0]);
    accelerometer.axis.y = mp_obj_get_float(acc[1]);
    accelerometer.axis.z = mp_obj_get_float(acc[2]);

    mp_obj_t *mag = NULL;
    size_t mag_len = 0;
    mp_obj_get_array(mag_obj, &mag_len, &mag);
    FusionVector magnetometer;
    magnetometer.axis.x = mp_obj_get_float(mag[0]);
    magnetometer.axis.y = mp_obj_get_float(mag[1]);
    magnetometer.axis.z = mp_obj_get_float(mag[2]);

    FusionAhrsUpdate(&fusion, gyroscope, accelerometer, magnetometer);
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusion));

    mp_obj_t euler_list[] = {
        mp_obj_new_float(euler.angle.roll),
        mp_obj_new_float(euler.angle.pitch),
        mp_obj_new_float(euler.angle.yaw),
    };

    return mp_obj_new_list(3, euler_list);
}

// Initialize the AHRS Fusion Algorithm
static mp_obj_t init_ahrs() {
    FusionAhrsInitialise(&fusion);

    const FusionAhrsSettings settings = {
        .sampleRate = 45, // TODO: Hardcoded
        .convention = FusionConventionNwu,
        .gain = 0.5f, // TODO: Hardcoded
        .gyroscopeRange = 500.0f, // TODO: Hardcoded
        .accelerationRejection = 10.0f, // TODO: Hardcoded
        .magneticRejection = 10.0f, // TODO: Hardcoded
        .rejectionTimeout = 5.0f, // TODO: Hardcoded
    };

    FusionAhrsSetSettings(&fusion, &settings);

    FusionBias bias;
    FusionBiasInitialise(&bias);

    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = 45; // TODO: Hardcoded

    FusionBiasSetSettings(&bias, &biasSettings);

    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_0(init_obj, init_ahrs);
static MP_DEFINE_CONST_FUN_OBJ_3(update_obj, update);

static const mp_rom_map_elem_t fusion_wrapper_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fusion_wrapper) },
    { MP_ROM_QSTR(MP_QSTR_init_ahrs), MP_ROM_PTR(&init_obj) },
    { MP_ROM_QSTR(MP_QSTR_update), MP_ROM_PTR(&update_obj) },
};
static MP_DEFINE_CONST_DICT(fusion_wrapper_globals, fusion_wrapper_globals_table);

const mp_obj_module_t fusion_wrapper = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&fusion_wrapper_globals,
};

MP_REGISTER_MODULE(MP_QSTR_fusion_wrapper, fusion_wrapper);