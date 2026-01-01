/*
A buffer which will not be exposed to MicroPython's garbage collector.

This module for MicroPython will allow the user to declare a buffer of floats of a specified size, and then store floats to it / retrieve floats from it. It does essentially no error checking, and must be initialized (using `init_buffer`) and de-initialized (using `free_buffer`) manually.

Its purpose is to enable the use of memory (e.g., SPIRAM / PSRAM) without slowing down the MicroPython garbage collector as would happen if you just declared a similarly-sized array/bytearray. Like all buffers in MicroPython, you should declare (initialize) it early in the program to avoid issues with fragmentation blocking the initialization.

--------------------------------------------------------------------------------
Copyright (C) 2025 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
*/

#include "py/runtime.h"

static float *buffer;
static bool initialized = false;

// Store the given value at the given index
static mp_obj_t store(mp_obj_t idx_obj, mp_obj_t value_obj) {
    if(initialized){
        int idx = mp_obj_get_int(idx_obj);
        float value = mp_obj_get_float(value_obj);
        buffer[idx] = value;
    }
    return mp_const_none;
}

// Retrieve the float from the given index
static mp_obj_t retrieve_from(mp_obj_t idx_obj) {
    if(initialized){
        int idx = mp_obj_get_int(idx_obj);
        float ret_val = buffer[idx];
        return mp_obj_new_float(ret_val);
    } else {
        return mp_const_none;
    }
}

// Initialize the buffer
static mp_obj_t init_buffer(mp_obj_t size_obj) {
    if(!initialized){
        int size = mp_obj_get_int(size_obj);
        buffer = malloc(size * sizeof(float));
        initialized = true;
    }
    return mp_const_none;
}

// Release the buffer / its memory
static mp_obj_t free_buffer() {
    if(initialized){
        initialized = false;
        free(buffer);
    }
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_2(store_obj, store);
static MP_DEFINE_CONST_FUN_OBJ_1(retrieve_obj, retrieve_from);
static MP_DEFINE_CONST_FUN_OBJ_1(init_obj, init_buffer);
static MP_DEFINE_CONST_FUN_OBJ_0(free_obj, free_buffer);

static const mp_rom_map_elem_t example_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_hidden_buffer) },
    { MP_ROM_QSTR(MP_QSTR_store), MP_ROM_PTR(&store_obj) },
    { MP_ROM_QSTR(MP_QSTR_retrieve_from), MP_ROM_PTR(&retrieve_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_buffer), MP_ROM_PTR(&init_obj) },
    { MP_ROM_QSTR(MP_QSTR_free_buffer), MP_ROM_PTR(&free_obj) },
};
static MP_DEFINE_CONST_DICT(example_module_globals, example_module_globals_table);

const mp_obj_module_t hidden_buffer = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&example_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_hidden_buffer, hidden_buffer);