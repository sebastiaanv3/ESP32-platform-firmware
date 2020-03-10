#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/runtime.h"

#include <driver_lis2dh12.h>

#ifdef CONFIG_DRIVER_LIS2DH12_ENABLE

// get_X()
STATIC mp_obj_t lis2dh12_get_X() {
    // convert to MicroPython object.
    return mp_obj_new_float(driver_lis2dh12_get_X());
}
// Define a Python reference to the function above
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_X_obj, lis2dh12_get_X);

// get_Y()
STATIC mp_obj_t lis2dh12_get_Y() {
    return mp_obj_new_float(driver_lis2dh12_get_Y());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_Y_obj, lis2dh12_get_Y);

// get_Z()
STATIC mp_obj_t lis2dh12_get_Z() {
    return mp_obj_new_float(driver_lis2dh12_get_Z());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_Z_obj, lis2dh12_get_Z);

// get_raw_X()
STATIC mp_obj_t lis2dh12_get_raw_X() {
    return mp_obj_new_int(driver_lis2dh12_get_raw_X());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_raw_X_obj, lis2dh12_get_raw_X);

// get_raw_Y()
STATIC mp_obj_t lis2dh12_get_raw_Y() {
    return mp_obj_new_int(driver_lis2dh12_get_raw_Y());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_raw_Y_obj, lis2dh12_get_raw_Y);

// get_raw_Z()
STATIC mp_obj_t lis2dh12_get_raw_Z() {
    return mp_obj_new_int(driver_lis2dh12_get_raw_Z());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_raw_Z_obj, lis2dh12_get_raw_Z);

// enable_temp()
STATIC mp_obj_t lis2dh12_enable_temp() {
    driver_lis2dh12_enable_temp();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_enable_temp_obj, lis2dh12_enable_temp);

// get_temp()
STATIC mp_obj_t lis2dh12_get_temperature() {
    return mp_obj_new_float(driver_lis2dh12_get_temperature());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_get_temperature_obj, lis2dh12_get_temperature);

// disable_temp()
STATIC mp_obj_t lis2dh12_disable_temp() {
    driver_lis2dh12_disable_temp();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_disable_temp_obj, lis2dh12_disable_temp);

// enable_tap()
STATIC mp_obj_t lis2dh12_enable_tap() {
    driver_lis2dh12_enable_tap();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_enable_tap_obj, lis2dh12_enable_tap);

// is_tapped()
STATIC mp_obj_t lis2dh12_is_tapped() {
    return mp_obj_new_bool(driver_lis2dh12_is_tapped());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_is_tapped_obj, lis2dh12_is_tapped);

// disable_tap()
STATIC mp_obj_t lis2dh12_disable_tap() {
    driver_lis2dh12_disable_tap();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(lis2dh12_disable_tap_obj, lis2dh12_disable_tap);

// Define all properties of the lis2dh12 module.
STATIC const mp_rom_map_elem_t lis2dh12_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_lis2dh12) },
    // accelerometer readings in milli-g's
    { MP_ROM_QSTR(MP_QSTR_get_X), MP_ROM_PTR(&lis2dh12_get_X_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_Y), MP_ROM_PTR(&lis2dh12_get_Y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_Z), MP_ROM_PTR(&lis2dh12_get_Z_obj) },
    // raw 16 bit accelerometer readings
    { MP_ROM_QSTR(MP_QSTR_get_raw_X), MP_ROM_PTR(&lis2dh12_get_raw_X_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_raw_Y), MP_ROM_PTR(&lis2dh12_get_raw_Y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_raw_Z), MP_ROM_PTR(&lis2dh12_get_raw_Z_obj) },
    // temperature sensor
    { MP_ROM_QSTR(MP_QSTR_enable_temp),     MP_ROM_PTR(&lis2dh12_enable_temp_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_temperature), MP_ROM_PTR(&lis2dh12_get_temperature_obj) },
    { MP_ROM_QSTR(MP_QSTR_disable_temp),    MP_ROM_PTR(&lis2dh12_disable_temp_obj) },
    // tap sensor
    { MP_ROM_QSTR(MP_QSTR_enable_tap),  MP_ROM_PTR(&lis2dh12_enable_tap_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_tapped),   MP_ROM_PTR(&lis2dh12_is_tapped_obj) },
    { MP_ROM_QSTR(MP_QSTR_disable_tap), MP_ROM_PTR(&lis2dh12_disable_tap_obj) },
};

STATIC MP_DEFINE_CONST_DICT(lis2dh12_module_globals, lis2dh12_module_globals_table);

const mp_obj_module_t lis2dh12_module = {
    .base    = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&lis2dh12_module_globals,
};

#endif //CONFIG_DRIVER_LIS2DH12_ENABLE
