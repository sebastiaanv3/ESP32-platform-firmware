/*
 * Driver for the LIS2DH12 I2C acceleromter sensor
 * @author Sebastiaan Vileijn <sebastiaan.vileijn@gmail.com>
 *
 */

#ifndef DRIVER_LIS2DH12_H
#define DRIVER_LIS2DH12_H

#include <stdint.h>
#include <esp_err.h>

#include "lis2dh12_reg.h"

extern bool isConnected(void);

lis2dh12_ctx_t dev_ctx;

bool xIsFresh;
bool yIsFresh;
bool zIsFresh;
bool tempIsFresh;

// required to convert readings to mg
uint8_t currentScale;
uint8_t currentMode;

float accelX;
float accelY;
float accelZ;

uint16_t rawX;
uint16_t rawY;
uint16_t rawZ;

float temperatureC;

// settings
uint8_t getDataRate(void);
void setDataRate(uint8_t dataRate);

uint8_t getScale(void);
void setScale(uint8_t scale);

uint8_t getMode(void);
void setMode(uint8_t mode);

// blocking wait
extern void waitForNewData(void);
bool available(void);

// sensor data
void parseAccelData(void);
void getTempData(void);

__BEGIN_DECLS

extern esp_err_t driver_lis2dh12_init(void);

// return latest acceleration data in milli-g's
extern float driver_lis2dh12_get_X();
extern float driver_lis2dh12_get_Y();
extern float driver_lis2dh12_get_Z();

// return raw 16 bit acceleration reading
extern int16_t driver_lis2dh12_get_raw_X();
extern int16_t driver_lis2dh12_get_raw_Y();
extern int16_t driver_lis2dh12_get_raw_Z();

// onboard temperature sensor
extern void driver_lis2dh12_enable_temp();
extern float driver_lis2dh12_get_temperature();
extern void driver_lis2dh12_disable_temp();

// single tap detection
extern void driver_lis2dh12_enable_tap();
extern bool driver_lis2dh12_is_tapped();
extern void driver_lis2dh12_disable_tap();

__END_DECLS

#endif // DRIVER_LIS2DH12_H
