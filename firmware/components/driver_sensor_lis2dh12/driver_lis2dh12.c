/*
 *
 * datasheet:
 *     https://www.st.com/en/mems-and-sensors/lis2dh12.html
 *
 * source:
 *     https://github.com/sparkfun/SparkFun_LIS2DH12_Arduino_Library/blob/master/src/lis2dh12_reg.h
 *     https://github.com/sparkfun/SparkFun_LIS2DH12_Arduino_Library/blob/master/src/lis2dh12_reg.c
 */

#include <sdkconfig.h>
#include <stdbool.h>

#include "include/driver_lis2dh12.h"
#include "include/driver_i2c.h"
#include "include/lis2dh12_reg.h"

#include "esp_log.h"

#include <stdbool.h>

#ifdef CONFIG_DRIVER_LIS2DH12_ENABLE
#if CONFIG_I2C_MASTER_FREQ_HZ > 400000
#error "The LIS2DH12 sensor supports an I2C clock of at most 400kHz."
#endif

#define LIS2DH12_ADDRESS 0x18

static const char *TAG = "driver_lis2dh12";
static bool driver_lis2dh12_init_done = false;

int32_t platform_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // TODO
    esp_err_t err = driver_i2c_read_bytes(LIS2DH12_ADDRESS, buf, len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error reading i2c data");
        return err;
    }

    return ESP_OK;
}

int32_t platform_write(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // error
    if (len > 30)
    {
        ESP_LOGE(TAG, "error setting up i2c write");
        return 1;
    }

    // TODO: to check
    esp_err_t err = driver_i2c_write_buffer(LIS2DH12_ADDRESS, buf, len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error writing i2c data");
        return err;
    }

    return ESP_OK;
}

// TODO: implement them all ;-)

esp_err_t driver_lis2dh12_init()
{
    if (driver_lis2dh12_init_done)
        return ESP_OK;

    ESP_LOGD(TAG, "init called");

    // configure the HW interface
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = (void*)0;

    // TODO I2C read
    //
    static uint8_t whoami;
    lis2dh12_device_id_get(&dev_ctx, &whoami);
    if (whoami == LIS2DH12_ID)
    {
        ESP_LOGD(TAG, "found");
    }
    // TODO: continue

    driver_lis2dh12_init_done = true;
    ESP_LOGD(TAG, "init done");

    return ESP_OK;
}

// return latest acceleration data in milli-g's
float driver_lis2dh12_get_X()
{
    // TODO
    return 0.1f;
}

float driver_lis2dh12_get_Y()
{
    // TODO
    return 0.2f;
}

float driver_lis2dh12_get_Z()
{
    // TODO
    return 0.3f;
}


// return raw 16 bit acceleration reading
int16_t driver_lis2dh12_get_raw_X()
{
    // TODO
    return 1;
}

int16_t driver_lis2dh12_get_raw_Y()
{
    // TODO
    return 2;
}

int16_t driver_lis2dh12_get_raw_Z()
{
    // TODO
    return 3;
}


// onboard temperature sensor
void driver_lis2dh12_enable_temp()
{
    // TODO
}

float driver_lis2dh12_get_temperature()
{
    // TODO
    return 20.0f;
}

void driver_lis2dh12_disable_temp()
{
    // TODO
}

// single tap detection
void driver_lis2dh12_enable_tap()
{
    // TODO
}

bool driver_lis2dh12_is_tapped()
{
    // TODO
    return true;
}

void driver_lis2dh12_disable_tap()
{
    // TODO
}

#endif //CONFIG_DRIVER_LIS2DH12_ENABLE
