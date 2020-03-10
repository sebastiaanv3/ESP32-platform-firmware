/* An I2C LIS2DH12 accelerometer driver
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
#include <string.h>

#include "include/system.h"
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
    if (len > 1)
    {
        // multi byte read, so first bit == 1
        reg |= 0x80;
    }

    // TODO repeatedly reading something fails due to a timeout...
    esp_err_t err = driver_i2c_read_reg(LIS2DH12_ADDRESS, reg, buf, len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error reading i2c data: 0x%x", err);
        return err;
    }
    else
    {
        return ESP_OK;
    }
}

int32_t platform_write(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // error
    if (len > 30)
    {
        ESP_LOGE(TAG, "error setting up i2c write");
        return 1;
    }

    esp_err_t err = driver_i2c_write_buffer_reg(LIS2DH12_ADDRESS, reg, buf, len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error writing i2c data: 0x%x", err);
        return err;
    }
    else
    {
        return ESP_OK;
    }
}

esp_err_t driver_lis2dh12_init(void)
{
    if (driver_lis2dh12_init_done)
        return ESP_OK;

    xIsFresh = false;
    yIsFresh = false;
    zIsFresh = false;
    tempIsFresh = false;

    currentScale = 0;
    currentMode = 0;

    ESP_LOGD(TAG, "init called");

    // configure the HW interface
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    // TODO: optional, we don't need it and set it to 0 for now
    dev_ctx.handle = (void*)0;

    if (!isConnected())
    {
        return false;
    }
    
    // enable block data update
    lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    // set output data rate to 25Hz
    setDataRate(LIS2DH12_ODR_25Hz);
    // set full scale to 2g
    setScale(LIS2DH12_2g);
    // continuous mode with 12 bit resolution
    setMode(LIS2DH12_HR_12bit);

    driver_lis2dh12_init_done = true;
    ESP_LOGD(TAG, "init done");

    return ESP_OK;
}

bool isConnected(void)
{
    // TODO I2C read OK?
    uint8_t data[1];
    platform_read(0, 0, data, sizeof(data));
    static uint8_t whoami;
    lis2dh12_device_id_get(&dev_ctx, &whoami);
    if (whoami == LIS2DH12_ID)
    {
        ESP_LOGD(TAG, "LIS2DH12 found");
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "LIS2DH12 not found");
        return false;
    }
}

bool available(void)
{
    lis2dh12_reg_t reg;
    lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
    if (reg.byte)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void waitForNewData(void)
{
    while (!available())
    {
        vTaskDelay(300 / portTICK_RATE_MS);
    }
}

uint8_t getDataRate(void)
{
    lis2dh12_odr_t dataRate;
    lis2dh12_data_rate_get(&dev_ctx, &dataRate);
    return (uint8_t)dataRate;
}

void setDataRate(uint8_t dataRate)
{
    if (dataRate > LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP)
    {
        dataRate = LIS2DH12_ODR_25Hz;
    }
    lis2dh12_data_rate_set(&dev_ctx, (lis2dh12_odr_t)dataRate);
}

uint8_t getScale(void)
{
    lis2dh12_fs_t scale;
    lis2dh12_full_scale_get(&dev_ctx, &scale);
    return (uint8_t)scale;
}

void setScale(uint8_t scale)
{
    if (scale > LIS2DH12_16g)
    {
        scale = LIS2DH12_2g;
    }

    currentScale = scale; // required for mg conversion in get_X(), get_Y() and get_Z() functions

    lis2dh12_full_scale_set(&dev_ctx, (lis2dh12_fs_t)scale);
}

void setMode(uint8_t mode)
{
    if (mode > LIS2DH12_HR_12bit)
    {
        mode = LIS2DH12_HR_12bit;
    }
    
    currentMode = mode;

    lis2dh12_operating_mode_set(&dev_ctx, (lis2dh12_op_md_t)mode);
}

uint8_t getMode(void)
{
    lis2dh12_op_md_t mode;
    lis2dh12_operating_mode_get(&dev_ctx, &mode);
    return (uint8_t)mode;
}

void getTempData(void)
{
    axis1bit16_t data_raw_temperature;
    memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);

    switch (currentMode)
    {
        case LIS2DH12_HR_12bit: // high resolution
            temperatureC = lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);
            break;
        case LIS2DH12_NM_10bit: // normal mode
            temperatureC = lis2dh12_from_lsb_nm_to_celsius(data_raw_temperature.i16bit);
            break;
        case LIS2DH12_LP_8bit:  // low power mode
            temperatureC = lis2dh12_from_lsb_lp_to_celsius(data_raw_temperature.i16bit);
            break;
    }

    tempIsFresh = true;
}

void parseAccelData(void)
{
    axis3bit16_t data_raw_acceleration;
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

    rawX = data_raw_acceleration.i16bit[0];
    rawY = data_raw_acceleration.i16bit[1];
    rawZ = data_raw_acceleration.i16bit[2];

    switch (currentScale)
    {
        case LIS2DH12_2g:
            switch (currentMode)
            {
                case LIS2DH12_HR_12bit: // high resolution
                    accelX = lis2dh12_from_fs2_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs2_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs2_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit: // normal mode
                    accelX = lis2dh12_from_fs2_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs2_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs2_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // low power mode
                    accelX = lis2dh12_from_fs2_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs2_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs2_lp_to_mg(rawZ);
                    break;
            }
            break;

        case LIS2DH12_4g:
            switch (currentMode)
            {
                case LIS2DH12_HR_12bit: // high resolution
                    accelX = lis2dh12_from_fs4_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs4_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs4_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit: // normal mode
                    accelX = lis2dh12_from_fs4_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs4_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs4_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // low power mode
                    accelX = lis2dh12_from_fs4_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs4_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs4_lp_to_mg(rawZ);
                    break;
            }
            break;

        case LIS2DH12_8g:
            switch (currentMode)
            {
                case LIS2DH12_HR_12bit: // high resolution
                    accelX = lis2dh12_from_fs8_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs8_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs8_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit: // normal mode
                    accelX = lis2dh12_from_fs8_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs8_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs8_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // low power mode
                    accelX = lis2dh12_from_fs8_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs8_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs8_lp_to_mg(rawZ);
                    break;
            }
            break;

        case LIS2DH12_16g:
            switch (currentMode)
            {
                case LIS2DH12_HR_12bit: // high resolution
                    accelX = lis2dh12_from_fs16_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs16_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs16_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit: // normal mode
                    accelX = lis2dh12_from_fs16_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs16_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs16_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // low power mode
                    accelX = lis2dh12_from_fs16_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs16_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs16_lp_to_mg(rawZ);
                    break;
            }
            break;

        // 2g
        default:
            accelX = lis2dh12_from_fs2_hr_to_mg(rawX);
            accelY = lis2dh12_from_fs2_hr_to_mg(rawY);
            accelZ = lis2dh12_from_fs2_hr_to_mg(rawZ);
            break;
    }

    xIsFresh = true;
    yIsFresh = true;
    zIsFresh = true;
}

// return latest acceleration data in milli-g's
float driver_lis2dh12_get_X(void)
{
    if (!xIsFresh)
    {
        waitForNewData();
        parseAccelData();
    }
    xIsFresh = false;
    return accelX;
}

float driver_lis2dh12_get_Y(void)
{
    if (!yIsFresh)
    {
        waitForNewData();
        parseAccelData();
    }
    yIsFresh = false;
    return accelY;
}

float driver_lis2dh12_get_Z(void)
{
    if (!zIsFresh)
    {
        waitForNewData();
        parseAccelData();
    }
    zIsFresh = false;
    return accelZ;
}


// return raw 16 bit acceleration reading
int16_t driver_lis2dh12_get_raw_X(void)
{
    if (!xIsFresh)
    {
        waitForNewData();
        parseAccelData();
    }
    xIsFresh = false;
    return rawX;
}

int16_t driver_lis2dh12_get_raw_Y(void)
{
    if (!yIsFresh)
    {
        waitForNewData();
        parseAccelData();
    }
    yIsFresh = false;
    return rawY;
}

int16_t driver_lis2dh12_get_raw_Z(void)
{
    if (!zIsFresh)
    {
        waitForNewData();
        parseAccelData();
    }
    zIsFresh = false;
    return rawZ;
}

// onboard temperature sensor
void driver_lis2dh12_enable_temp(void)
{
    lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);
}

float driver_lis2dh12_get_temperature(void)
{
    if (!tempIsFresh)
    {
        waitForNewData();
        getTempData();
    }
    tempIsFresh = false;
    return temperatureC;
}

void driver_lis2dh12_disable_temp(void)
{
    lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);
}

// single tap detection
void driver_lis2dh12_enable_tap(void)
{
    lis2dh12_click_cfg_t newBits;
    if (lis2dh12_tap_conf_get(&dev_ctx, &newBits) == 0)
    {
        newBits.xs = true;
        newBits.ys = true;
        newBits.zs = true;
        lis2dh12_tap_conf_set(&dev_ctx, &newBits);
    }
}

bool driver_lis2dh12_is_tapped(void)
{
    lis2dh12_click_src_t interruptSource;
    lis2dh12_tap_source_get(&dev_ctx, &interruptSource);
    if (interruptSource.x || interruptSource.y || interruptSource.z)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void driver_lis2dh12_disable_tap(void)
{
    lis2dh12_click_cfg_t newBits;
    if (lis2dh12_tap_conf_get(&dev_ctx, &newBits) == 0)
    {
        newBits.xs = false;
        newBits.ys = false;
        newBits.zs = false;
        lis2dh12_tap_conf_set(&dev_ctx, &newBits);
    }
}

#endif //CONFIG_DRIVER_LIS2DH12_ENABLE
