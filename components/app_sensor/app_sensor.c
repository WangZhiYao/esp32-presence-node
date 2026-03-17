#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "app_sensor.h"

#define TAG "app_sensor"

esp_err_t app_sensor_init(void)
{
    return ESP_OK;
}

esp_err_t app_sensor_deinit(void)
{
    return ESP_OK;
}
