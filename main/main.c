#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "app_espnow.h"
#include "app_event.h"
#include "app_network.h"
#include "app_sensor.h"
#include "app_storage.h"
#include "app_protocol.h"

#define TAG "app_main"

static atomic_bool s_registered = false;

static void app_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base != APP_EVENT_BASE)
        return;

    switch ((app_event_id_t)event_id)
    {
    case APP_EVENT_ESPNOW_REGISTERED:
    {
        app_event_espnow_registered_t *evt = (app_event_espnow_registered_t *)event_data;
        atomic_store(&s_registered, true);
        ESP_LOGI(TAG, "Registered: node_id=%d gateway=" MACSTR, evt->node_id, MAC2STR(evt->gateway_mac));
        break;
    }
    case APP_EVENT_ESPNOW_UNREGISTERED:
        atomic_store(&s_registered, false);
        ESP_LOGW(TAG, "Lost gateway, will retry registration");
        break;
    case APP_EVENT_SENSOR_DATA:
    {
        app_event_sensor_data_t *sensor_evt = (app_event_sensor_data_t *)event_data;
        esp_err_t err = app_espnow_send_data(sensor_evt->sensor_type, sensor_evt->data, sensor_evt->data_len);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "send failed (type=%d): %s", sensor_evt->sensor_type, esp_err_to_name(err));
        }
        break;
    }
    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t err;

    err = app_storage_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "storage: %s", esp_err_to_name(err));
        return;
    }

    err = app_event_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "event: %s", esp_err_to_name(err));
        return;
    }

    err = app_network_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "network: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "free heap before espnow: %lu", esp_get_free_heap_size());
    err = app_espnow_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "espnow: %s", esp_err_to_name(err));
        return;
    }

    err = app_event_handler_register(ESP_EVENT_ANY_ID, app_event_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "event register: %s", esp_err_to_name(err));
        return;
    }

    err = app_sensor_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "sensor: %s", esp_err_to_name(err));
        return;
    }

    atomic_store(&s_registered, app_espnow_is_registered());

    ESP_LOGI(TAG, "Presence node started");
}
