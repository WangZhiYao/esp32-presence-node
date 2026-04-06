#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "app_sensor.h"
#include "app_event.h"
#include "app_protocol.h"
#include "ld2412.h"

#define TAG "app_sensor"

static void ld2412_data_handler(const ld2412_data_t *data, void *user_ctx)
{
    app_protocol_presence_data_t payload = {
        .target_state    = (uint8_t)data->target_state,
        .moving_distance = data->moving_distance,
        .moving_energy   = data->moving_energy,
        .static_distance = data->static_distance,
        .static_energy   = data->static_energy,
    };

    ESP_LOGI(TAG, "Presence data: state=%d mov=%ucm/%u%% sta=%ucm/%u%%",
             payload.target_state, payload.moving_distance, payload.moving_energy,
             payload.static_distance, payload.static_energy);

    app_event_sensor_data_t evt = {
        .sensor_type = APP_PROTOCOL_SENSOR_PRESENCE,
        .data_len    = sizeof(payload),
    };
    memcpy(evt.data, &payload, sizeof(payload));

    app_event_post_with_timeout(APP_EVENT_SENSOR_DATA, &evt, sizeof(evt), 0);
}

esp_err_t app_sensor_init(void)
{
    const ld2412_config_t cfg = {
        .uart_port = CONFIG_LD2412_UART_PORT_NUM,
        .tx_pin    = CONFIG_LD2412_UART_TX_PIN,
        .rx_pin    = CONFIG_LD2412_UART_RX_PIN,
        .data_cb   = ld2412_data_handler,
        .user_ctx  = NULL,
    };
    return ld2412_init(&cfg);
}

esp_err_t app_sensor_deinit(void)
{
    return ld2412_deinit();
}
