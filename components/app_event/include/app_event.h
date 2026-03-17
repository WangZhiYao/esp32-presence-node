#pragma once

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(APP_EVENT_BASE);

/**
 * @brief Application Layer Event ID Enumeration
 */
typedef enum
{
    /* ── ESP-NOW Events ── */
    APP_EVENT_ESPNOW_REGISTERED,   /*!< Node registered with gateway, event_data: uint8_t*(node_id) */
    APP_EVENT_ESPNOW_UNREGISTERED, /*!< Node lost gateway, reset to unregistered state */

    /* ── Sensor Events ── */
    APP_EVENT_SENSOR_DATA,         /*!< Sensor data ready to send, event_data: app_event_sensor_data_t* */
} app_event_id_t;

/**
 * @brief ESP-NOW registered event payload
 */
typedef struct {
    uint8_t node_id;                    /*!< Assigned node ID from gateway */
    uint8_t gateway_mac[6];             /*!< Gateway MAC address */
} app_event_espnow_registered_t;

/**
 * @brief Sensor data event payload
 *
 * Sensor tasks post this event when new data is ready.
 * main.c listens for it and calls app_espnow_send_data().
 */
typedef struct {
    uint8_t  sensor_type;  /*!< Sensor type (app_protocol_sensor_type_t) */
    uint16_t data_len;     /*!< Valid bytes in data[] */
    uint8_t  data[16];     /*!< Binary sensor payload (e.g. app_protocol_env_data_t = 16 bytes) */
} app_event_sensor_data_t;

/**
 * @brief Initialize the custom event loop
 * @return ESP_OK on success
 */
esp_err_t app_event_init(void);

/**
 * @brief Post an event (blocks until queue has space)
 */
esp_err_t app_event_post(app_event_id_t event_id, const void *event_data, size_t event_data_size);

/**
 * @brief Post an event with timeout
 * @param timeout  Ticks to wait; pass 0 for non-blocking
 */
esp_err_t app_event_post_with_timeout(app_event_id_t event_id, const void *event_data,
                                      size_t event_data_size, TickType_t timeout);

/**
 * @brief Register an event handler
 * @param event_id  Specific event ID or ESP_EVENT_ANY_ID
 */
esp_err_t app_event_handler_register(int32_t event_id, esp_event_handler_t event_handler,
                                     void *event_handler_arg);

/**
 * @brief Unregister an event handler
 */
esp_err_t app_event_handler_unregister(int32_t event_id, esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif
