#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize ESP-NOW and start the internal espnow_task
 *
 * The task handles registration, heartbeat, and send-failure tracking
 * autonomously. No polling from main is required.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_espnow_init(void);

/**
 * @brief Send Data Report to the Gateway
 *
 * Thread-safe — may be called from any task.
 *
 * @param sensor_type Sensor type identifier (app_protocol_sensor_type_t)
 * @param data        Pointer to the binary payload
 * @param len         Length of the payload in bytes
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_espnow_send_data(uint8_t sensor_type, const uint8_t *data, size_t len);

/**
 * @brief Check if the node is currently registered with a gateway
 *
 * @return true if registered, false otherwise
 */
bool app_espnow_is_registered(void);

#ifdef __cplusplus
}
#endif
