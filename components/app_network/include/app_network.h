#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WiFi driver in STA mode (no AP connection)
 *
 * Only starts the WiFi driver — no SSID/password needed.
 * ESP-NOW uses the WiFi radio but does not associate with any AP.
 *
 * Dependencies:
 *  - NVS must be initialized before calling this.
 *
 * @return ESP_OK on success
 */
esp_err_t app_network_init(void);

/**
 * @brief Deinitialize WiFi driver
 *
 * @return ESP_OK on success
 */
esp_err_t app_network_deinit(void);

#ifdef __cplusplus
}
#endif
