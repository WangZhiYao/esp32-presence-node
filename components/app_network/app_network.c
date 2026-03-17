#include <stdbool.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "app_network.h"

#define TAG "app_network"

// Tracks if the network stack has been initialized.
static bool s_initialized = false;

esp_err_t app_network_init(void)
{
    if (s_initialized)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    esp_err_t err;

    // 1. Initialize the underlying TCP/IP stack.
    err = esp_netif_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // 2. Initialize the system event loop.
    // ESP_ERR_INVALID_STATE means it's already been initialized, which is acceptable.
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Create the default network interface for WiFi station mode.
    esp_netif_create_default_wifi_sta();

    // 4. Initialize the WiFi driver with default configuration.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // 5. Configure WiFi storage. Setting to RAM means no configuration is persisted.
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_storage failed: %s", esp_err_to_name(err));
        esp_wifi_deinit();
        return err;
    }

    // 6. Set the WiFi mode to Station. ESP-NOW requires STA or AP-STA mode.
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
        esp_wifi_deinit();
        return err;
    }

    // 7. Start the WiFi driver.
    err = esp_wifi_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        esp_wifi_deinit();
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "WiFi STA started (ESP-NOW mode, no AP connection)");
    return ESP_OK;
}

esp_err_t app_network_deinit(void)
{
    if (!s_initialized)
    {
        ESP_LOGW(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_wifi_stop();
    esp_wifi_deinit();
    // Note: netif and event loop are not de-initialized here as they might be
    // used by other components. A more robust deinit would use reference counting.
    // For this simple case, we just deinit the WiFi part.
    s_initialized = false;
    ESP_LOGI(TAG, "WiFi deinitialized");
    return ESP_OK;
}
