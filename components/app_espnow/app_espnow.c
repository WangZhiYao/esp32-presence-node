#include <string.h>
#include <stddef.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "app_espnow.h"
#include "app_storage.h"
#include "app_protocol.h"
#include "app_event.h"

#define TAG "app_espnow"

#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#define RX_QUEUE_DEPTH 8
#define TX_QUEUE_DEPTH 4
#define TASK_STACK_SIZE 4096
#define TASK_PRIORITY 5
#define QUEUE_TICK_TIMEOUT_MS 500
#define REGISTER_INTERVAL_MS 5000
#define HEARTBEAT_INTERVAL_MS 25000
#define MAX_CONSECUTIVE_FAILURES 10
#define MAX_WIFI_CHANNEL 13

/* Storage Keys */
#define NVS_NAMESPACE "node_cfg"
#define NVS_KEY_NODE_ID "node_id"
#define NVS_KEY_GATEWAY_MAC "gw_mac"
#define NVS_KEY_CHANNEL "channel"

/* Default Broadcast MAC Address */
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ── RX queue item ── */
typedef struct
{
    uint8_t src_addr[ESP_NOW_ETH_ALEN];
    int data_len;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} rx_item_t;

/* ── TX queue item (for app_espnow_send_data) ── */
typedef struct
{
    uint16_t frame_len;
    uint8_t frame[ESP_NOW_MAX_DATA_LEN];
} tx_item_t;

/* ── State owned exclusively by espnow_task ── */
static TaskHandle_t s_task_handle = NULL;
static QueueHandle_t s_rx_queue = NULL;
static QueueHandle_t s_tx_queue = NULL;

static uint8_t s_node_id = 0;
static uint8_t s_gateway_mac[ESP_NOW_ETH_ALEN] = {0};
static uint8_t s_channel = 0;
static int s_consecutive_failures = 0;
static bool s_heartbeat_pending = false;
static uint8_t s_scan_channel = 1;

/* Shared atomics */
static atomic_bool s_registered = false;
static atomic_uint_fast16_t s_seq_num = 0;

/* ── Helpers ── */

static void do_reset_state(void)
{
    s_node_id = 0;
    memset(s_gateway_mac, 0, ESP_NOW_ETH_ALEN);
    s_channel = 0;
    s_consecutive_failures = 0;
    s_scan_channel = 1;
    atomic_store(&s_registered, false);

    /* Flush TX queue to prevent sending stale data after reset */
    if (s_tx_queue)
    {
        tx_item_t dummy;
        while (xQueueReceive(s_tx_queue, &dummy, 0) == pdTRUE)
        {
            /* discard */
        }
    }
}

static void do_reset_state_with_nvs(void)
{
    if (esp_now_is_peer_exist(s_gateway_mac))
        esp_now_del_peer(s_gateway_mac);
    do_reset_state();

    esp_err_t err;
    err = app_storage_erase_key(NVS_NAMESPACE, NVS_KEY_NODE_ID);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to erase %s: %s", NVS_KEY_NODE_ID, esp_err_to_name(err));
    }

    err = app_storage_erase_key(NVS_NAMESPACE, NVS_KEY_GATEWAY_MAC);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to erase %s: %s", NVS_KEY_GATEWAY_MAC, esp_err_to_name(err));
    }

    err = app_storage_erase_key(NVS_NAMESPACE, NVS_KEY_CHANNEL);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to erase %s: %s", NVS_KEY_CHANNEL, esp_err_to_name(err));
    }
}

static esp_err_t espnow_send(const void *data, size_t len)
{
    if (atomic_load(&s_registered))
    {
        return esp_now_send(s_gateway_mac, (const uint8_t *)data, len);
    }

    return esp_now_send(s_broadcast_mac, (const uint8_t *)data, len);
}

/* ── Callbacks (WiFi task context — minimal work only) ── */

static void app_espnow_send_cb(const esp_now_send_info_t *tx_info,
                               esp_now_send_status_t status)
{
    if (tx_info == NULL)
    {
        return;
    }

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "Send to " MACSTR " OK", MAC2STR(tx_info->des_addr));
    }
    else
    {
        ESP_LOGW(TAG, "Send to " MACSTR " failed", MAC2STR(tx_info->des_addr));
    }
}

static void app_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                               const uint8_t *data, int len)
{
    if (s_rx_queue == NULL)
        return;

    if (len < (int)sizeof(app_protocol_header_t) || len > ESP_NOW_MAX_DATA_LEN)
        return;

    rx_item_t item;
    memcpy(item.src_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    item.data_len = len;
    memcpy(item.data, data, len);

    /* Non-blocking: drop if queue full */
    if (xQueueSend(s_rx_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "RX queue full, packet dropped");
    }
}

/* ── Packet handling (runs in espnow_task) ── */

static void handle_rx_packet(const rx_item_t *item)
{
    const app_protocol_header_t *header = (const app_protocol_header_t *)item->data;

    switch (header->type)
    {
    case APP_PROTOCOL_MSG_REGISTER_RESP:
    {
        if (item->data_len < (int)sizeof(app_protocol_register_resp_t))
            break;
        const app_protocol_register_resp_t *resp =
            (const app_protocol_register_resp_t *)item->data;

        ESP_LOGI(TAG, "REGISTER_RESP: assigned_id=%d channel=%d gateway=" MACSTR,
                 resp->assigned_id, resp->channel, MAC2STR(item->src_addr));

        if (resp->assigned_id == 0)
            break;

        /* 1. Switch to the channel assigned by gateway */
        esp_err_t ch_err = esp_wifi_set_channel(resp->channel, WIFI_SECOND_CHAN_NONE);
        if (ch_err != ESP_OK)
        {
            ESP_LOGE(TAG, "set_channel(%d) failed: %s",
                     resp->channel, esp_err_to_name(ch_err));
            break;
        }

        /* Sanity check: ensure gateway is not broadcast */
        if (memcmp(item->src_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)
        {
            ESP_LOGW(TAG, "Ignored broadcast address as gateway");
            break;
        }

        /* 2. Add gateway peer */
        if (!esp_now_is_peer_exist(item->src_addr))
        {
            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, item->src_addr, ESP_NOW_ETH_ALEN);
            peer.channel = 0;
            peer.ifidx = ESPNOW_WIFI_IF;
            peer.encrypt = false;
            esp_err_t e = esp_now_add_peer(&peer);
            if (e != ESP_OK)
            {
                ESP_LOGE(TAG, "add_peer failed: %s", esp_err_to_name(e));
                break;
            }
        }

        /* 3. Save to NVS */
        app_storage_set_u8(NVS_NAMESPACE, NVS_KEY_NODE_ID, resp->assigned_id);
        app_storage_set_blob(NVS_NAMESPACE, NVS_KEY_GATEWAY_MAC,
                             item->src_addr, ESP_NOW_ETH_ALEN);
        app_storage_set_u8(NVS_NAMESPACE, NVS_KEY_CHANNEL, resp->channel);

        /* 4. Update state */
        s_node_id = resp->assigned_id;
        memcpy(s_gateway_mac, item->src_addr, ESP_NOW_ETH_ALEN);
        s_channel = resp->channel;
        s_consecutive_failures = 0;
        atomic_store(&s_registered, true);

        /* 5. Notify app */
        app_event_espnow_registered_t evt = {.node_id = s_node_id};
        memcpy(evt.gateway_mac, s_gateway_mac, sizeof(evt.gateway_mac));
        app_event_post(APP_EVENT_ESPNOW_REGISTERED, &evt, sizeof(evt));
        break;
    }
    case APP_PROTOCOL_MSG_HEARTBEAT_ACK:
        s_heartbeat_pending = false;
        s_consecutive_failures = 0;
        ESP_LOGD(TAG, "Heartbeat ACK");
        break;

    default:
        break;
    }
}

/* ── Internal send helpers (called from espnow_task) ── */

static void send_register_req(void)
{
    /* Rotate through WiFi channels so we can discover the gateway */
    esp_err_t ch_err = esp_wifi_set_channel(s_scan_channel, WIFI_SECOND_CHAN_NONE);
    if (ch_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to set channel %d: %s, skipping", s_scan_channel, esp_err_to_name(ch_err));
        s_scan_channel = (s_scan_channel % MAX_WIFI_CHANNEL) + 1;
        return;
    }
    ESP_LOGI(TAG, "Scanning channel %d for gateway...", s_scan_channel);
    s_scan_channel = (s_scan_channel % MAX_WIFI_CHANNEL) + 1;

    app_protocol_register_req_t req = {0};
    req.header.type = APP_PROTOCOL_MSG_REGISTER_REQ;
    req.header.node_id = 0;
    req.header.seq = (uint16_t)atomic_fetch_add(&s_seq_num, 1);
    req.device_type = 0x01;
    req.fw_version = 0x01;

    espnow_send(&req, sizeof(req));
}

static void send_heartbeat(void)
{
    /* If previous heartbeat got no ACK, count a failure */
    if (s_heartbeat_pending)
    {
        s_consecutive_failures++;
        ESP_LOGW(TAG, "No ACK for previous heartbeat (%d/%d)",
                 s_consecutive_failures, MAX_CONSECUTIVE_FAILURES);
        if (s_consecutive_failures >= MAX_CONSECUTIVE_FAILURES)
        {
            ESP_LOGE(TAG, "Gateway unreachable after %d failures, resetting",
                     MAX_CONSECUTIVE_FAILURES);
            do_reset_state_with_nvs();
            app_event_post(APP_EVENT_ESPNOW_UNREGISTERED, NULL, 0);
            return;
        }
    }

    app_protocol_heartbeat_t hb = {0};
    hb.header.type = APP_PROTOCOL_MSG_HEARTBEAT;
    hb.header.node_id = s_node_id;
    hb.header.seq = (uint16_t)atomic_fetch_add(&s_seq_num, 1);

    espnow_send(&hb, sizeof(hb));
    s_heartbeat_pending = true;
    ESP_LOGI(TAG, "Heartbeat queued (seq=%d)", hb.header.seq);
}

/* ── Main espnow_task ── */

static void espnow_task(void *arg)
{
    int64_t last_register_us = 0;
    int64_t last_heartbeat_us = 0;

    /* If restored from NVS, send heartbeat immediately to verify gateway */
    if (atomic_load(&s_registered))
    {
        send_heartbeat();
        last_heartbeat_us = esp_timer_get_time();
    }

    while (1)
    {
        /* 1. Process incoming packets (500ms timeout doubles as tick) */
        rx_item_t item;
        if (xQueueReceive(s_rx_queue, &item,
                          pdMS_TO_TICKS(QUEUE_TICK_TIMEOUT_MS)) == pdTRUE)
            handle_rx_packet(&item);

        /* 2. Drain TX queue (data reports from other tasks) */
        if (atomic_load(&s_registered))
        {
            tx_item_t tx;
            while (xQueueReceive(s_tx_queue, &tx, 0) == pdTRUE)
            {
                esp_err_t err = esp_now_send(s_gateway_mac, tx.frame, tx.frame_len);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Data send failed: %s", esp_err_to_name(err));
                }
                else
                {
                    ESP_LOGI(TAG, "Data sent (%d bytes)", tx.frame_len);
                }
            }
        }

        /* 3. Timed sends */
        int64_t now_us = esp_timer_get_time();

        if (!atomic_load(&s_registered))
        {
            if ((now_us - last_register_us) >= (REGISTER_INTERVAL_MS * 1000LL))
            {
                send_register_req();
                last_register_us = now_us;
            }
        }
        else
        {
            if ((now_us - last_heartbeat_us) >= (HEARTBEAT_INTERVAL_MS * 1000LL))
            {
                send_heartbeat();
                last_heartbeat_us = now_us;
            }
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════════════ */

esp_err_t app_espnow_init(void)
{
    esp_err_t err;

    /* 1. Initialize ESP-NOW */
    err = esp_now_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. Create RX queue before registering callbacks */
    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(rx_item_t));
    if (s_rx_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create RX queue");
        esp_now_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_item_t));
    if (s_tx_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create TX queue");
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
        esp_now_deinit();
        return ESP_ERR_NO_MEM;
    }

    /* 3. Register callbacks */
    err = esp_now_register_recv_cb(app_espnow_recv_cb);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Register recv cb failed: %s", esp_err_to_name(err));
        esp_now_deinit();
        return err;
    }

    err = esp_now_register_send_cb(app_espnow_send_cb);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Register send cb failed: %s", esp_err_to_name(err));
        esp_now_deinit();
        return err;
    }

    /* 4. Add broadcast peer */
    if (!esp_now_is_peer_exist(s_broadcast_mac))
    {
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
        peer.channel = 0;
        peer.ifidx = ESPNOW_WIFI_IF;
        peer.encrypt = false;
        err = esp_now_add_peer(&peer);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add broadcast peer: %s",
                     esp_err_to_name(err));
            esp_now_deinit();
            return err;
        }
    }

    /* 5. Restore state from NVS */
    uint8_t saved_id = 0;
    uint8_t saved_mac[ESP_NOW_ETH_ALEN] = {0};
    uint8_t saved_channel = 0;
    size_t mac_len = sizeof(saved_mac);

    esp_err_t err_id = app_storage_get_u8(NVS_NAMESPACE, NVS_KEY_NODE_ID,
                                          &saved_id);
    esp_err_t err_mac = app_storage_get_blob(NVS_NAMESPACE, NVS_KEY_GATEWAY_MAC,
                                             saved_mac, &mac_len);
    esp_err_t err_ch = app_storage_get_u8(NVS_NAMESPACE, NVS_KEY_CHANNEL,
                                          &saved_channel);

    if (err_id == ESP_OK && err_mac == ESP_OK && err_ch == ESP_OK && saved_id != 0 && saved_channel != 0)
    {
        esp_wifi_set_channel(saved_channel, WIFI_SECOND_CHAN_NONE);

        s_node_id = saved_id;
        memcpy(s_gateway_mac, saved_mac, ESP_NOW_ETH_ALEN);
        s_channel = saved_channel;

        if (!esp_now_is_peer_exist(s_gateway_mac))
        {
            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, s_gateway_mac, ESP_NOW_ETH_ALEN);
            peer.channel = 0;
            peer.ifidx = ESPNOW_WIFI_IF;
            peer.encrypt = false;
            esp_now_add_peer(&peer);
        }

        atomic_store(&s_registered, true);
        ESP_LOGI(TAG, "Restored: node_id=%d channel=%d gateway=" MACSTR,
                 s_node_id, s_channel, MAC2STR(s_gateway_mac));
    }
    else
    {
        ESP_LOGW(TAG, "No valid state in NVS, starting unregistered");
        do_reset_state();
    }

    /* 6. Create task */
    BaseType_t ret = xTaskCreate(espnow_task, "espnow_task",
                                 TASK_STACK_SIZE, NULL,
                                 TASK_PRIORITY, &s_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create espnow_task");
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
        esp_now_deinit();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t app_espnow_send_data(uint8_t sensor_type, const uint8_t *data, size_t len)
{
    if (!atomic_load(&s_registered))
    {
        ESP_LOGW(TAG, "Cannot send data: not registered");
        return ESP_ERR_INVALID_STATE;
    }

    if (len > APP_PROTOCOL_USER_DATA_MAX_LEN)
    {
        ESP_LOGE(TAG, "Data too long: %d (max %d)",
                 (int)len, APP_PROTOCOL_USER_DATA_MAX_LEN);
        return ESP_ERR_INVALID_SIZE;
    }

    tx_item_t tx = {0};
    app_protocol_data_report_t *report = (app_protocol_data_report_t *)tx.frame;
    report->header.type    = APP_PROTOCOL_MSG_DATA_REPORT;
    report->header.node_id = s_node_id;
    report->header.seq     = (uint16_t)atomic_fetch_add(&s_seq_num, 1);
    report->sensor_type    = sensor_type;
    report->data_len       = (uint16_t)len;
    memcpy(report->data, data, len);
    tx.frame_len = (uint16_t)(offsetof(app_protocol_data_report_t, data) + len);

    if (xQueueSend(s_tx_queue, &tx, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "TX queue full, data report dropped");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool app_espnow_is_registered(void)
{
    return atomic_load(&s_registered);
}
