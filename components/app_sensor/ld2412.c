#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "ld2412.h"

#define TAG "ld2412"

#define LD2412_BAUD_RATE 256000
#define LD2412_RX_BUF_SIZE 512
#define LD2412_TASK_STACK 3072
#define LD2412_TASK_PRIO 6
#define LD2412_MAX_FRAME 64
#define LD2412_CMD_TIMEOUT pdMS_TO_TICKS(1000)
#define LD2412_REPORT_INTERVAL pdMS_TO_TICKS(CONFIG_LD2412_REPORT_INTERVAL_MS)

/* ───────────────────────── Frame Parser States ───────────────────────── */

typedef enum
{
    PARSE_WAIT_HEADER,
    PARSE_READ_LENGTH,
    PARSE_READ_DATA,
    PARSE_VALIDATE_TAIL,
} parse_state_t;

typedef enum
{
    FRAME_TYPE_DATA,
    FRAME_TYPE_CMD,
} frame_type_t;

/* ───────────────────────── Module State ───────────────────────── */

static int s_uart_port = -1;
static TaskHandle_t s_task_handle = NULL;
static portMUX_TYPE s_data_lock = portMUX_INITIALIZER_UNLOCKED;
static ld2412_data_t s_latest_data;
static SemaphoreHandle_t s_ack_sem = NULL;
static SemaphoreHandle_t s_cmd_mutex = NULL;
static uint8_t s_ack_buf[LD2412_MAX_FRAME];
static uint16_t s_ack_len = 0;
static ld2412_data_cb_t s_data_cb = NULL;
static void *s_user_ctx = NULL;

/* ───────────────────────── Forward Declarations ───────────────────────── */

static void ld2412_uart_task(void *arg);
static void parse_data_frame(const uint8_t *data, uint16_t len);
static void parse_ack_frame(const uint8_t *data, uint16_t len);
static esp_err_t send_command(uint16_t cmd, const uint8_t *data, uint16_t data_len);
static esp_err_t enter_config_mode(void);
static esp_err_t exit_config_mode(void);

/* ───────────────────────── Init / Deinit ───────────────────────── */

esp_err_t ld2412_init(const ld2412_config_t *config)
{
    if (config == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_uart_port >= 0)
        return ESP_ERR_INVALID_STATE;

    const uart_config_t uart_cfg = {
        .baud_rate = LD2412_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;
    ret = uart_driver_install(config->uart_port, LD2412_RX_BUF_SIZE, 0, 0, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "uart_driver_install failed");

    ret = uart_param_config(config->uart_port, &uart_cfg);
    if (ret != ESP_OK)
    {
        uart_driver_delete(config->uart_port);
        return ret;
    }

    ret = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        uart_driver_delete(config->uart_port);
        return ret;
    }

    s_uart_port = config->uart_port;
    memset(&s_latest_data, 0, sizeof(s_latest_data));
    s_data_cb = config->data_cb;
    s_user_ctx = config->user_ctx;

    s_ack_sem = xSemaphoreCreateBinary();
    if (s_ack_sem == NULL)
    {
        uart_driver_delete(s_uart_port);
        s_uart_port = -1;
        return ESP_ERR_NO_MEM;
    }

    s_cmd_mutex = xSemaphoreCreateMutex();
    if (s_cmd_mutex == NULL)
    {
        vSemaphoreDelete(s_ack_sem);
        s_ack_sem = NULL;
        uart_driver_delete(s_uart_port);
        s_uart_port = -1;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreate(ld2412_uart_task, "ld2412", LD2412_TASK_STACK,
                                NULL, LD2412_TASK_PRIO, &s_task_handle);
    if (ok != pdPASS)
    {
        vSemaphoreDelete(s_cmd_mutex);
        s_cmd_mutex = NULL;
        vSemaphoreDelete(s_ack_sem);
        s_ack_sem = NULL;
        uart_driver_delete(s_uart_port);
        s_uart_port = -1;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initialized on UART%d (TX=%d, RX=%d)",
             config->uart_port, config->tx_pin, config->rx_pin);
    return ESP_OK;
}

esp_err_t ld2412_deinit(void)
{
    if (s_uart_port < 0)
        return ESP_ERR_INVALID_STATE;

    if (s_task_handle)
    {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }
    if (s_ack_sem)
    {
        vSemaphoreDelete(s_ack_sem);
        s_ack_sem = NULL;
    }
    if (s_cmd_mutex)
    {
        vSemaphoreDelete(s_cmd_mutex);
        s_cmd_mutex = NULL;
    }
    uart_driver_delete(s_uart_port);
    s_uart_port = -1;
    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

/* ───────────────────────── Data Access ───────────────────────── */

esp_err_t ld2412_get_data(ld2412_data_t *out)
{
    if (out == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_uart_port < 0)
        return ESP_ERR_INVALID_STATE;

    portENTER_CRITICAL(&s_data_lock);
    *out = s_latest_data;
    portEXIT_CRITICAL(&s_data_lock);
    return ESP_OK;
}

/* ───────────────────────── Frame Parsers ───────────────────────── */

static void parse_data_frame(const uint8_t *data, uint16_t len)
{
    /*
     * Basic mode frame content (11 bytes):
     *   [0]   data type (0x02 = basic)
     *   [1]   head (0xAA)
     *   [2]   target state
     *   [3-4] moving distance (LE, cm)
     *   [5]   moving energy
     *   [6-7] static distance (LE, cm)
     *   [8]   static energy
     *   [9]   tail (0x55)
     *   [10]  check byte
     */
    if (len < 11)
        return;
    if (data[1] != LD2412_DATA_HEAD)
        return;

    /* Find tail position — basic mode at [9], engineering mode further */
    uint16_t tail_pos = 9;
    if (data[0] == 0x01)
    {
        /* Engineering mode: target data + gate energies + light, tail is at len-2 */
        tail_pos = len - 2;
    }
    if (data[tail_pos] != LD2412_DATA_TAIL)
        return;

    ld2412_data_t d;
    d.target_state = (ld2412_target_state_t)data[2];
    d.moving_distance = (uint16_t)(data[3] | (data[4] << 8));
    d.moving_energy = data[5];
    d.static_distance = (uint16_t)(data[6] | (data[7] << 8));
    d.static_energy = data[8];

    portENTER_CRITICAL(&s_data_lock);
    s_latest_data = d;
    portEXIT_CRITICAL(&s_data_lock);

    ESP_LOGD(TAG, "state=%d mov=%ucm/%u%% sta=%ucm/%u%%",
             d.target_state, d.moving_distance, d.moving_energy,
             d.static_distance, d.static_energy);

    /* Throttle callback to configured minimum interval */
    static TickType_t s_last_report_tick = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - s_last_report_tick) < LD2412_REPORT_INTERVAL)
        return;
    s_last_report_tick = now;

    if (s_data_cb)
    {
        s_data_cb(&d, s_user_ctx);
    }
}

static void parse_ack_frame(const uint8_t *data, uint16_t len)
{
    /* ACK frame: cmd_echo(2) + status(2) + [return_data...] */
    if (len < 4)
        return;

    /* Store full ACK payload for callers that need return data */
    s_ack_len = len;
    memcpy(s_ack_buf, data, (len <= LD2412_MAX_FRAME) ? len : LD2412_MAX_FRAME);

    if (s_ack_sem)
        xSemaphoreGive(s_ack_sem);
}

/* ───────────────────────── UART Receive Task ───────────────────────── */

static void ld2412_uart_task(void *arg)
{
    parse_state_t state = PARSE_WAIT_HEADER;
    frame_type_t frame_type = FRAME_TYPE_DATA;
    uint8_t frame_buf[LD2412_MAX_FRAME];
    uint16_t frame_len = 0;
    uint16_t data_len = 0;
    uint8_t len_buf[2];
    uint8_t len_idx = 0;
    uint8_t tail_idx = 0;

    const uint8_t data_header[] = {
        LD2412_DATA_FRAME_HEADER_0, LD2412_DATA_FRAME_HEADER_1,
        LD2412_DATA_FRAME_HEADER_2, LD2412_DATA_FRAME_HEADER_3};
    const uint8_t cmd_header[] = {
        LD2412_CMD_FRAME_HEADER_0, LD2412_CMD_FRAME_HEADER_1,
        LD2412_CMD_FRAME_HEADER_2, LD2412_CMD_FRAME_HEADER_3};
    const uint8_t data_tail[] = {
        LD2412_DATA_FRAME_TAIL_0, LD2412_DATA_FRAME_TAIL_1,
        LD2412_DATA_FRAME_TAIL_2, LD2412_DATA_FRAME_TAIL_3};
    const uint8_t cmd_tail[] = {
        LD2412_CMD_FRAME_TAIL_0, LD2412_CMD_FRAME_TAIL_1,
        LD2412_CMD_FRAME_TAIL_2, LD2412_CMD_FRAME_TAIL_3};

    uint8_t rx_byte;
    uint8_t data_hdr_idx = 0;
    uint8_t cmd_hdr_idx = 0;

    ESP_LOGI(TAG, "UART task started");

    while (1)
    {
        int n = uart_read_bytes(s_uart_port, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0)
            continue;

        switch (state)
        {
        case PARSE_WAIT_HEADER:
            if (rx_byte == data_header[data_hdr_idx])
            {
                data_hdr_idx++;
            }
            else
            {
                data_hdr_idx = (rx_byte == data_header[0]) ? 1 : 0;
            }
            if (rx_byte == cmd_header[cmd_hdr_idx])
            {
                cmd_hdr_idx++;
            }
            else
            {
                cmd_hdr_idx = (rx_byte == cmd_header[0]) ? 1 : 0;
            }

            if (data_hdr_idx == 4)
            {
                frame_type = FRAME_TYPE_DATA;
                state = PARSE_READ_LENGTH;
                data_hdr_idx = 0;
                cmd_hdr_idx = 0;
                len_idx = 0;
            }
            else if (cmd_hdr_idx == 4)
            {
                frame_type = FRAME_TYPE_CMD;
                state = PARSE_READ_LENGTH;
                data_hdr_idx = 0;
                cmd_hdr_idx = 0;
                len_idx = 0;
            }
            break;

        case PARSE_READ_LENGTH:
            len_buf[len_idx++] = rx_byte;
            if (len_idx == 2)
            {
                data_len = (uint16_t)(len_buf[0] | (len_buf[1] << 8));
                if (data_len > LD2412_MAX_FRAME)
                {
                    state = PARSE_WAIT_HEADER;
                }
                else
                {
                    frame_len = 0;
                    state = PARSE_READ_DATA;
                }
            }
            break;

        case PARSE_READ_DATA:
            frame_buf[frame_len++] = rx_byte;
            if (frame_len >= data_len)
            {
                tail_idx = 0;
                state = PARSE_VALIDATE_TAIL;
            }
            break;

        case PARSE_VALIDATE_TAIL:
        {
            const uint8_t *expected_tail = (frame_type == FRAME_TYPE_DATA) ? data_tail : cmd_tail;
            if (rx_byte == expected_tail[tail_idx])
            {
                tail_idx++;
                if (tail_idx == 4)
                {
                    if (frame_type == FRAME_TYPE_DATA)
                        parse_data_frame(frame_buf, data_len);
                    else
                        parse_ack_frame(frame_buf, data_len);
                    state = PARSE_WAIT_HEADER;
                }
            }
            else
            {
                state = PARSE_WAIT_HEADER;
            }
            break;
        }
        } /* end switch */
    } /* end while(1) */
}

/* ───────────────────────── Command Sending ───────────────────────── */

static esp_err_t send_command(uint16_t cmd, const uint8_t *data, uint16_t data_len)
{
    if (s_uart_port < 0)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_cmd_mutex, LD2412_CMD_TIMEOUT) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    uint16_t payload_len = 2 + data_len;           /* cmd(2) + data */
    uint8_t buf[4 + 2 + 2 + LD2412_MAX_FRAME + 4]; /* header + len + cmd + data + tail */
    uint8_t pos = 0;

    /* Header */
    buf[pos++] = LD2412_CMD_FRAME_HEADER_0;
    buf[pos++] = LD2412_CMD_FRAME_HEADER_1;
    buf[pos++] = LD2412_CMD_FRAME_HEADER_2;
    buf[pos++] = LD2412_CMD_FRAME_HEADER_3;

    /* Length (little-endian) */
    buf[pos++] = (uint8_t)(payload_len & 0xFF);
    buf[pos++] = (uint8_t)((payload_len >> 8) & 0xFF);

    /* Command word (little-endian) */
    buf[pos++] = (uint8_t)(cmd & 0xFF);
    buf[pos++] = (uint8_t)((cmd >> 8) & 0xFF);

    /* Data */
    if (data && data_len > 0)
    {
        memcpy(&buf[pos], data, data_len);
        pos += data_len;
    }

    /* Tail */
    buf[pos++] = LD2412_CMD_FRAME_TAIL_0;
    buf[pos++] = LD2412_CMD_FRAME_TAIL_1;
    buf[pos++] = LD2412_CMD_FRAME_TAIL_2;
    buf[pos++] = LD2412_CMD_FRAME_TAIL_3;

    /* Clear any pending ack */
    xSemaphoreTake(s_ack_sem, 0);

    esp_err_t ret = ESP_OK;
    int written = uart_write_bytes(s_uart_port, buf, pos);
    if (written < 0)
    {
        ret = ESP_FAIL;
        goto out;
    }

    /* Wait for ACK */
    if (xSemaphoreTake(s_ack_sem, LD2412_CMD_TIMEOUT) != pdTRUE)
    {
        ESP_LOGW(TAG, "Command 0x%04X timeout", cmd);
        ret = ESP_ERR_TIMEOUT;
        goto out;
    }

    /* ACK status is at bytes [2..3] (little-endian), 0 = success */
    if (s_ack_len < 4 || s_ack_buf[2] != 0)
    {
        ESP_LOGW(TAG, "Command 0x%04X failed, status=%d", cmd,
                 (s_ack_len >= 4) ? s_ack_buf[2] : -1);
        ret = ESP_FAIL;
    }

out:
    xSemaphoreGive(s_cmd_mutex);
    return ret;
}

static esp_err_t enter_config_mode(void)
{
    const uint8_t val[] = {0x01, 0x00};
    return send_command(LD2412_CMD_ENABLE_CONFIG, val, sizeof(val));
}

static esp_err_t exit_config_mode(void)
{
    return send_command(LD2412_CMD_DISABLE_CONFIG, NULL, 0);
}

/* ───────────────────────── Public Configuration API ───────────────────────── */

esp_err_t ld2412_set_basic_param(uint8_t min_gate, uint8_t max_gate,
                                 uint16_t timeout_s, uint8_t out_pin)
{
    esp_err_t ret = enter_config_mode();
    if (ret != ESP_OK)
        return ret;

    /* cmd 0x0002: min_gate(1) + max_gate(1) + timeout(2 LE) + out_pin(1) = 5 bytes */
    uint8_t data[] = {
        min_gate,
        max_gate,
        (uint8_t)(timeout_s & 0xFF), (uint8_t)((timeout_s >> 8) & 0xFF),
        out_pin};
    ret = send_command(LD2412_CMD_SET_BASIC_PARAM, data, sizeof(data));

    exit_config_mode();
    return ret;
}

esp_err_t ld2412_set_moving_sensitivity(const uint8_t thresholds[LD2412_GATE_COUNT])
{
    if (thresholds == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = enter_config_mode();
    if (ret != ESP_OK)
        return ret;

    /* cmd 0x0003: 14 bytes, one per gate */
    ret = send_command(LD2412_CMD_SET_MOVING_SENS, thresholds, LD2412_GATE_COUNT);

    exit_config_mode();
    return ret;
}

esp_err_t ld2412_set_static_sensitivity(const uint8_t thresholds[LD2412_GATE_COUNT])
{
    if (thresholds == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = enter_config_mode();
    if (ret != ESP_OK)
        return ret;

    /* cmd 0x0004: 14 bytes, one per gate */
    ret = send_command(LD2412_CMD_SET_STATIC_SENS, thresholds, LD2412_GATE_COUNT);

    exit_config_mode();
    return ret;
}

esp_err_t ld2412_read_firmware_version(ld2412_firmware_t *out)
{
    esp_err_t ret = enter_config_mode();
    if (ret != ESP_OK)
        return ret;

    ret = send_command(LD2412_CMD_READ_FW_VER, NULL, 0);
    if (ret == ESP_OK && out != NULL)
    {
        /* ACK payload: cmd_echo(2) + status(2) + fw_type(2) + minor(2) + major(4) */
        if (s_ack_len >= 12)
        {
            out->fw_type = (uint16_t)(s_ack_buf[4] | (s_ack_buf[5] << 8));
            out->minor_version = (uint16_t)(s_ack_buf[6] | (s_ack_buf[7] << 8));
            out->major_version = (uint32_t)(s_ack_buf[8] | (s_ack_buf[9] << 8) |
                                            (s_ack_buf[10] << 16) | (s_ack_buf[11] << 24));
        }
    }

    exit_config_mode();
    return ret;
}

esp_err_t ld2412_restart(void)
{
    esp_err_t ret = enter_config_mode();
    if (ret != ESP_OK)
        return ret;

    ret = send_command(LD2412_CMD_RESTART, NULL, 0);
    /* No exit_config_mode needed — module is restarting */
    return ret;
}

esp_err_t ld2412_factory_reset(void)
{
    esp_err_t ret = enter_config_mode();
    if (ret != ESP_OK)
        return ret;

    ret = send_command(LD2412_CMD_FACTORY_RESET, NULL, 0);

    exit_config_mode();
    return ret;
}
