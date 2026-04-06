#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────── Protocol Frame Constants ───────────────────────── */

/** Command frame header: 0xFD 0xFC 0xFB 0xFA */
#define LD2412_CMD_FRAME_HEADER_0 0xFD
#define LD2412_CMD_FRAME_HEADER_1 0xFC
#define LD2412_CMD_FRAME_HEADER_2 0xFB
#define LD2412_CMD_FRAME_HEADER_3 0xFA

/** Command frame tail: 0x04 0x03 0x02 0x01 */
#define LD2412_CMD_FRAME_TAIL_0 0x04
#define LD2412_CMD_FRAME_TAIL_1 0x03
#define LD2412_CMD_FRAME_TAIL_2 0x02
#define LD2412_CMD_FRAME_TAIL_3 0x01

/** Data frame header: 0xF4 0xF3 0xF2 0xF1 */
#define LD2412_DATA_FRAME_HEADER_0 0xF4
#define LD2412_DATA_FRAME_HEADER_1 0xF3
#define LD2412_DATA_FRAME_HEADER_2 0xF2
#define LD2412_DATA_FRAME_HEADER_3 0xF1

/** Data frame tail: 0xF8 0xF7 0xF6 0xF5 */
#define LD2412_DATA_FRAME_TAIL_0 0xF8
#define LD2412_DATA_FRAME_TAIL_1 0xF7
#define LD2412_DATA_FRAME_TAIL_2 0xF6
#define LD2412_DATA_FRAME_TAIL_3 0xF5

/* ───────────────────────── Command Words ───────────────────────── */

#define LD2412_CMD_ENABLE_CONFIG       0x00FF
#define LD2412_CMD_DISABLE_CONFIG      0x00FE
#define LD2412_CMD_SET_RESOLUTION      0x0001
#define LD2412_CMD_SET_BASIC_PARAM     0x0002
#define LD2412_CMD_SET_MOVING_SENS     0x0003
#define LD2412_CMD_SET_STATIC_SENS     0x0004
#define LD2412_CMD_ENTER_CALIBRATION    0x000B
#define LD2412_CMD_READ_RESOLUTION     0x0011
#define LD2412_CMD_READ_BASIC_PARAM    0x0012
#define LD2412_CMD_READ_MOVING_SENS    0x0013
#define LD2412_CMD_READ_STATIC_SENS    0x0014
#define LD2412_CMD_QUERY_CALIBRATION   0x001B
#define LD2412_CMD_ENABLE_ENGINEERING  0x0062
#define LD2412_CMD_DISABLE_ENGINEERING 0x0063
#define LD2412_CMD_READ_FW_VER         0x00A0
#define LD2412_CMD_SET_BAUD_RATE       0x00A1
#define LD2412_CMD_FACTORY_RESET       0x00A2
#define LD2412_CMD_RESTART             0x00A3
#define LD2412_CMD_SET_LIGHT           0x00A4
#define LD2412_CMD_READ_MAC            0x00A5
#define LD2412_CMD_SET_LIGHT_CTRL      0x000C
#define LD2412_CMD_READ_LIGHT_CTRL     0x001C

/** Number of distance gates (0-13) */
#define LD2412_GATE_COUNT 14

/* ───────────────────────── Data Frame Constants ───────────────────────── */

#define LD2412_DATA_HEAD 0xAA
#define LD2412_DATA_TAIL 0x55

/* ───────────────────────── Target State ───────────────────────── */

typedef enum {
    LD2412_TARGET_NONE              = 0x00,
    LD2412_TARGET_MOVING            = 0x01,
    LD2412_TARGET_STATIC            = 0x02,
    LD2412_TARGET_MOVING_AND_STATIC = 0x03,
    LD2412_TARGET_CALIBRATING       = 0x04,
    LD2412_TARGET_CALIBRATE_OK      = 0x05,
    LD2412_TARGET_CALIBRATE_FAIL    = 0x06,
} ld2412_target_state_t;

/* ───────────────────────── Data Structures ───────────────────────── */

typedef struct {
    ld2412_target_state_t target_state;
    uint16_t moving_distance;  /**< cm */
    uint8_t  moving_energy;    /**< 0-100 */
    uint16_t static_distance;  /**< cm */
    uint8_t  static_energy;    /**< 0-100 */
} ld2412_data_t;

typedef struct {
    uint16_t fw_type;       /**< e.g. 0x2412 */
    uint16_t minor_version;
    uint32_t major_version;
} ld2412_firmware_t;

/** Callback invoked from the UART task when a new data frame is received */
typedef void (*ld2412_data_cb_t)(const ld2412_data_t *data, void *user_ctx);

typedef struct {
    int uart_port;
    int tx_pin;
    int rx_pin;
    ld2412_data_cb_t data_cb;   /**< Optional data callback (called from UART task context) */
    void            *user_ctx;  /**< User context passed to data_cb */
} ld2412_config_t;

/* ───────────────────────── Public API ───────────────────────── */

esp_err_t ld2412_init(const ld2412_config_t *config);
esp_err_t ld2412_deinit(void);
esp_err_t ld2412_get_data(ld2412_data_t *out);

/**
 * Set basic parameters (cmd 0x0002).
 * @param min_gate  Minimum distance gate (1-13)
 * @param max_gate  Maximum distance gate (1-13, >= min_gate)
 * @param timeout_s No-one timeout in seconds (min 5)
 * @param out_pin   OUT pin config: 0 = active high, 1 = active low
 */
esp_err_t ld2412_set_basic_param(uint8_t min_gate, uint8_t max_gate,
                                 uint16_t timeout_s, uint8_t out_pin);

/**
 * Set moving sensitivity for all 14 gates at once (cmd 0x0003).
 * @param thresholds Array of 14 threshold values (gate 0-13)
 */
esp_err_t ld2412_set_moving_sensitivity(const uint8_t thresholds[LD2412_GATE_COUNT]);

/**
 * Set static sensitivity for all 14 gates at once (cmd 0x0004).
 * @param thresholds Array of 14 threshold values (gate 0-13)
 */
esp_err_t ld2412_set_static_sensitivity(const uint8_t thresholds[LD2412_GATE_COUNT]);

esp_err_t ld2412_read_firmware_version(ld2412_firmware_t *out);
esp_err_t ld2412_restart(void);
esp_err_t ld2412_factory_reset(void);

#ifdef __cplusplus
}
#endif
