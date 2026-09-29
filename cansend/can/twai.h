/**
 * twai.h  –  ESP-IDF style TWAI (CAN) driver for the STM32 bxCAN peripheral
 *
 * Same API as ESP-IDF components/driver/twai/include/driver/twai.h
 * (twai_driver_install, twai_start, twai_transmit, twai_receive,
 * twai_read_alerts, twai_initiate_recovery, ...) implemented on CAN1 of
 * the STM32F2/F4 with direct register access (no HAL CAN, no STM32_CAN).
 *
 * Differences from ESP-IDF:
 *   - Timeouts are in milliseconds (pdMS_TO_TICKS(ms) is defined as ms).
 *   - tx_io / rx_io are Arduino pin names (PD1 / PD0, PB9 / PB8, PA12 / PA11).
 *   - twai_timing_config_t holds a bitrate; BRP / TSEG are computed from PCLK1.
 *   - Filter code/mask use the bxCAN register layout (std ID << 21,
 *     ext ID << 3 | IDE); a mask bit of 1 means "don't care", as on ESP32.
 *   - TWAI_MODE_NO_ACK maps to bxCAN loopback (frames go out on the bus and
 *     are received back without needing an ACK). TWAI_MODE_LOOPBACK is
 *     internal loopback + silent (nothing reaches the bus).
 *   - The per-message ss / self flags are ignored.
 *
 * Bus-off recovery works like ESP-IDF: on TWAI_ALERT_BUS_OFF call
 * twai_initiate_recovery(); on TWAI_ALERT_BUS_RECOVERED call twai_start().
 */

#pragma once

#include <Arduino.h>

// ─── ESP-IDF error codes ─────────────────────────────────────────────────────
#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_NOT_SUPPORTED  0x106
#define ESP_ERR_TIMEOUT        0x107
const char *esp_err_to_name(esp_err_t err);
#endif

#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(ms) (ms)
#endif

#define TWAI_WAIT_FOREVER 0xFFFFFFFFUL

// ─── Frame ───────────────────────────────────────────────────────────────────
#define TWAI_FRAME_MAX_DLC      8
#define TWAI_STD_ID_MASK        0x7FF
#define TWAI_EXTD_ID_MASK       0x1FFFFFFF

typedef struct {
  union {
    struct {
      uint32_t extd: 1;          // 1 = 29-bit extended ID
      uint32_t rtr: 1;           // 1 = remote frame
      uint32_t ss: 1;            // ignored on STM32
      uint32_t self: 1;          // ignored on STM32
      uint32_t dlc_non_comp: 1;  // allow DLC > 8
      uint32_t reserved: 27;
    };
    uint32_t flags;
  };
  uint32_t identifier;
  uint8_t  data_length_code;
  uint8_t  data[TWAI_FRAME_MAX_DLC];
} twai_message_t;

// ─── Modes / states ──────────────────────────────────────────────────────────
typedef enum {
  TWAI_MODE_NORMAL,       // normal TX / RX / ACK
  TWAI_MODE_NO_ACK,       // bxCAN loopback: TX succeeds without an ACK
  TWAI_MODE_LISTEN_ONLY,  // bxCAN silent: receive only, never drives the bus
  TWAI_MODE_LOOPBACK,     // STM32 extra: internal loopback + silent (self-test)
} twai_mode_t;

typedef enum {
  TWAI_STATE_STOPPED,
  TWAI_STATE_RUNNING,
  TWAI_STATE_BUS_OFF,
  TWAI_STATE_RECOVERING,
} twai_state_t;

// ─── Alerts (same bit values as ESP-IDF) ─────────────────────────────────────
#define TWAI_ALERT_TX_IDLE               0x00000001
#define TWAI_ALERT_TX_SUCCESS            0x00000002
#define TWAI_ALERT_RX_DATA               0x00000004
#define TWAI_ALERT_BELOW_ERR_WARN        0x00000008
#define TWAI_ALERT_ERR_ACTIVE            0x00000010
#define TWAI_ALERT_RECOVERY_IN_PROGRESS  0x00000020
#define TWAI_ALERT_BUS_RECOVERED         0x00000040
#define TWAI_ALERT_ARB_LOST              0x00000080
#define TWAI_ALERT_ABOVE_ERR_WARN        0x00000100
#define TWAI_ALERT_BUS_ERROR             0x00000200
#define TWAI_ALERT_TX_FAILED             0x00000400
#define TWAI_ALERT_RX_QUEUE_FULL         0x00000800
#define TWAI_ALERT_ERR_PASS              0x00001000
#define TWAI_ALERT_BUS_OFF               0x00002000
#define TWAI_ALERT_RX_FIFO_OVERRUN       0x00004000
#define TWAI_ALERT_ALL                   0x00007FFF
#define TWAI_ALERT_NONE                  0x00000000

// ─── Configuration ───────────────────────────────────────────────────────────
typedef struct {
  twai_mode_t mode;
  uint32_t tx_io;                  // Arduino pin, e.g. PD1
  uint32_t rx_io;                  // Arduino pin, e.g. PD0
  uint32_t tx_queue_len;           // software TX queue (max 16)
  uint32_t rx_queue_len;           // software RX queue (max 64)
  uint32_t alerts_enabled;
  bool     auto_bus_off_recovery;  // STM32 extra: hardware ABOM
} twai_general_config_t;

#define TWAI_GENERAL_CONFIG_DEFAULT(tx_io_num, rx_io_num, op_mode) { \
  .mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num,           \
  .tx_queue_len = 5, .rx_queue_len = 5,                              \
  .alerts_enabled = TWAI_ALERT_NONE, .auto_bus_off_recovery = false }

typedef struct {
  uint32_t bitrate;                // bit/s
  uint16_t sample_point_permille;  // 0 = default 875 (87.5 %)
  uint8_t  sjw;                    // 1..4, 0 = default 1
} twai_timing_config_t;

#define TWAI_TIMING_CONFIG_25KBITS()   { .bitrate = 25000,   .sample_point_permille = 875, .sjw = 1 }
#define TWAI_TIMING_CONFIG_50KBITS()   { .bitrate = 50000,   .sample_point_permille = 875, .sjw = 1 }
#define TWAI_TIMING_CONFIG_100KBITS()  { .bitrate = 100000,  .sample_point_permille = 875, .sjw = 1 }
#define TWAI_TIMING_CONFIG_125KBITS()  { .bitrate = 125000,  .sample_point_permille = 875, .sjw = 1 }
#define TWAI_TIMING_CONFIG_250KBITS()  { .bitrate = 250000,  .sample_point_permille = 875, .sjw = 1 }
#define TWAI_TIMING_CONFIG_500KBITS()  { .bitrate = 500000,  .sample_point_permille = 875, .sjw = 1 }
#define TWAI_TIMING_CONFIG_800KBITS()  { .bitrate = 800000,  .sample_point_permille = 800, .sjw = 1 }
#define TWAI_TIMING_CONFIG_1MBITS()    { .bitrate = 1000000, .sample_point_permille = 800, .sjw = 1 }

typedef struct {
  uint32_t acceptance_code;
  uint32_t acceptance_mask;  // bit = 1 → don't care
  bool     single_filter;    // ignored (always one 32-bit filter)
} twai_filter_config_t;

#define TWAI_FILTER_CONFIG_ACCEPT_ALL() \
  { .acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true }

// Accept only one standard ID (data or remote frames)
#define TWAI_FILTER_CONFIG_STD_ID(id) \
  { .acceptance_code = ((uint32_t)(id) << 21), .acceptance_mask = 0x001FFFFB, .single_filter = true }

// ─── Status ──────────────────────────────────────────────────────────────────
typedef struct {
  twai_state_t state;
  uint32_t msgs_to_tx;        // software queue + busy mailboxes
  uint32_t msgs_to_rx;
  uint32_t tx_error_counter;  // TEC
  uint32_t rx_error_counter;  // REC
  uint32_t tx_failed_count;
  uint32_t rx_missed_count;   // software RX queue was full
  uint32_t rx_overrun_count;  // hardware FIFO overrun
  uint32_t arb_lost_count;
  uint32_t bus_error_count;
} twai_status_info_t;

// ─── API ─────────────────────────────────────────────────────────────────────
esp_err_t twai_driver_install(const twai_general_config_t *g_config,
                              const twai_timing_config_t *t_config,
                              const twai_filter_config_t *f_config);
esp_err_t twai_driver_uninstall(void);
esp_err_t twai_start(void);
esp_err_t twai_stop(void);
esp_err_t twai_transmit(const twai_message_t *message, uint32_t timeout_ms);
esp_err_t twai_receive(twai_message_t *message, uint32_t timeout_ms);
esp_err_t twai_read_alerts(uint32_t *alerts, uint32_t timeout_ms);
esp_err_t twai_reconfigure_alerts(uint32_t alerts_enabled, uint32_t *current_alerts);
esp_err_t twai_initiate_recovery(void);
esp_err_t twai_get_status_info(twai_status_info_t *status_info);
esp_err_t twai_clear_transmit_queue(void);  // also aborts frames stuck in mailboxes
esp_err_t twai_clear_receive_queue(void);
