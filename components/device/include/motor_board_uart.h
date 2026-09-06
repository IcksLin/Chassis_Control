#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define MOTOR_BOARD_CHANNEL_COUNT 4

typedef enum {
    MOTOR_BOARD_FRAME_NONE = 0,
    MOTOR_BOARD_FRAME_ENCODER_TOTAL,
    MOTOR_BOARD_FRAME_ENCODER_DELTA,
    MOTOR_BOARD_FRAME_SPEED,
} motor_board_frame_type_t;

typedef struct {
    motor_board_frame_type_t type;
    int32_t value[MOTOR_BOARD_CHANNEL_COUNT];
} motor_board_frame_t;

typedef struct { int uart_num; int tx_gpio; int rx_gpio; int baud_rate; } motor_board_uart_config_t;

esp_err_t motor_board_uart_init(const motor_board_uart_config_t *config);
esp_err_t motor_board_uart_set_speeds(const int16_t speed_mm_s[MOTOR_BOARD_CHANNEL_COUNT]);
esp_err_t motor_board_uart_stop(void);
esp_err_t motor_board_uart_set_upload(bool total_encoder, bool delta_encoder, bool speed);
esp_err_t motor_board_uart_read_frame(motor_board_frame_t *frame, uint32_t timeout_ms);
