#pragma once
/**
 * @file motor_board_uart.h
 * @brief 四路电机驱动下位机串口协议接口
 */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define MOTOR_BOARD_CHANNEL_COUNT 4

/** @brief 下位机反馈帧的数据类别。 */
typedef enum {
    MOTOR_BOARD_FRAME_NONE = 0,
    MOTOR_BOARD_FRAME_ENCODER_TOTAL,
    MOTOR_BOARD_FRAME_ENCODER_DELTA,
    MOTOR_BOARD_FRAME_SPEED,
} motor_board_frame_type_t;

/** @brief 一帧解析后的四通道反馈数据。 */
typedef struct {
    motor_board_frame_type_t type;
    int32_t value[MOTOR_BOARD_CHANNEL_COUNT];
} motor_board_frame_t;

/** @brief 电机下位机 UART 硬件参数。 */
typedef struct { int uart_num; int tx_gpio; int rx_gpio; int baud_rate; } motor_board_uart_config_t;

/** @brief 初始化下位机 UART。 */
esp_err_t motor_board_uart_init(const motor_board_uart_config_t *config);
/** @brief 发送四个物理通道的目标速度。 */
esp_err_t motor_board_uart_set_speeds(const int16_t speed_mm_s[MOTOR_BOARD_CHANNEL_COUNT]);
/** @brief 发送四通道零速命令。 */
esp_err_t motor_board_uart_stop(void);
/** @brief 配置下位机主动上传的数据类别。 */
esp_err_t motor_board_uart_set_upload(bool total_encoder, bool delta_encoder, bool speed);
/** @brief 接收并解析一帧下位机反馈。 */
esp_err_t motor_board_uart_read_frame(motor_board_frame_t *frame, uint32_t timeout_ms);
