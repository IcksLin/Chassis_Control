#pragma once
/**
 * @file chassis_hal.h
 * @brief 四轮底盘逻辑速度与编码器方向统一接口
 */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @brief 底盘四个逻辑轮的位置编号。 */
typedef enum {
    CHASSIS_WHEEL_RIGHT_REAR = 0,
    CHASSIS_WHEEL_LEFT_REAR,
    CHASSIS_WHEEL_RIGHT_FRONT,
    CHASSIS_WHEEL_LEFT_FRONT,
    CHASSIS_WHEEL_COUNT,
} chassis_wheel_t;

/** @brief 四个逻辑轮的线速度集合，单位 mm/s。 */
typedef struct { int16_t speed_mm_s[CHASSIS_WHEEL_COUNT]; } chassis_wheel_speeds_t;
/** @brief 四个逻辑轮的编码器周期增量集合。 */
typedef struct { int32_t delta[CHASSIS_WHEEL_COUNT]; } chassis_encoder_delta_t;

/** @brief 初始化方向映射与速度命令看门狗。 */
esp_err_t chassis_hal_init(const int8_t direction[CHASSIS_WHEEL_COUNT], uint32_t timeout_ms);
/** @brief 写入四轮逻辑目标速度。 */
esp_err_t chassis_hal_set_wheel_speeds(const chassis_wheel_speeds_t *logical_speed);
/** @brief 清零全部目标速度。 */
void chassis_hal_stop(void);
/** @brief 获取经过方向映射及超时保护后的物理通道速度。 */
void chassis_hal_get_motor_commands(chassis_wheel_speeds_t *motor_speed, bool *timed_out);
/** @brief 写入物理电机通道编码器增量并转换方向。 */
void chassis_hal_update_encoder_delta(const int32_t motor_delta[CHASSIS_WHEEL_COUNT]);
/** @brief 获取最近一帧逻辑编码器增量。 */
void chassis_hal_get_encoder_delta(chassis_encoder_delta_t *logical_delta);
/** @brief 获取指定逻辑轮的可读名称。 */
const char *chassis_hal_wheel_name(chassis_wheel_t wheel);
