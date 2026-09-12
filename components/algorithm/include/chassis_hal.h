#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    CHASSIS_WHEEL_RIGHT_REAR = 0,
    CHASSIS_WHEEL_LEFT_REAR,
    CHASSIS_WHEEL_RIGHT_FRONT,
    CHASSIS_WHEEL_LEFT_FRONT,
    CHASSIS_WHEEL_COUNT,
} chassis_wheel_t;

typedef struct { int16_t speed_mm_s[CHASSIS_WHEEL_COUNT]; } chassis_wheel_speeds_t;
typedef struct { int32_t delta[CHASSIS_WHEEL_COUNT]; } chassis_encoder_delta_t;

esp_err_t chassis_hal_init(const int8_t direction[CHASSIS_WHEEL_COUNT], uint32_t timeout_ms);
esp_err_t chassis_hal_set_wheel_speeds(const chassis_wheel_speeds_t *logical_speed);
void chassis_hal_stop(void);
void chassis_hal_get_motor_commands(chassis_wheel_speeds_t *motor_speed, bool *timed_out);
void chassis_hal_update_encoder_delta(const int32_t motor_delta[CHASSIS_WHEEL_COUNT]);
void chassis_hal_get_encoder_delta(chassis_encoder_delta_t *logical_delta);
const char *chassis_hal_wheel_name(chassis_wheel_t wheel);
