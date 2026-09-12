#include "chassis_hal.h"

#include <stddef.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    chassis_wheel_speeds_t target;
    chassis_encoder_delta_t encoder;
    int8_t direction[CHASSIS_WHEEL_COUNT];
    int64_t last_set_us;
    int64_t timeout_us;
    SemaphoreHandle_t lock;
} chassis_hal_state_t;

static chassis_hal_state_t s_hal;
static const char *const WHEEL_NAMES[CHASSIS_WHEEL_COUNT] = {
    "right_rear/M1", "left_rear/M2", "right_front/M3", "left_front/M4"
};

esp_err_t chassis_hal_init(const int8_t direction[CHASSIS_WHEEL_COUNT], uint32_t timeout_ms)
{
    if (!direction || timeout_ms == 0) return ESP_ERR_INVALID_ARG;
    memset(&s_hal, 0, sizeof(s_hal));
    for (size_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (direction[i] != 1 && direction[i] != -1) return ESP_ERR_INVALID_ARG;
        s_hal.direction[i] = direction[i];
    }
    s_hal.timeout_us = timeout_ms * 1000LL;
    s_hal.last_set_us = esp_timer_get_time();
    s_hal.lock = xSemaphoreCreateMutex();
    return s_hal.lock ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t chassis_hal_set_wheel_speeds(const chassis_wheel_speeds_t *speed)
{
    if (!speed || !s_hal.lock) return ESP_ERR_INVALID_ARG;
    for (size_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        if (speed->speed_mm_s[i] < -1000 || speed->speed_mm_s[i] > 1000) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_hal.lock, portMAX_DELAY);
    s_hal.target = *speed; s_hal.last_set_us = esp_timer_get_time();
    xSemaphoreGive(s_hal.lock);
    return ESP_OK;
}

void chassis_hal_stop(void)
{
    const chassis_wheel_speeds_t zero = {0};
    (void)chassis_hal_set_wheel_speeds(&zero);
}

void chassis_hal_get_motor_commands(chassis_wheel_speeds_t *motor_speed, bool *timed_out)
{
    if (!motor_speed || !s_hal.lock) return;
    xSemaphoreTake(s_hal.lock, portMAX_DELAY);
    bool expired = esp_timer_get_time() - s_hal.last_set_us > s_hal.timeout_us;
    for (size_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        motor_speed->speed_mm_s[i] = expired ? 0 : s_hal.target.speed_mm_s[i] * s_hal.direction[i];
    xSemaphoreGive(s_hal.lock);
    if (timed_out) *timed_out = expired;
}

void chassis_hal_update_encoder_delta(const int32_t motor_delta[CHASSIS_WHEEL_COUNT])
{
    if (!motor_delta || !s_hal.lock) return;
    xSemaphoreTake(s_hal.lock, portMAX_DELAY);
    for (size_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) s_hal.encoder.delta[i] = motor_delta[i] * s_hal.direction[i];
    xSemaphoreGive(s_hal.lock);
}

void chassis_hal_get_encoder_delta(chassis_encoder_delta_t *logical_delta)
{
    if (!logical_delta || !s_hal.lock) return;
    xSemaphoreTake(s_hal.lock, portMAX_DELAY); *logical_delta = s_hal.encoder; xSemaphoreGive(s_hal.lock);
}

const char *chassis_hal_wheel_name(chassis_wheel_t wheel)
{
    return wheel < CHASSIS_WHEEL_COUNT ? WHEEL_NAMES[wheel] : "unknown";
}
