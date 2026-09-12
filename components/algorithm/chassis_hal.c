/**
 * @file chassis_hal.c
 * @brief 四轮逻辑编号、方向修正、速度看门狗与编码器映射 HAL 实现
 */
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

/**
 * @brief 初始化底盘 HAL 状态和各轮方向系数
 * @param direction 四个逻辑轮的方向系数，只允许 1 或 -1
 * @param timeout_ms 速度命令失效超时时间
 * @return esp_err_t ESP_OK 表示初始化成功
 */
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

/**
 * @brief 写入四个逻辑轮的目标速度并刷新命令时间戳
 * @param speed 逻辑轮速度，单位 mm/s
 * @return esp_err_t ESP_OK 表示速度范围与状态有效
 */
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

/**
 * @brief 将四个逻辑轮目标速度清零
 */
void chassis_hal_stop(void)
{
    const chassis_wheel_speeds_t zero = {0};
    (void)chassis_hal_set_wheel_speeds(&zero);
}

/**
 * @brief 获取经过方向换算和超时保护后的下位机通道命令
 * @param motor_speed 输出的四通道物理电机速度
 * @param timed_out 可选输出，返回本次命令是否因看门狗超时而清零
 */
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

/**
 * @brief 将下位机物理通道编码器增量换算为逻辑轮正方向
 * @param motor_delta Motor1 到 Motor4 的编码器增量
 */
void chassis_hal_update_encoder_delta(const int32_t motor_delta[CHASSIS_WHEEL_COUNT])
{
    if (!motor_delta || !s_hal.lock) return;
    xSemaphoreTake(s_hal.lock, portMAX_DELAY);
    for (size_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) s_hal.encoder.delta[i] = motor_delta[i] * s_hal.direction[i];
    xSemaphoreGive(s_hal.lock);
}

/**
 * @brief 获取最近一次逻辑轮编码器增量
 * @param logical_delta 输出的四轮逻辑增量
 */
void chassis_hal_get_encoder_delta(chassis_encoder_delta_t *logical_delta)
{
    if (!logical_delta || !s_hal.lock) return;
    xSemaphoreTake(s_hal.lock, portMAX_DELAY); *logical_delta = s_hal.encoder; xSemaphoreGive(s_hal.lock);
}

/**
 * @brief 获取逻辑轮编号对应的轮位与电机通道名称
 * @param wheel 逻辑轮编号
 * @return const char* 静态名称字符串，编号非法时返回 unknown
 */
const char *chassis_hal_wheel_name(chassis_wheel_t wheel)
{
    return wheel < CHASSIS_WHEEL_COUNT ? WHEEL_NAMES[wheel] : "unknown";
}
