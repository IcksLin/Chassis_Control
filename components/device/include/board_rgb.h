/**
 * @file board_rgb.h
 * @brief 开发板板载 WS2812 RGB 指示灯安全关闭接口
 */
#pragma once
#include "esp_err.h"
/** @brief 将 GPIO48 保持低电平，使常见 ESP32-S3 板载 WS2812 进入复位/熄灭状态。 */
esp_err_t board_rgb_init_off(void);
