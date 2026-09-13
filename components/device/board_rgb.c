/**
 * @file board_rgb.c
 * @brief 将板载 RGB 数据线钳位为低电平，防止上电或复位后的残留显示
 */
#include "board_rgb.h"
#include "driver/gpio.h"
/* ESP32-S3-DevKitC-1 早期版使用 GPIO48，v1.1 使用 GPIO38。 */
#define BOARD_RGB_PIN_MASK ((1ULL << GPIO_NUM_38) | (1ULL << GPIO_NUM_48))
esp_err_t board_rgb_init_off(void)
{
    const gpio_config_t config = {.pin_bit_mask = BOARD_RGB_PIN_MASK, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) return err;
    err = gpio_set_level(GPIO_NUM_38, 0);
    return err == ESP_OK ? gpio_set_level(GPIO_NUM_48, 0) : err;
}
