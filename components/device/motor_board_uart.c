#include "motor_board_uart.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"

#define RX_CHUNK_SIZE 64
#define FRAME_BUFFER_SIZE 128

static uart_port_t s_uart = UART_NUM_MAX;
static char s_frame_buffer[FRAME_BUFFER_SIZE];
static size_t s_frame_used;
static bool s_frame_active;

static esp_err_t write_frame(const char *frame)
{
    if (s_uart == UART_NUM_MAX || !frame) return ESP_ERR_INVALID_STATE;
    int length = strlen(frame);
    return uart_write_bytes(s_uart, frame, length) == length ? ESP_OK : ESP_FAIL;
}

esp_err_t motor_board_uart_init(const motor_board_uart_config_t *config)
{
    if (!config || config->uart_num < 0 || config->uart_num >= UART_NUM_MAX) return ESP_ERR_INVALID_ARG;
    s_uart = (uart_port_t)config->uart_num;
    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(s_uart, 2048, 0, 0, NULL, 0), "motor_uart", "driver install");
    ESP_RETURN_ON_ERROR(uart_param_config(s_uart, &uart_config), "motor_uart", "parameters");
    return uart_set_pin(s_uart, config->tx_gpio, config->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

esp_err_t motor_board_uart_set_speeds(const int16_t speed[MOTOR_BOARD_CHANNEL_COUNT])
{
    if (!speed) return ESP_ERR_INVALID_ARG;
    char frame[64];
    int n = snprintf(frame, sizeof(frame), "$spd:%d,%d,%d,%d#", speed[0], speed[1], speed[2], speed[3]);
    return n > 0 && n < sizeof(frame) ? write_frame(frame) : ESP_ERR_INVALID_SIZE;
}

esp_err_t motor_board_uart_stop(void)
{
    const int16_t zero[MOTOR_BOARD_CHANNEL_COUNT] = {0};
    return motor_board_uart_set_speeds(zero);
}

esp_err_t motor_board_uart_set_upload(bool total, bool delta, bool speed)
{
    char frame[32];
    int n = snprintf(frame, sizeof(frame), "$upload:%d,%d,%d#", total, delta, speed);
    return n > 0 && n < sizeof(frame) ? write_frame(frame) : ESP_ERR_INVALID_SIZE;
}

static bool parse_frame(const char *text, motor_board_frame_t *frame)
{
    const char *payload;
    if (!strncmp(text, "MAll:", 5)) { frame->type = MOTOR_BOARD_FRAME_ENCODER_TOTAL; payload = text + 5; }
    else if (!strncmp(text, "MTEP:", 5)) { frame->type = MOTOR_BOARD_FRAME_ENCODER_DELTA; payload = text + 5; }
    else if (!strncmp(text, "MSPD:", 5)) { frame->type = MOTOR_BOARD_FRAME_SPEED; payload = text + 5; }
    else return false;
    return sscanf(payload, "%" SCNd32 ",%" SCNd32 ",%" SCNd32 ",%" SCNd32,
                  &frame->value[0], &frame->value[1], &frame->value[2], &frame->value[3]) == 4;
}

esp_err_t motor_board_uart_read_frame(motor_board_frame_t *frame, uint32_t timeout_ms)
{
    if (!frame || s_uart == UART_NUM_MAX) return ESP_ERR_INVALID_ARG;
    uint8_t bytes[RX_CHUNK_SIZE];
    int count = uart_read_bytes(s_uart, bytes, sizeof(bytes), pdMS_TO_TICKS(timeout_ms));
    if (count < 0) return ESP_FAIL;
    for (int i = 0; i < count; ++i) {
        char c = bytes[i];
        if (c == '$') { s_frame_active = true; s_frame_used = 0; }
        else if (s_frame_active && c == '#') {
            s_frame_buffer[s_frame_used] = '\0'; s_frame_active = false; s_frame_used = 0;
            if (parse_frame(s_frame_buffer, frame)) return ESP_OK;
        } else if (s_frame_active && s_frame_used + 1 < sizeof(s_frame_buffer)) s_frame_buffer[s_frame_used++] = c;
        else if (s_frame_active) { s_frame_active = false; s_frame_used = 0; }
    }
    return ESP_ERR_TIMEOUT;
}
