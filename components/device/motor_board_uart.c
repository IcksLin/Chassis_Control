/**
 * @file motor_board_uart.c
 * @brief 四路电机驱动下位机串口协议封装与反馈帧解析
 */
#include "motor_board_uart.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define RX_CHUNK_SIZE 64
#define FRAME_BUFFER_SIZE 128

static uart_port_t s_uart = UART_NUM_MAX;
static char s_frame_buffer[FRAME_BUFFER_SIZE];
static size_t s_frame_used;
static bool s_frame_active;
static SemaphoreHandle_t s_tx_lock;

/**
 * @brief 向电机下位机发送一帧完整 ASCII 协议数据
 * @param frame 以美元符号开头、井号结尾的协议字符串
 * @return esp_err_t ESP_OK 表示整帧写入成功
 */
static esp_err_t write_frame(const char *frame)
{
    if (s_uart == UART_NUM_MAX || !frame || !s_tx_lock) return ESP_ERR_INVALID_STATE;
    int length = strlen(frame);
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    const int written = uart_write_bytes(s_uart, frame, length);
    xSemaphoreGive(s_tx_lock);
    return written == length ? ESP_OK : ESP_FAIL;
}

/**
 * @brief 初始化电机下位机 UART 端口与引脚
 * @param config UART 编号、引脚和波特率配置
 * @return esp_err_t ESP_OK 表示初始化成功
 */
esp_err_t motor_board_uart_init(const motor_board_uart_config_t *config)
{
    if (!config || config->uart_num < 0 || config->uart_num >= UART_NUM_MAX) return ESP_ERR_INVALID_ARG;
    s_uart = (uart_port_t)config->uart_num;
    if (!s_tx_lock) s_tx_lock = xSemaphoreCreateMutex();
    if (!s_tx_lock) return ESP_ERR_NO_MEM;
    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(s_uart, 2048, 0, 0, NULL, 0), "motor_uart", "driver install");
    ESP_RETURN_ON_ERROR(uart_param_config(s_uart, &uart_config), "motor_uart", "parameters");
    return uart_set_pin(s_uart, config->tx_gpio, config->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief 设置四个下位机物理通道的目标速度
 * @param speed Motor1 到 Motor4 的速度数组
 * @return esp_err_t ESP_OK 表示命令帧发送成功
 */
esp_err_t motor_board_uart_set_speeds(const int16_t speed[MOTOR_BOARD_CHANNEL_COUNT])
{
    if (!speed) return ESP_ERR_INVALID_ARG;
    char frame[64];
    int n = snprintf(frame, sizeof(frame), "$spd:%d,%d,%d,%d#", speed[0], speed[1], speed[2], speed[3]);
    return n > 0 && n < sizeof(frame) ? write_frame(frame) : ESP_ERR_INVALID_SIZE;
}

/**
 * @brief 向下位机发送四通道零速命令
 * @return esp_err_t ESP_OK 表示停止命令发送成功
 */
esp_err_t motor_board_uart_stop(void)
{
    const int16_t zero[MOTOR_BOARD_CHANNEL_COUNT] = {0};
    return motor_board_uart_set_speeds(zero);
}

/**
 * @brief 设置下位机主动上传的数据类型
 * @param total 是否上传编码器累计值
 * @param delta 是否上传编码器增量
 * @param speed 是否上传实测速度
 * @return esp_err_t ESP_OK 表示配置命令发送成功
 */
esp_err_t motor_board_uart_set_upload(bool total, bool delta, bool speed)
{
    char frame[32];
    int n = snprintf(frame, sizeof(frame), "$upload:%d,%d,%d#", total, delta, speed);
    return n > 0 && n < sizeof(frame) ? write_frame(frame) : ESP_ERR_INVALID_SIZE;
}

/**
 * @brief 解析一帧去除首尾定界符后的下位机反馈
 * @param text 帧正文
 * @param frame 解析后的帧类型和四通道数值
 * @return bool true 表示帧类型和四个数值均有效
 */
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

/**
 * @brief 从 UART 字节流中提取并解析一帧下位机反馈
 * @param frame 输出帧
 * @param timeout_ms 本次串口读取超时时间
 * @return esp_err_t ESP_OK 表示获得完整有效帧，ESP_ERR_TIMEOUT 表示尚未成帧
 */
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
