#include "chassis_runtime.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include "chassis_hal.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_board_uart.h"

#define MOTOR_UART_NUM 1
#define MOTOR_TX_GPIO 17
#define MOTOR_RX_GPIO 18
#define MOTOR_BAUD_RATE 115200
#define FORWARD_PERIOD_MS 20
#define COMMAND_TIMEOUT_MS 500
#define TEST_SPEED_MM_S 120
#define TEST_RUN_MS 800
#define TEST_STOP_MS 500

static const char *TAG = "chassis_runtime";
static volatile bool s_test_running;
static volatile bool s_test_abort;
static const int8_t WHEEL_DIRECTION[CHASSIS_WHEEL_COUNT] = {1, 1, 1, 1};

static void forward_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        chassis_wheel_speeds_t command = {0};
        chassis_hal_get_motor_commands(&command, NULL);
        motor_board_uart_set_speeds(command.speed_mm_s);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(FORWARD_PERIOD_MS));
    }
}

static void receive_task(void *arg)
{
    int64_t last_report_us = 0;
    while (true) {
        motor_board_frame_t frame;
        if (motor_board_uart_read_frame(&frame, 100) != ESP_OK || frame.type != MOTOR_BOARD_FRAME_ENCODER_DELTA) continue;
        chassis_hal_update_encoder_delta(frame.value);
        int64_t now = esp_timer_get_time();
        if (now - last_report_us >= 100000) {
            chassis_encoder_delta_t encoder;
            chassis_hal_get_encoder_delta(&encoder);
            last_report_us = now;
            printf("ENC logical RR=%" PRId32 " RF=%" PRId32 " LR=%" PRId32 " LF=%" PRId32 "\n",
                   encoder.delta[0], encoder.delta[1], encoder.delta[2], encoder.delta[3]);
        }
    }
}

static void stop_all(void) { chassis_hal_stop(); motor_board_uart_stop(); }

static void sequential_test_task(void *arg)
{
    puts("TEST START: one wheel at a time; send any character to abort.");
    for (chassis_wheel_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        chassis_wheel_speeds_t speed = {0};
        speed.speed_mm_s[wheel] = TEST_SPEED_MM_S;
        printf("TEST wheel %u: %s, command=+%d mm/s\n", (unsigned)(wheel + 1), chassis_hal_wheel_name(wheel), TEST_SPEED_MM_S);
        for (int elapsed = 0; elapsed < TEST_RUN_MS; elapsed += 50) {
            chassis_hal_set_wheel_speeds(&speed);
            vTaskDelay(pdMS_TO_TICKS(50));
            if (s_test_abort) {
                stop_all(); puts("TEST ABORTED; all wheels stopped."); s_test_running = false; vTaskDelete(NULL);
            }
        }
        stop_all(); vTaskDelay(pdMS_TO_TICKS(TEST_STOP_MS));
    }
    puts("TEST COMPLETE; all wheels stopped."); s_test_running = false; vTaskDelete(NULL);
}

static void console_task(void *arg)
{
    puts("Ready. Lift the chassis clear of the floor, then enter 't' to run the test.");
    puts("During the test, enter any character to stop immediately.");
    while (true) {
        int ch = getchar();
        if (!s_test_running && (ch == 't' || ch == 'T')) {
            s_test_abort = false; s_test_running = true;
            if (xTaskCreate(sequential_test_task, "wheel_test", 4096, NULL, 3, NULL) != pdPASS) {
                s_test_running = false; puts("Could not start test task.");
            }
        } else if (s_test_running && ch != EOF) s_test_abort = true;
        else vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t chassis_runtime_start(void)
{
    ESP_RETURN_ON_ERROR(chassis_hal_init(WHEEL_DIRECTION, COMMAND_TIMEOUT_MS), TAG, "HAL initialization");
    const motor_board_uart_config_t config = {
        .uart_num = MOTOR_UART_NUM, .tx_gpio = MOTOR_TX_GPIO,
        .rx_gpio = MOTOR_RX_GPIO, .baud_rate = MOTOR_BAUD_RATE,
    };
    ESP_RETURN_ON_ERROR(motor_board_uart_init(&config), TAG, "motor UART initialization");
    stop_all(); vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(motor_board_uart_set_upload(false, true, false), TAG, "encoder upload setup");
    ESP_LOGI(TAG, "motor UART: TX GPIO%d, RX GPIO%d, 115200 8N1", MOTOR_TX_GPIO, MOTOR_RX_GPIO);
    ESP_LOGI(TAG, "50 Hz forwarding enabled; 500 ms command watchdog enabled");
    if (xTaskCreate(receive_task, "motor_rx", 4096, NULL, 5, NULL) != pdPASS ||
        xTaskCreate(forward_task, "motor_forward", 3072, NULL, 4, NULL) != pdPASS ||
        xTaskCreate(console_task, "console", 4096, NULL, 3, NULL) != pdPASS) {
        stop_all(); return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
