/**
 * @file chassis_runtime.c
 * @brief 底盘设备初始化、50 Hz 命令转发、反馈接收与单轮测试任务
 */
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
#include "imu963ra.h"
#include "board_rgb.h"
#include "motor_board_uart.h"
#include "move_control.h"
#include "imu_web_runtime.h"
#include "motion_safety.h"

#define MOTOR_UART_NUM 1
#define MOTOR_TX_GPIO 17
#define MOTOR_RX_GPIO 18
#define MOTOR_BAUD_RATE 115200
#define FORWARD_PERIOD_MS 20
#define COMMAND_TIMEOUT_MS 500
#define CONTROL_LEASE_MS 1500
#define PERIPHERAL_SETTLE_MS 200
#define TASK_START_GUARD_MS 200
#define TEST_SPEED_MM_S 120
#define TEST_RUN_MS 3000
#define TEST_STOP_MS 500

static const char *TAG = "chassis_runtime";
static volatile bool s_test_running;
static volatile bool s_test_abort;
static const int8_t WHEEL_DIRECTION[CHASSIS_WHEEL_COUNT] = {1, 1, 1, 1};
static bool s_peripherals_initialized;
static bool s_services_initialized;
static bool s_tasks_started;

/**
 * @brief 以 50 Hz 将 HAL 目标速度转发给电机下位机
 * @param arg FreeRTOS 任务参数，当前未使用
 */
static void forward_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        chassis_wheel_speeds_t command = {0};
        imu_motion_state_t motion;
        const motion_safety_owner_t owner = motion_safety_owner();
        if (owner == MOTION_SAFETY_OWNER_WEB && imu_runtime_get_motion_state(&motion) &&
            move_control_update(motion.yaw_deg, motion.gyro_z_dps,
                                                                          motion.roll_deg, motion.pitch_deg, &command))
            (void)chassis_hal_set_wheel_speeds(&command);
        else if (owner == MOTION_SAFETY_OWNER_NONE || owner == MOTION_SAFETY_OWNER_WEB) {
            move_control_stop();
            chassis_hal_stop();
        }
        chassis_hal_get_motor_commands(&command, NULL);
        if (motor_board_uart_set_speeds(command.speed_mm_s) != ESP_OK) {
            motion_safety_lock("motor UART transmit failure");
            move_control_stop();
            chassis_hal_stop();
        }
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(FORWARD_PERIOD_MS));
    }
}

/**
 * @brief 接收下位机反馈并更新逻辑轮编码器增量
 * @param arg FreeRTOS 任务参数，当前未使用
 */
static void receive_task(void *arg)
{
    int64_t last_report_us = 0;
    while (true) {
        motor_board_frame_t frame;
        if (motor_board_uart_read_frame(&frame, 100) != ESP_OK || frame.type != MOTOR_BOARD_FRAME_ENCODER_DELTA) continue;
        motion_safety_note_feedback();
        chassis_hal_update_encoder_delta(frame.value);
        int64_t now = esp_timer_get_time();
        if (now - last_report_us >= 100000) {
            chassis_encoder_delta_t encoder;
            chassis_hal_get_encoder_delta(&encoder);
            last_report_us = now;
            printf("ENC logical RR=%" PRId32 " LR=%" PRId32 " RF=%" PRId32 " LF=%" PRId32 "\n",
                   encoder.delta[0], encoder.delta[1], encoder.delta[2], encoder.delta[3]);
        }
    }
}

/**
 * @brief 同时清零 HAL 目标速度并向下位机发送立即停止命令
 */
static void stop_all(void)
{
    motion_safety_lock("explicit stop");
    move_control_stop();
    chassis_hal_stop();
    (void)motor_board_uart_stop();
}

/*
* @brief 轮胎功能测试函数
*/
/**
 * @brief 按逻辑编号依次运行四轮功能检测
 * @param arg FreeRTOS 任务参数，当前未使用
 * @note 仅在串口明确输入 t 时创建，结束或中止时强制发送零速。
 */
static void sequential_test_task(void *arg)
{
    const uint32_t session = (uint32_t)(uintptr_t)arg;
    uint32_t sequence = 1;
    puts("TEST START: one wheel at a time; send any character to abort.");
    for (chassis_wheel_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        chassis_wheel_speeds_t speed = {0};
        speed.speed_mm_s[wheel] = TEST_SPEED_MM_S;
        printf("TEST wheel %u: %s, command=+%d mm/s\n", (unsigned)(wheel + 1), chassis_hal_wheel_name(wheel), TEST_SPEED_MM_S);
        for (int elapsed = 0; elapsed < TEST_RUN_MS; elapsed += 50) {
            if (motion_safety_renew(MOTION_SAFETY_OWNER_TEST, session, sequence++) != ESP_OK) {
                stop_all(); puts("TEST SAFETY LOCKED; all wheels stopped."); s_test_running = false; vTaskDelete(NULL);
            }
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

/**
 * @brief 监听串口控制字符并管理单轮检测任务
 * @param arg FreeRTOS 任务参数，当前未使用
 */
static void console_task(void *arg)
{
    puts("Ready. Lift the chassis clear of the floor, then enter 't' to run the test.");
    puts("During the test, enter any character to stop immediately.");
    while (true) {
        int ch = getchar();
        if (!s_test_running && (ch == 't' || ch == 'T')) {
            uint32_t session = 0;
            s_test_abort = false; s_test_running = true;
            if (motion_safety_acquire(MOTION_SAFETY_OWNER_TEST, &session) != ESP_OK ||
                xTaskCreate(sequential_test_task, "wheel_test", 4096,
                            (void *)(uintptr_t)session, 3, NULL) != pdPASS) {
                s_test_running = false; puts("Could not start test task.");
                stop_all();
            }
        } else if (s_test_running && ch != EOF) s_test_abort = true;
        else vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 第一阶段：初始化所有物理外设
 * @return esp_err_t ESP_OK 表示 IMU 和电机串口均初始化成功
 * @note 本阶段不创建任何本工程 FreeRTOS 任务；配置完成后等待 200 ms，
 *       稳定期结束才置位外设就绪标志，防止服务提前访问硬件。
 */
esp_err_t chassis_runtime_init_peripherals(void)
{
    if (s_peripherals_initialized) return ESP_OK;
    ESP_RETURN_ON_ERROR(board_rgb_init_off(), TAG, "board RGB peripheral initialization");
    ESP_RETURN_ON_ERROR(imu963ra_init(), TAG, "IMU peripheral initialization");
    ESP_LOGI(TAG, "IMU963RA found at 0x%02x; SDA GPIO8, SCL GPIO9", imu963ra_address());
    const motor_board_uart_config_t config = {
        .uart_num = MOTOR_UART_NUM, .tx_gpio = MOTOR_TX_GPIO,
        .rx_gpio = MOTOR_RX_GPIO, .baud_rate = MOTOR_BAUD_RATE,
    };
    ESP_RETURN_ON_ERROR(motor_board_uart_init(&config), TAG, "motor UART initialization");
    ESP_RETURN_ON_ERROR(motor_board_uart_stop(), TAG, "initial motor stop");
    vTaskDelay(pdMS_TO_TICKS(PERIPHERAL_SETTLE_MS));
    s_peripherals_initialized = true;
    return ESP_OK;
}

/**
 * @brief 第二阶段：初始化软件 HAL、数据状态、Wi-Fi 和 HTTP 服务
 * @return esp_err_t ESP_OK 表示所有服务初始化成功
 * @note 必须先完成外设阶段；本阶段不创建本工程的周期任务。
 */
esp_err_t chassis_runtime_init_services(void)
{
    if (!s_peripherals_initialized) return ESP_ERR_INVALID_STATE;
    if (s_services_initialized) return ESP_OK;
    ESP_RETURN_ON_ERROR(chassis_hal_init(WHEEL_DIRECTION, COMMAND_TIMEOUT_MS), TAG, "HAL service initialization");
    ESP_RETURN_ON_ERROR(motion_safety_init(CONTROL_LEASE_MS), TAG, "motion safety service initialization");
    move_control_init();
    ESP_RETURN_ON_ERROR(imu_runtime_service_init(), TAG, "IMU state service initialization");
    ESP_RETURN_ON_ERROR(wifi_service_init(), TAG, "Wi-Fi service initialization");
    ESP_RETURN_ON_ERROR(imu_web_service_init(), TAG, "IMU web service initialization");
    ESP_RETURN_ON_ERROR(motor_board_uart_set_upload(false, true, false), TAG, "encoder upload setup");
    ESP_LOGI(TAG, "motor UART: TX GPIO%d, RX GPIO%d, 115200 8N1", MOTOR_TX_GPIO, MOTOR_RX_GPIO);
    ESP_LOGI(TAG, "50 Hz forwarding; 1500 ms owner lease; continuous zero-speed fail-safe enabled");
    s_services_initialized = true;
    return ESP_OK;
}

/**
 * @brief 第三阶段：统一启动本工程的中断或 FreeRTOS 线程
 * @return esp_err_t ESP_OK 表示全部周期任务创建成功
 * @note 外设与服务全部就绪后统一等待 200 ms，再启动任何本工程任务。
 */
esp_err_t chassis_runtime_start_tasks(void)
{
    if (!s_services_initialized) return ESP_ERR_INVALID_STATE;
    if (s_tasks_started) return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(TASK_START_GUARD_MS));
    ESP_RETURN_ON_ERROR(imu_web_runtime_start_sampling(), TAG, "IMU task start");
    if (xTaskCreate(receive_task, "motor_rx", 4096, NULL, 5, NULL) != pdPASS ||
        xTaskCreate(forward_task, "motor_forward", 3072, NULL, 4, NULL) != pdPASS ||
        xTaskCreate(console_task, "console", 4096, NULL, 3, NULL) != pdPASS) {
        stop_all(); return ESP_ERR_NO_MEM;
    }
    s_tasks_started = true;
    return ESP_OK;
}

/**
 * @brief 按“外设→服务→中断或线程”执行统一初始化流水线
 * @return esp_err_t ESP_OK 表示三个初始化阶段均成功
 */
esp_err_t chassis_runtime_start(void)
{
    ESP_RETURN_ON_ERROR(chassis_runtime_init_peripherals(), TAG, "peripheral phase");
    ESP_RETURN_ON_ERROR(chassis_runtime_init_services(), TAG, "service phase");
    ESP_RETURN_ON_ERROR(chassis_runtime_start_tasks(), TAG, "task phase");
    return ESP_OK;
}
