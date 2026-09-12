#pragma once
/**
 * @file imu_web_runtime.h
 * @brief IMU 状态服务、Wi-Fi、HTTP 与采样任务的分阶段接口
 */

#include "esp_err.h"

/** @brief 初始化姿态共享状态与同步对象，不创建周期任务。 */
esp_err_t imu_runtime_service_init(void);
/** @brief 初始化并连接 Wi-Fi STA 服务。 */
esp_err_t wifi_service_init(void);
/** @brief 初始化姿态 HTTP 服务及其路由。 */
esp_err_t imu_web_service_init(void);
/** @brief 在外设和服务全部初始化完成后启动 100 Hz 采样任务。 */
esp_err_t imu_web_runtime_start_sampling(void);
