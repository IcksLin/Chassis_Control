#pragma once
/**
 * @file imu_web_runtime.h
 * @brief IMU 状态服务、Wi-Fi、HTTP 与采样任务的分阶段接口
 */

#include "esp_err.h"
#include <stdbool.h>
/** @brief 供控制层读取的最近姿态及 Z 轴角速度。 */
typedef struct { float roll_deg, pitch_deg, yaw_deg, gyro_z_dps; } imu_motion_state_t;

/** @brief 初始化姿态共享状态与同步对象，不创建周期任务。 */
esp_err_t imu_runtime_service_init(void);
/** @brief 初始化并连接 Wi-Fi STA 服务。 */
esp_err_t wifi_service_init(void);
/** @brief 初始化姿态 HTTP 服务及其路由。 */
esp_err_t imu_web_service_init(void);
/** @brief 在外设和服务全部初始化完成后启动 100 Hz 采样任务。 */
esp_err_t imu_web_runtime_start_sampling(void);
/** @brief 获取最近一帧融合 yaw，单位 deg；姿态尚未就绪时返回 false。 */
bool imu_runtime_get_yaw(float *yaw_deg);
/** @brief 原子读取控制所需姿态状态；尚未完成静止标定时返回 false。 */
bool imu_runtime_get_motion_state(imu_motion_state_t *state);
