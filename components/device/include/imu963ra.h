#pragma once
/**
 * @file imu963ra.h
 * @brief IMU963RA 六轴及磁力计物理量读取接口
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @brief 一帧完成单位换算的 IMU963RA 传感器数据。 */
typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float mag_gauss[3];
    bool mag_valid;
} imu963ra_sample_t;

/** @brief 初始化硬件 I2C、LSM6DSR 和板载磁力计。 */
esp_err_t imu963ra_init(void);
/** @brief 读取一帧六轴及最近有效磁场数据。 */
esp_err_t imu963ra_read(imu963ra_sample_t *sample);
/** @brief 获取探测到的 LSM6DSR 七位 I2C 地址。 */
uint8_t imu963ra_address(void);
