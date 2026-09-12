#pragma once
/**
 * @file imu963ra_attitude.h
 * @brief IMU963RA 姿态解算与用户归零接口
 */

#include <stdbool.h>
#include <stdint.h>

/** @brief 欧拉角与姿态四元数输出。 */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
} imu963ra_attitude_t;

/** @brief 重置解算器内部状态。 */
void imu963ra_attitude_reset(void);
/** @brief 设置三轴陀螺仪静态零偏，单位 deg/s。 */
void imu963ra_attitude_set_gyro_bias_dps(float x, float y, float z);
/** @brief 使用加速度方向建立水平初始姿态。 */
void imu963ra_attitude_init_horizontal(float ax, float ay, float az);
/** @brief 使用实测 dt 更新一次姿态，可通过 mag_valid 控制磁场融合。 */
void imu963ra_attitude_update_horizontal(float gx, float gy, float gz,
                                         float ax, float ay, float az,
                                         float mx, float my, float mz,
                                         bool mag_valid, float dt);
/** @brief 将当前姿态设置为用户显示零点。 */
void imu963ra_attitude_zero_current(void);
/** @brief 获取应用用户零点后的姿态结果。 */
void imu963ra_attitude_get(imu963ra_attitude_t *out);
