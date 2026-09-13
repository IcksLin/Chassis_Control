/**
 * @file move_control.h
 * @brief 四个 45 度全向轮的速度矢量合成接口
 */
#pragma once
#include <stdint.h>
#include "chassis_hal.h"
/**
 * @brief 将底盘平移与旋转指令合成为四轮逻辑速度
 * @param vx 前进方向归一化速度，范围建议为 -1.0 到 1.0
 * @param vy 右侧方向归一化速度，范围建议为 -1.0 到 1.0
 * @param w 顺时针旋转归一化速度，范围建议为 -1.0 到 1.0
 * @param max_speed 单轮最大速度，单位 mm/s
 * @param out 输出的右后、左后、右前、左前逻辑轮速度
 * @note 函数只进行算法计算，不访问硬件；输出仍需经过底盘 HAL 看门狗。
 */
void move_control_init(void);
/** @brief 原子清除运动意图，供安全锁和异常路径调用。 */
void move_control_stop(void);
void move_control_set_command(float vx, float vy, float rotation, float yaw_deg, int16_t max_speed);
/** @brief 设置绝对目标航向并进入原地旋转控制。 */
void move_control_set_heading_target(float yaw_deg, int16_t max_speed);
/** @brief 用当前姿态更新航向 PD 与可选坡度前馈，并生成轮速。 */
bool move_control_update(float yaw_deg, float gyro_z_dps, float roll_deg, float pitch_deg,
                         chassis_wheel_speeds_t *out);
bool move_control_is_active(void);
/** @brief 设置航向 PD 与坡度前馈参数；超出安全范围的参数返回 false。 */
bool move_control_set_tuning(float kp, float kd, float pitch_feedforward, float roll_feedforward);
void move_control_vector_to_wheels(float vx, float vy, float w, int16_t max_speed,
                                   chassis_wheel_speeds_t *out);
