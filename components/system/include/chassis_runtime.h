#pragma once
/**
 * @file chassis_runtime.h
 * @brief 底盘系统运行时启动接口
 */
#include "esp_err.h"
/** @brief 第一阶段：初始化 IMU 和电机串口等物理外设。 */
esp_err_t chassis_runtime_init_peripherals(void);
/** @brief 第二阶段：初始化 HAL、网络和网页等服务。 */
esp_err_t chassis_runtime_init_services(void);
/** @brief 第三阶段：延时后启动采样、通信和控制台任务。 */
esp_err_t chassis_runtime_start_tasks(void);
/** @brief 按“外设→服务→线程”流水线启动整个工程。 */
esp_err_t chassis_runtime_start(void);
