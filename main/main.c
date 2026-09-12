/**
 * @file main.c
 * @brief ESP32-S3 四驱底盘控制与姿态监视程序入口
 */
#include "chassis_runtime.h"

/**
 * @brief 启动底盘运行时及其全部设备与系统服务
 */
void app_main(void) { chassis_runtime_start(); }
