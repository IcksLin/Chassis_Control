/**
 * @file motion_safety.h
 * @brief 底盘运动控制所有者、会话代次和超时租约安全门
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @brief 可独占底盘的命令来源。 */
typedef enum {
    MOTION_SAFETY_OWNER_NONE = 0,
    MOTION_SAFETY_OWNER_WEB,
    MOTION_SAFETY_OWNER_TEST,
} motion_safety_owner_t;

/**
 * @brief 初始化默认上锁的运动安全门
 * @param lease_ms 控制端心跳最大允许间隔，单位 ms
 * @return esp_err_t ESP_OK 表示互斥锁创建成功
 */
esp_err_t motion_safety_init(uint32_t lease_ms);

/**
 * @brief 为指定控制源建立新的独占会话
 * @param owner 请求控制权的来源
 * @param session 输出新会话令牌
 * @return esp_err_t ESP_OK 表示已解锁但仍保持零速
 */
esp_err_t motion_safety_acquire(motion_safety_owner_t owner, uint32_t *session);

/**
 * @brief 刷新控制会话租约并拒绝乱序业务命令
 * @param owner 当前控制来源
 * @param session acquire 返回的会话令牌
 * @param sequence 严格单调递增的业务命令序号；0 仅刷新心跳
 * @return esp_err_t ESP_OK 表示会话仍有效
 */
esp_err_t motion_safety_renew(motion_safety_owner_t owner, uint32_t session, uint32_t sequence);

/**
 * @brief 立即撤销所有者并进入持续零速状态
 * @param reason 供日志定位的静态原因字符串
 */
void motion_safety_lock(const char *reason);

/**
 * @brief 检查租约与会话是否允许生成非零轮速
 * @return bool true 表示当前拥有有效控制租约
 * @note 超时检查在 50 Hz 转发任务中调用，超时会原子上锁。
 */
bool motion_safety_motion_allowed(void);

/**
 * @brief 记录收到一帧有效下位机编码器反馈
 * @note 接收任务每次成功解析反馈后调用；反馈中断会使运动安全门上锁。
 */
void motion_safety_note_feedback(void);

/**
 * @brief 获取当前有效控制来源
 * @return motion_safety_owner_t 租约有效时返回所有者，否则返回 NONE
 */
motion_safety_owner_t motion_safety_owner(void);
