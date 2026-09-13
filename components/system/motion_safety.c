/**
 * @file motion_safety.c
 * @brief 通过所有者锁、会话代次、序号和租约阻断失联后的陈旧运动命令
 */
#include "motion_safety.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "motion_safety";
static SemaphoreHandle_t s_lock;
static motion_safety_owner_t s_owner;
static uint32_t s_session;
static uint32_t s_last_sequence;
static int64_t s_deadline_us;
static int64_t s_lease_us;
static int64_t s_last_feedback_us;

#define FEEDBACK_TIMEOUT_US 1500000LL

esp_err_t motion_safety_init(uint32_t lease_ms)
{
    if (lease_ms == 0) return ESP_ERR_INVALID_ARG;
    if (s_lock) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    s_owner = MOTION_SAFETY_OWNER_NONE;
    s_session = 0;
    s_last_sequence = 0;
    s_deadline_us = 0;
    s_lease_us = (int64_t)lease_ms * 1000LL;
    s_last_feedback_us = 0;
    return ESP_OK;
}

esp_err_t motion_safety_acquire(motion_safety_owner_t owner, uint32_t *session)
{
    if (!s_lock || !session || owner == MOTION_SAFETY_OWNER_NONE) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_owner != MOTION_SAFETY_OWNER_NONE && s_owner != owner) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (++s_session == 0) ++s_session;
    s_owner = owner;
    s_last_sequence = 0;
    s_deadline_us = esp_timer_get_time() + s_lease_us;
    *session = s_session;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t motion_safety_renew(motion_safety_owner_t owner, uint32_t session, uint32_t sequence)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const int64_t now = esp_timer_get_time();
    if (owner != s_owner || session != s_session || now > s_deadline_us ||
        (sequence != 0 && sequence <= s_last_sequence)) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (sequence != 0) s_last_sequence = sequence;
    s_deadline_us = now + s_lease_us;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

void motion_safety_lock(const char *reason)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool changed = s_owner != MOTION_SAFETY_OWNER_NONE;
    s_owner = MOTION_SAFETY_OWNER_NONE;
    s_last_sequence = 0;
    s_deadline_us = 0;
    xSemaphoreGive(s_lock);
    if (changed) ESP_LOGW(TAG, "motion locked: %s", reason ? reason : "unspecified");
}

bool motion_safety_motion_allowed(void)
{
    if (!s_lock) return false;
    bool lease_expired = false;
    bool feedback_expired = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool allowed = s_owner != MOTION_SAFETY_OWNER_NONE;
    const int64_t now = esp_timer_get_time();
    if (allowed && now > s_deadline_us) {
        lease_expired = true;
    } else if (allowed && (s_last_feedback_us == 0 || now - s_last_feedback_us > FEEDBACK_TIMEOUT_US)) {
        feedback_expired = true;
    }
    if (lease_expired || feedback_expired) {
        s_owner = MOTION_SAFETY_OWNER_NONE;
        s_last_sequence = 0;
        s_deadline_us = 0;
        allowed = false;
    }
    xSemaphoreGive(s_lock);
    if (lease_expired) ESP_LOGE(TAG, "control lease expired; forcing continuous zero speed");
    if (feedback_expired) ESP_LOGE(TAG, "motor feedback expired; forcing continuous zero speed");
    return allowed;
}

void motion_safety_note_feedback(void)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_last_feedback_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
}

motion_safety_owner_t motion_safety_owner(void)
{
    if (!motion_safety_motion_allowed()) return MOTION_SAFETY_OWNER_NONE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const motion_safety_owner_t owner = s_owner;
    xSemaphoreGive(s_lock);
    return owner;
}
