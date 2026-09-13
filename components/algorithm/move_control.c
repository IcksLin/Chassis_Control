/**
 * @file move_control.c
 * @brief 四轮 45 度全向底盘的矢量合成实现
 */
#include "move_control.h"
#include <math.h>

#define HEADING_KP_DEFAULT 0.040f
#define HEADING_KD_DEFAULT 0.006f
#define HEADING_MAX_ROTATION 0.35f
#define COMMAND_DEADBAND 0.03f
/* 坡度前馈需依据实车正负方向标定；默认关闭，避免未经标定的错误补偿。 */
#define PITCH_SLOPE_FEEDFORWARD_DEFAULT 0.0f
#define ROLL_SLOPE_FEEDFORWARD_DEFAULT 0.0f

static struct { float vx, vy, rotation, reference_yaw; int16_t max_speed; bool active; } s_control;
static struct { float kp, kd, pitch, roll; } s_tuning;

static float wrap_degrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void move_control_init(void)
{
    s_control = (typeof(s_control)){0};
    s_tuning = (typeof(s_tuning)){HEADING_KP_DEFAULT, HEADING_KD_DEFAULT,
                                  PITCH_SLOPE_FEEDFORWARD_DEFAULT, ROLL_SLOPE_FEEDFORWARD_DEFAULT};
}

bool move_control_set_tuning(float kp, float kd, float pitch_feedforward, float roll_feedforward)
{
    if (kp < 0.0f || kp > 0.20f || kd < 0.0f || kd > 0.05f || fabsf(pitch_feedforward) > 0.50f || fabsf(roll_feedforward) > 0.50f) return false;
    s_tuning = (typeof(s_tuning)){kp, kd, pitch_feedforward, roll_feedforward};
    return true;
}

void move_control_set_command(float vx, float vy, float rotation, float yaw_deg, int16_t max_speed)
{
    const bool moving = fabsf(vx) > COMMAND_DEADBAND || fabsf(vy) > COMMAND_DEADBAND || fabsf(rotation) > COMMAND_DEADBAND;
    if (!moving) { s_control.active = false; return; }
    if (!s_control.active || fabsf(rotation) > COMMAND_DEADBAND) s_control.reference_yaw = yaw_deg;
    s_control.vx = vx; s_control.vy = vy; s_control.rotation = rotation;
    s_control.max_speed = max_speed; s_control.active = true;
}

void move_control_set_heading_target(float yaw_deg, int16_t max_speed)
{
    s_control.vx = 0.0f;
    s_control.vy = 0.0f;
    s_control.rotation = 0.0f;
    s_control.reference_yaw = wrap_degrees(yaw_deg);
    s_control.max_speed = max_speed;
    s_control.active = true;
}

bool move_control_update(float yaw_deg, float gyro_z_dps, float roll_deg, float pitch_deg,
                         chassis_wheel_speeds_t *out)
{
    if (!out || !s_control.active) return false;
    float rotation = s_control.rotation;
    if (fabsf(rotation) <= COMMAND_DEADBAND) {
        rotation = s_tuning.kp * wrap_degrees(s_control.reference_yaw - yaw_deg)
                   - s_tuning.kd * gyro_z_dps;
        if (rotation > HEADING_MAX_ROTATION) rotation = HEADING_MAX_ROTATION;
        if (rotation < -HEADING_MAX_ROTATION) rotation = -HEADING_MAX_ROTATION;
    }
    const float vx = s_control.vx + s_tuning.pitch * sinf(pitch_deg * 0.0174532925f);
    const float vy = s_control.vy - s_tuning.roll * sinf(roll_deg * 0.0174532925f);
    move_control_vector_to_wheels(vx, vy, rotation, s_control.max_speed, out);
    return true;
}

bool move_control_is_active(void) { return s_control.active; }
void move_control_vector_to_wheels(float vx, float vy, float w, int16_t max_speed,
                                   chassis_wheel_speeds_t *out)
{
    if (!out || max_speed <= 0) return;
    const float wheel_vector[CHASSIS_WHEEL_COUNT] = {
        vx - vy - w, vx + vy - w, vx + vy + w, vx - vy + w};
    float peak = 0.0f;
    for (int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) peak = fmaxf(peak, fabsf(wheel_vector[i]));
    const float scale = peak > 1.0f ? (float)max_speed / peak : (float)max_speed;
    for (int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) out->speed_mm_s[i] = (int16_t)lroundf(wheel_vector[i] * scale);
}
