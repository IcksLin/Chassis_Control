#pragma once

#include <stdint.h>

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
} imu963ra_attitude_t;

void imu963ra_attitude_reset(void);
void imu963ra_attitude_set_gyro_bias_dps(float x, float y, float z);
void imu963ra_attitude_init_horizontal(float ax, float ay, float az);
void imu963ra_attitude_update_horizontal(float gx, float gy, float gz,
                                         float ax, float ay, float az, float dt);
void imu963ra_attitude_zero_current(void);
void imu963ra_attitude_get(imu963ra_attitude_t *out);
