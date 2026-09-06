#pragma once

#include "esp_err.h"

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
} imu963ra_sample_t;

esp_err_t imu963ra_init(void);
esp_err_t imu963ra_read(imu963ra_sample_t *sample);
uint8_t imu963ra_address(void);
