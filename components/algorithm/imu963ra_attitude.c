/* Adapted from 26NUEDC-H's imu963ra_attitude wrapper.  The Fusion library
 * itself is vendored unchanged under fusion/ and retains its MIT licence. */
#include "imu963ra_attitude.h"

#include <math.h>
#include <stdbool.h>
#include "Fusion.h"

static struct {
    FusionAhrs ahrs;
    FusionQuaternion zero;
    FusionVector bias;
    bool zero_valid;
} s;

static FusionQuaternion conjugate(FusionQuaternion q)
{
    q.element.x = -q.element.x;
    q.element.y = -q.element.y;
    q.element.z = -q.element.z;
    return q;
}

void imu963ra_attitude_reset(void)
{
    const FusionAhrsSettings settings = {
        .sampleRate = 100.0f,
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 500,
    };
    FusionAhrsInitialise(&s.ahrs);
    FusionAhrsSetSettings(&s.ahrs, &settings);
    s.zero = FUSION_QUATERNION_IDENTITY;
    s.bias = FUSION_VECTOR_ZERO;
    s.zero_valid = false;
}

void imu963ra_attitude_set_gyro_bias_dps(float x, float y, float z)
{
    s.bias = (FusionVector){.axis = {.x = x, .y = y, .z = z}};
}

void imu963ra_attitude_init_horizontal(float ax, float ay, float az)
{
    imu963ra_attitude_reset();
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    FusionAhrsSetQuaternion(&s.ahrs, FusionQuaternionNormalise((FusionQuaternion){
        .element = {.w = cr * cp, .x = sr * cp, .y = cr * sp, .z = -sr * sp}
    }));
}

void imu963ra_attitude_update_horizontal(float gx, float gy, float gz,
                                         float ax, float ay, float az, float dt)
{
    if (dt <= 0.0f || dt > 0.2f) return;
    FusionAhrsSetSamplePeriod(&s.ahrs, dt);
    FusionAhrsUpdateNoMagnetometer(&s.ahrs,
        (FusionVector){.axis = {.x = gx - s.bias.axis.x, .y = gy - s.bias.axis.y, .z = gz - s.bias.axis.z}},
        (FusionVector){.axis = {.x = ax, .y = ay, .z = az}});
}

void imu963ra_attitude_zero_current(void)
{
    s.zero = conjugate(FusionAhrsGetQuaternion(&s.ahrs));
    s.zero_valid = true;
}

void imu963ra_attitude_get(imu963ra_attitude_t *out)
{
    if (!out) return;
    FusionQuaternion q = FusionAhrsGetQuaternion(&s.ahrs);
    if (s.zero_valid) q = FusionQuaternionProduct(s.zero, q);
    q = FusionQuaternionNormalise(q);
    const FusionEuler e = FusionQuaternionToEuler(q);
    *out = (imu963ra_attitude_t){
        .roll_deg = -e.angle.roll, .pitch_deg = e.angle.pitch, .yaw_deg = e.angle.yaw,
        .quaternion_w = q.element.w, .quaternion_x = q.element.x,
        .quaternion_y = q.element.y, .quaternion_z = q.element.z,
    };
}
