/**
 * @file imu963ra_attitude.c
 * @brief 基于 Fusion 的 IMU963RA 姿态解算、零偏与姿态归零实现
 * @note 适配自 26NUEDC-H，第三方 Fusion 库保持原样并保留 MIT 许可证。
 */
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

/**
 * @brief 计算单位四元数的共轭
 * @param q 输入四元数
 * @return FusionQuaternion 共轭四元数
 */
static FusionQuaternion conjugate(FusionQuaternion q)
{
    q.element.x = -q.element.x;
    q.element.y = -q.element.y;
    q.element.z = -q.element.z;
    return q;
}

/**
 * @brief 重置 Fusion AHRS、参数、零偏和用户归零状态
 */
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

/**
 * @brief 设置静止标定得到的三轴陀螺仪零偏
 * @param x X 轴零偏，单位 deg/s
 * @param y Y 轴零偏，单位 deg/s
 * @param z Z 轴零偏，单位 deg/s
 */
void imu963ra_attitude_set_gyro_bias_dps(float x, float y, float z)
{
    s.bias = (FusionVector){.axis = {.x = x, .y = y, .z = z}};
}

/**
 * @brief 根据静止加速度向量建立水平初始姿态
 * @param ax X 轴加速度，单位 g
 * @param ay Y 轴加速度，单位 g
 * @param az Z 轴加速度，单位 g
 */
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

/**
 * @brief 使用一帧惯性与可选磁场数据更新水平安装姿态
 * @param gx X 轴角速度，单位 deg/s
 * @param gy Y 轴角速度，单位 deg/s
 * @param gz Z 轴角速度，单位 deg/s
 * @param ax X 轴加速度，单位 g
 * @param ay Y 轴加速度，单位 g
 * @param az Z 轴加速度，单位 g
 * @param mx X 轴磁场强度，单位 G
 * @param my Y 轴磁场强度，单位 G
 * @param mz Z 轴磁场强度，单位 G
 * @param mag_valid 是否允许本帧磁场参与融合
 * @param dt 相邻两次更新的实测时间间隔，单位 s
 */
void imu963ra_attitude_update_horizontal(float gx, float gy, float gz,
                                         float ax, float ay, float az,
                                         float mx, float my, float mz,
                                         bool mag_valid, float dt)
{
    if (dt <= 0.0f || dt > 0.2f) return;
    FusionAhrsSetSamplePeriod(&s.ahrs, dt);
    const FusionVector gyro = {.axis = {
        .x = gx - s.bias.axis.x, .y = gy - s.bias.axis.y, .z = gz - s.bias.axis.z}};
    const FusionVector accel = {.axis = {.x = ax, .y = ay, .z = az}};
    if (mag_valid) {
        FusionAhrsUpdate(&s.ahrs, gyro, accel,
                         (FusionVector){.axis = {.x = mx, .y = my, .z = mz}});
    } else {
        FusionAhrsUpdateNoMagnetometer(&s.ahrs, gyro, accel);
    }
}

/**
 * @brief 将当前姿态保存为后续输出的用户零点
 */
void imu963ra_attitude_zero_current(void)
{
    s.zero = conjugate(FusionAhrsGetQuaternion(&s.ahrs));
    s.zero_valid = true;
}

/**
 * @brief 获取应用用户零点后的欧拉角和四元数
 * @param out 姿态结果输出缓冲区
 */
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
