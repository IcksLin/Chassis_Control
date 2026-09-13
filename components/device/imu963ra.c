/**
 * @file imu963ra.c
 * @brief IMU963RA 六轴传感器与板载磁力计硬件 I2C 驱动实现
 * @note 板载磁力计（QMC5883L 兼容）位于 LSM6DSR 辅助 I2C 总线，主机无法直接
 *       访问。初始化先用 pass-through 校验并配置磁力计，随后关闭 pass-through
 *       并交由 Sensor Hub 连续搬运；采样循环只读取 SENSOR_HUB 结果寄存器，
 *       不在运行期直接操作辅助总线。寄存器序列参考 26NUEDC-H 实现。
 */
#include "imu963ra.h"

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9

/* LSM6DSR 主寄存器页寄存器。 */
#define REG_WHO_AM_I 0x0f
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G  0x11
#define REG_CTRL3_C  0x12
#define REG_CTRL4_C  0x13
#define REG_CTRL9_XL 0x18
#define REG_INT1_CTRL 0x0d
#define REG_OUTX_L_G 0x22

/* LSM6DSR Sensor Hub 寄存器页，访问前需将 FUNC_CFG_ACCESS 的 SHUB 位置 1。 */
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_SENSOR_HUB_1 0x02
#define REG_MASTER_CONFIG 0x14
#define REG_SLV0_ADD 0x15
#define REG_SLV0_SUBADD 0x16
#define REG_SLV0_CONFIG 0x17
#define PAGE_MAIN 0x00
#define HUB_PAGE_ACCESS 0x40

/* 板载 QMC5883L 兼容磁力计寄存器与配置值。 */
#define MAG_ADDRESS 0x0d
#define MAG_REG_DATA 0x00
#define MAG_REG_CONTROL1 0x09
#define MAG_REG_CONTROL2 0x0a
#define MAG_REG_SET_RESET 0x0b
#define MAG_REG_CHIP_ID 0x0d
#define MAG_CHIP_ID 0xff
#define MAG_8G_100HZ 0x19

/* MASTER_CONFIG 位定义：aux_sens_on[1:0] | master_on(2) | shub_pu_en(3) |
 * pass_through(4) | start_config(5) | write_once(6) | rst_master_regs(7)。 */
#define MASTER_OFF 0x00
#define MASTER_PASS_THROUGH 0x18
#define MASTER_RESET 0x80
#define MASTER_DRDY_CONTINUOUS 0x4c
#define SLV0_AUTO_LEN6 0x06

/* 磁力计探测窗口需覆盖模块上电与断电恢复时间。 */
#define MAG_PROBE_RETRY 100
#define MAG_PROBE_INTERVAL_MS 10

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static i2c_master_dev_handle_t s_mag_dev;
static uint8_t s_address;
static bool s_mag_available;
static bool s_mag_has_sample;
static float s_last_mag_gauss[3];
static const char *TAG = "imu963ra";

/**
 * @brief 向 LSM6DSR 当前选中的寄存器页写入一个字节
 * @param reg 寄存器地址
 * @param value 待写入的数据
 * @return esp_err_t ESP_OK 表示写入成功，其他值表示总线错误
 * @note 每次访问最多重试三次，以容忍上电初期的瞬态错误。
 */
static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    esp_err_t err = ESP_FAIL;
    for (int retry = 0; retry < 3; ++retry) {
        err = i2c_master_transmit(s_dev, data, sizeof(data), 100);
        if (err == ESP_OK) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return err;
}

/**
 * @brief 从 LSM6DSR 当前选中的寄存器页连续读取一组寄存器
 * @param reg 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 读取字节数
 * @return esp_err_t ESP_OK 表示读取成功，其他值表示总线错误
 */
static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t size)
{
    esp_err_t err = ESP_FAIL;
    for (int retry = 0; retry < 3; ++retry) {
        err = i2c_master_transmit_receive(s_dev, &reg, 1, data, size, 100);
        if (err == ESP_OK) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return err;
}

/**
 * @brief 选择 Sensor Hub 寄存器页
 * @return esp_err_t ESP_OK 表示寄存器页切换命令写入成功
 * @note 调用方负责在结束后恢复主寄存器页。
 */
static esp_err_t hub_open(void) { return write_reg(REG_FUNC_CFG_ACCESS, HUB_PAGE_ACCESS); }

/**
 * @brief 恢复主寄存器页，保证六轴寄存器可正常访问
 * @return esp_err_t ESP_OK 表示寄存器页切换命令写入成功
 */
static esp_err_t hub_close(void) { return write_reg(REG_FUNC_CFG_ACCESS, PAGE_MAIN); }

/**
 * @brief 在 pass-through 模式下写磁力计寄存器
 * @param reg 磁力计寄存器地址
 * @param value 待写入的数据
 * @return esp_err_t ESP_OK 表示写入成功
 * @note 必须在 pass-through 已开启时调用。
 */
static esp_err_t mag_write_direct(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(s_mag_dev, data, sizeof(data), 100);
}

/**
 * @brief 在 pass-through 模式下读磁力计寄存器
 * @param reg 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 读取字节数
 * @return esp_err_t ESP_OK 表示读取成功
 * @note 必须在 pass-through 已开启时调用。
 */
static esp_err_t mag_read_direct(uint8_t reg, uint8_t *data, size_t size)
{
    return i2c_master_transmit_receive(s_mag_dev, &reg, 1, data, size, 100);
}

/**
 * @brief 复位 Sensor Hub 主机并清除上次运行残留的从机配置
 * @return esp_err_t ESP_OK 表示复位时序完成
 * @note 无论复位是否成功都会恢复主寄存器页。
 */
static esp_err_t hub_reset_master(void)
{
    esp_err_t err = hub_open();
    if (err != ESP_OK) return err;
    err = write_reg(REG_MASTER_CONFIG, MASTER_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (err == ESP_OK) err = write_reg(REG_MASTER_CONFIG, MASTER_OFF);
    vTaskDelay(pdMS_TO_TICKS(2));
    (void)hub_close();
    return err;
}

/**
 * @brief 初始化板载磁力计并启动 Sensor Hub 连续采样
 * @return esp_err_t ESP_OK 表示磁力计 ID 与配置均有效，其他值表示不可用
 * @details 开启 pass-through 后直连校验 QMC5883L 芯片 ID（0x0D 期望 0xFF）
 *          并完成复位与量程配置，再关闭 pass-through，改为 Sensor Hub 连续
 *          采样：0x4C 由加速度计数据就绪触发，不依赖外部 INT2 引脚。
 * @note 必须在 LSM6DSR 加速度计 ODR 设置之后调用，Sensor Hub 时钟由 ODR 提供；
 *       所有退出路径都会恢复主寄存器页，磁力计失败不影响六轴采样。
 */
static esp_err_t hub_configure_magnetometer(void)
{
    esp_err_t err = hub_reset_master();
    if (err != ESP_OK) return err;

    /* 第一步：pass-through 直连探测，区分“内部磁力计/连线异常”与主机触发异常。 */
    err = hub_open();
    if (err == ESP_OK) err = write_reg(REG_MASTER_CONFIG, MASTER_PASS_THROUGH);
    (void)hub_close();
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));
    /* pass-through 将辅助总线桥接到主机总线，这里先做总线恢复，释放可能
     * 停在事务中间的磁力计（ESP32 软复位而模块未断电时的常见情况）。 */
    (void)i2c_master_bus_reset(s_bus);
    vTaskDelay(pdMS_TO_TICKS(5));

    esp_err_t probe = ESP_ERR_NOT_FOUND;
    for (int retry = 0; retry < MAG_PROBE_RETRY && probe != ESP_OK; ++retry) {
        probe = i2c_master_probe(s_bus, MAG_ADDRESS, 100);
        if (probe != ESP_OK) vTaskDelay(pdMS_TO_TICKS(MAG_PROBE_INTERVAL_MS));
    }
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "magnetometer did not acknowledge in pass-through mode");
        return ESP_ERR_NOT_FOUND;
    }
    if (!s_mag_dev) {
        const i2c_device_config_t mag_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = MAG_ADDRESS,
            .scl_speed_hz = 100000,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &mag_config, &s_mag_dev),
                            TAG, "add pass-through magnetometer");
    }
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(mag_read_direct(MAG_REG_CHIP_ID, &id, 1), TAG, "read magnetometer id");
    if (id != MAG_CHIP_ID) {
        ESP_LOGW(TAG, "magnetometer id mismatch: 0x%02x", id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "magnetometer pass-through id: 0x%02x", id);
    ESP_RETURN_ON_ERROR(mag_write_direct(MAG_REG_CONTROL2, 0x80), TAG, "reset magnetometer");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(mag_write_direct(MAG_REG_CONTROL2, 0x00), TAG, "release magnetometer reset");
    ESP_RETURN_ON_ERROR(mag_write_direct(MAG_REG_CONTROL1, MAG_8G_100HZ), TAG, "configure magnetometer");
    ESP_RETURN_ON_ERROR(mag_write_direct(MAG_REG_SET_RESET, 0x01), TAG, "set magnetometer reset period");

    /* 第二步：关闭 pass-through，Sensor Hub 每周期自动搬运 6 字节磁场数据。 */
    err = hub_open();
    if (err == ESP_OK) err = write_reg(REG_MASTER_CONFIG, MASTER_OFF);
    if (err == ESP_OK) err = write_reg(REG_SLV0_ADD, (uint8_t)((MAG_ADDRESS << 1) | 0x01));
    if (err == ESP_OK) err = write_reg(REG_SLV0_SUBADD, MAG_REG_DATA);
    if (err == ESP_OK) err = write_reg(REG_SLV0_CONFIG, SLV0_AUTO_LEN6);
    if (err == ESP_OK) err = write_reg(REG_MASTER_CONFIG, MASTER_DRDY_CONTINUOUS);
    (void)hub_close();
    /* 给 Sensor Hub 至少一个 ODR 周期建立首帧，避免启动阶段读到全 0/0xff。 */
    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

/**
 * @brief 初始化 IMU963RA 主器件、六轴采样与板载磁力计
 * @return esp_err_t ESP_OK 表示主 IMU 初始化成功
 * @note 首次总线访问前等待 200 ms，并执行 I2C 总线恢复及地址探测重试；
 *       磁力计不可用时仅记录日志，不阻塞六轴启动。
 */
esp_err_t imu963ra_init(void)
{
    /* 首次总线访问前的上电与数据访问保护延时。 */
    vTaskDelay(pdMS_TO_TICKS(200));
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus), TAG, "create I2C bus");
    /* 软件复位而模块未断电时，从机可能停在事务中间，先恢复总线再探测。 */
    ESP_RETURN_ON_ERROR(i2c_master_bus_reset(s_bus), TAG, "reset I2C bus");
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int retry = 0; retry < 30 && !s_dev; ++retry) {
        for (uint8_t address = 0x6a; address <= 0x6b; ++address) {
            if (i2c_master_probe(s_bus, address, 100) != ESP_OK) continue;
            const i2c_device_config_t device_config = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = address,
                .scl_speed_hz = 400000,
            };
            ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &device_config, &s_dev),
                                TAG, "add LSM6DSR device");
            uint8_t id = 0;
            if (read_regs(REG_WHO_AM_I, &id, 1) == ESP_OK && id == 0x6b) {
                s_address = address;
                break;
            }
            i2c_master_bus_rm_device(s_dev);
            s_dev = NULL;
        }
        if (!s_dev) vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_dev) return ESP_ERR_NOT_FOUND;

    /* 复位前强制切回主寄存器页；断电或上次运行可能停留在嵌入式页。 */
    ESP_RETURN_ON_ERROR(hub_close(), TAG, "select main page");
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL3_C, 0x01), TAG, "reset IMU");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL3_C, 0x44), TAG, "enable BDU"); /* BDU + 寄存器自增 */
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL1_XL, 0x4c), TAG, "configure accel"); /* 104 Hz, ±8 g */
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL2_G, 0x4c), TAG, "configure gyro");  /* 104 Hz, ±2000 dps */
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL4_C, 0x02), TAG, "configure filter");
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL9_XL, 0x01), TAG, "disable I3C");
    ESP_RETURN_ON_ERROR(write_reg(REG_INT1_CTRL, 0x03), TAG, "enable data-ready");
    vTaskDelay(pdMS_TO_TICKS(20)); /* Sensor Hub 时钟由加速度计 ODR 提供 */

    /* 磁力计为可选外设：初始化失败仅记录日志，六轴采样保持可用。 */
    s_mag_available = hub_configure_magnetometer() == ESP_OK;
    ESP_LOGI(TAG, "magnetometer: %s", s_mag_available ? "ready" : "unavailable");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

/**
 * @brief 读取一次加速度、角速度和最近有效磁场数据
 * @param sample 输出的物理量样本，单位分别为 g、deg/s 和 G
 * @return esp_err_t ESP_OK 表示六轴数据读取成功
 * @note 磁场本周期未更新时保留上一帧，避免有效标志在网页端闪烁。
 */
esp_err_t imu963ra_read(imu963ra_sample_t *sample)
{
    if (!sample || !s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t raw[12];
    esp_err_t err = read_regs(REG_OUTX_L_G, raw, sizeof(raw));
    if (err != ESP_OK) return err;
    for (int axis = 0; axis < 3; ++axis) {
        int16_t gyro = (int16_t)((uint16_t)raw[axis * 2] | ((uint16_t)raw[axis * 2 + 1] << 8));
        int16_t accel = (int16_t)((uint16_t)raw[6 + axis * 2] | ((uint16_t)raw[7 + axis * 2] << 8));
        sample->gyro_dps[axis] = gyro / 14.3f;
        sample->accel_g[axis] = accel / 4098.0f;
    }
    sample->mag_valid = s_mag_has_sample;
    for (int axis = 0; axis < 3; ++axis) {
        sample->mag_gauss[axis] = s_last_mag_gauss[axis];
    }
    /* Sensor Hub 自主填充 SENSOR_HUB_1..6，采样循环只读结果，不发起辅助总线事务。 */
    if (s_mag_available && hub_open() == ESP_OK) {
        uint8_t mag[6];
        if (read_regs(REG_SENSOR_HUB_1, mag, sizeof(mag)) == ESP_OK) {
            for (int axis = 0; axis < 3; ++axis) {
                int16_t value = (int16_t)((uint16_t)mag[axis * 2] |
                                          ((uint16_t)mag[axis * 2 + 1] << 8));
                sample->mag_gauss[axis] = value / 3000.0f;
                s_last_mag_gauss[axis] = sample->mag_gauss[axis];
            }
            s_mag_has_sample = true;
            sample->mag_valid = true;
        }
        (void)hub_close();
    }
    return ESP_OK;
}

/**
 * @brief 获取初始化时识别到的 LSM6DSR 七位 I2C 地址
 * @return uint8_t I2C 地址，通常为 0x6A 或 0x6B
 */
uint8_t imu963ra_address(void) { return s_address; }
