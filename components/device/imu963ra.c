#include "imu963ra.h"

#include <stdint.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define REG_WHO_AM_I 0x0f
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G  0x11
#define REG_CTRL3_C  0x12
#define REG_CTRL4_C  0x13
#define REG_CTRL9_XL 0x18
#define REG_OUTX_L_G 0x22

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_address;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(s_dev, data, sizeof(data), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t size)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, size, 100);
}

esp_err_t imu963ra_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_bus));

    for (uint8_t address = 0x6a; address <= 0x6b; ++address) {
        if (i2c_master_probe(s_bus, address, 100) != ESP_OK) continue;
        const i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 400000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &device_config, &s_dev));
        uint8_t id = 0;
        if (read_regs(REG_WHO_AM_I, &id, 1) == ESP_OK && id == 0x6b) {
            s_address = address;
            break;
        }
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (!s_dev) return ESP_ERR_NOT_FOUND;

    ESP_ERROR_CHECK(write_reg(REG_CTRL3_C, 0x01));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(write_reg(REG_CTRL3_C, 0x44)); /* BDU + register increment */
    ESP_ERROR_CHECK(write_reg(REG_CTRL1_XL, 0x4c)); /* 104 Hz, +/-8 g */
    ESP_ERROR_CHECK(write_reg(REG_CTRL2_G, 0x4c));  /* 104 Hz, +/-2000 dps */
    ESP_ERROR_CHECK(write_reg(REG_CTRL4_C, 0x02));
    ESP_ERROR_CHECK(write_reg(REG_CTRL9_XL, 0x01)); /* disable I3C */
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

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
    return ESP_OK;
}

uint8_t imu963ra_address(void) { return s_address; }
