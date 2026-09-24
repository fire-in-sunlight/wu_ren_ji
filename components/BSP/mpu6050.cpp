// mpu6050.cpp
#include "mpu6050.h"
#include "config.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static i2c_master_bus_handle_t  g_bus_handle = NULL;
static i2c_master_dev_handle_t  g_dev_handle = NULL;

static esp_err_t i2c_master_init(void)
{
    // 1. 配置 I2C 主机总线
    i2c_master_bus_config_t bus_config = {
        .i2c_port      = I2C_NUM_0,
        .sda_io_num    = MPU6050_SDA,
        .scl_io_num    = MPU6050_SCL,
        .clk_source    = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    bus_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &g_bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    =100000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_bus_handle, &dev_config, &g_dev_handle));

    return ESP_OK;
}

static esp_err_t mpu_read_reg(uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(g_dev_handle, &reg, 1, data, 1, 1000);
}

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(g_dev_handle, buf, 2, 1000);
}

esp_err_t MPU6050_Init(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    // WHO_AM_I 验证
    uint8_t whoami = 0;
    ESP_ERROR_CHECK(mpu_read_reg(0x75, &whoami));
    if (whoami != 0x68) {
        return ESP_FAIL;
    }

    // 复位
    mpu_write_reg(0x6B, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write_reg(0x6B, 0x01);
    mpu_write_reg(0x19, 4);
    mpu_write_reg(0x1A, 0x03);
    mpu_write_reg(0x1B, 0x18);
    mpu_write_reg(0x1C, 0x00);

    return ESP_OK;
}

esp_err_t MPU6050_ReadRaw(MPU6050_RawData_t *raw)
{
    uint8_t buf[14];
    uint8_t reg = 0x3B;

    esp_err_t ret = i2c_master_transmit_receive(g_dev_handle, &reg, 1, buf, 14, 1000);
    if (ret != ESP_OK) return ret;

    // 高位在前，拼接成 16 位有符号整数
    raw->ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->az   = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

void MPU6050_Convert(const MPU6050_RawData_t *raw,
                     float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz)
{
    *ax = raw->ax / 16384.0f;   // 单位: g
    *ay = raw->ay / 16384.0f;
    *az = raw->az / 16384.0f;

    *gx = raw->gx / 16.4f;      // 单位: °/s
    *gy = raw->gy / 16.4f;
    *gz = raw->gz / 16.4f;
}

esp_err_t MPU6050_Calibrate(MPU6050_Calib_t *calib, int samples)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < samples; i++) {
        MPU6050_RawData_t raw;
        esp_err_t ret = MPU6050_ReadRaw(&raw);
        if (ret != ESP_OK) return ret;

        sum_gx += raw.gx;
        sum_gy += raw.gy;
        sum_gz += raw.gz;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    calib->gx_offset = sum_gx / samples / 16.4f;
    calib->gy_offset = sum_gy / samples / 16.4f;
    calib->gz_offset = sum_gz / samples / 16.4f;

    return ESP_OK;
}

void Attitude_Update(Attitude_t *att,
                     float ax, float ay, float az,
                     float gx, float gy, float gz, float dt)
{
    float accel_roll  = atan2f(ay, az) * 57.2958f;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;

    att->roll  += gx * dt;
    att->pitch += gy * dt;

    att->roll  = 0.98f * att->roll  + 0.02f * accel_roll;
    att->pitch = 0.98f * att->pitch + 0.02f * accel_pitch;
}