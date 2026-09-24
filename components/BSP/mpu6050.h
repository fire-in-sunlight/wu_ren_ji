#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include "driver/i2c_master.h"   // ← 放在 extern "C" 外面

typedef struct {
    int16_t ax, ay, az;   // 加速度计原始值
    int16_t gx, gy, gz;   // 陀螺仪原始值
    int16_t temp;         // 温度原始值
} MPU6050_RawData_t;

// 零偏校准值
typedef struct {
    float gx_offset;
    float gy_offset;
    float gz_offset;
} MPU6050_Calib_t;

// 姿态角
typedef struct {
    float roll;
    float pitch;
} Attitude_t;

esp_err_t MPU6050_Init(void);

esp_err_t MPU6050_ReadRaw(MPU6050_RawData_t *raw);

void MPU6050_Convert(const MPU6050_RawData_t *raw,
                     float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz);

esp_err_t MPU6050_Calibrate(MPU6050_Calib_t *calib, int samples);

void Attitude_Update(Attitude_t *att,
                     float ax, float ay, float az,
                     float gx, float gy, float gz, float dt);

#endif