#include "test.h"
#include "config.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include "WiFi.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "TEST";

// ==================== MPU6050 测试 ====================
// 打印实时姿态角，可用手倾斜板子验证
void Test_MPU6050(void)
{
    ESP_LOGI(TAG, "==== MPU6050 Test ====");

    if (MPU6050_Init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed!");
        return;
    }

    MPU6050_Calib_t calib;
    ESP_LOGI(TAG, "Calibrating gyro, keep still...");
    vTaskDelay(pdMS_TO_TICKS(500));
    MPU6050_Calibrate(&calib, 200);
    ESP_LOGI(TAG, "Offsets: gx=%.3f gy=%.3f gz=%.3f",
             calib.gx_offset, calib.gy_offset, calib.gz_offset);

    Attitude_t att = {};
    int64_t last = esp_timer_get_time();

    for (int i = 0; i < 300; i++) {   // 跑 30 秒左右（100Hz × 300）
        MPU6050_RawData_t raw;
        if (MPU6050_ReadRaw(&raw) != ESP_OK) {
            ESP_LOGE(TAG, "ReadRaw failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float ax, ay, az, gx, gy, gz;
        MPU6050_Convert(&raw, &ax, &ay, &az, &gx, &gy, &gz);

        gx -= calib.gx_offset;
        gy -= calib.gy_offset;
        gz -= calib.gz_offset;

        int64_t now = esp_timer_get_time();
        float dt = (now - last) / 1000000.0f;
        last = now;

        Attitude_Update(&att, ax, ay, az, gx, gy, gz, dt);

        printf("Roll=%7.2f  Pitch=%7.2f  |  gx=%6.2f gy=%6.2f gz=%6.2f\n",
               att.roll, att.pitch, gx, gy, gz);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "MPU6050 test done");
}

// ==================== 电机扫描测试 ====================
// 🚨 必须拆桨！🚨
void Test_Motor_Sweep(void)
{
    ESP_LOGI(TAG, "==== Motor Sweep Test ====");
    ESP_LOGW(TAG, "!!! REMOVE PROPELLERS !!!");

    Motor_init();
    Motor_arm_change(true);   // 解锁

    // 从 0 缓慢加速到 50%，再降回 0
    for (int duty = 0; duty <= 512; duty += 16) {
        for (int i = 0; i < 4; i++) {
            Motor_set_speed(i, duty);
        }
        printf("Duty = %d\n", duty);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    for (int duty = 512; duty >= 0; duty -= 16) {
        for (int i = 0; i < 4; i++) {
            Motor_set_speed(i, duty);
        }
        printf("Duty = %d\n", duty);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    Motor_arm_change(false);  // 上锁
    ESP_LOGI(TAG, "Motor sweep test done");
}

// ==================== PID 测试 ====================
// 模拟"目标 100，当前 0"，看输出是否合理
void Test_PID(void)
{
    ESP_LOGI(TAG, "==== PID Test ====");

    PID_t pid;
    PID_init(&pid);

    float measured = 0.0f;
    float target   = 100.0f;
    float dt       = 0.01f;   // 100Hz

    for (int i = 0; i < 50; i++) {
        float out = PID_compute(&pid, target, measured, dt);
        printf("t=%5.2fs  measured=%7.2f  out=%7.2f\n",
               i * dt, measured, out);

        // 简单一阶系统模拟：测量值逐步逼近输出
        measured += out * dt;
    }

    ESP_LOGI(TAG, "PID test done");
}

// ==================== WiFi 测试 ====================
void Test_WiFi(void)
{
    ESP_LOGI(TAG, "==== WiFi Test ====");
    WiFi_Init();

    for (int i = 0; i < 100; i++) {
        RCCommand_t rc = {};
        WiFi_GetRCCommand(&rc);

        printf("connected=%d  arm=%d  thr=%.2f  roll=%.2f pitch=%.2f yaw=%.2f\n",
               WiFi_Is_Connected(), rc.arm,
               rc.throttle, rc.roll, rc.pitch, rc.yaw);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ==================== 综合测试（不含电机）====================
void Test_All(void)
{
    ESP_LOGI(TAG, "==== Full Test (no motor) ====");
    Test_MPU6050();
    Test_PID();
    ESP_LOGI(TAG, "Full test done");
}