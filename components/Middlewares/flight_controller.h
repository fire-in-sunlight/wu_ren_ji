#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdbool.h>
#include "mpu6050.h"

// 飞控状态
typedef struct {
    Attitude_t attitude;       // 当前姿态
    float      throttle;       // 0.0 ~ 1.0
    float      target_roll;    // 目标角度（预留）
    float      target_pitch;
    float      target_yaw;
    bool       running;
} FlightState_t;

// 初始化飞控（初始化 MPU6050、PID、电机、保护）
void Flight_Init(void);

// 启动飞控任务（固定 Core 1）
void Flight_Start(void);

// 获取当前姿态（线程安全，后续给 WiFi 用）
void Flight_GetAttitude(Attitude_t *out);

// 获取当前状态
void Flight_GetState(FlightState_t *out);

#endif