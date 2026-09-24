#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include <stdint.h>

// ===== 遥控指令结构（地面站 → 无人机）=====
typedef struct {
    float throttle;   // 0.0 ~ 1.0
    float roll;       // -1.0 ~ 1.0
    float pitch;      // -1.0 ~ 1.0
    float yaw;        // -1.0 ~ 1.0
    uint8_t arm;      // 0=锁定 1=解锁
} RCCommand_t;

// ===== 遥测结构（无人机 → 地面站）=====
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float voltage;
    uint8_t armed;
    uint8_t protect_triggered;
} Telemetry_t;

// 初始化 WiFi（STA 模式，连路由器）
void WiFi_Init(void);

// 查询连接状态
bool WiFi_Is_Connected(void);

// 获取最近一次收到的遥控指令（线程安全）
bool WiFi_GetRCCommand(RCCommand_t *out);

// 发送遥测数据
void WiFi_SendTelemetry(const Telemetry_t *tlm);

#endif