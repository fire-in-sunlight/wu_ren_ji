#include "protect.h"
#include "motor.h"
#include "flight_controller.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "PROTECT";

// ==================== 参数 ====================
#define RC_TIMEOUT_US        500000    // 500ms 无信号 → 失联
#define LOW_BATTERY_VOLT     10.5f     // 低压阈值（3S 电池）
#define FLIP_ANGLE_DEG       60.0f     // 倾角超过 60° 视为翻机

// ==================== 状态 ====================
static bool             g_triggered = false;
static ProtectReason_t  g_reason    = PROTECT_OK;
static int64_t          g_last_rc_time = 0;

// ==================== 接口 ====================

void Protect_Init(void)
{
    g_triggered    = false;
    g_reason       = PROTECT_OK;
    g_last_rc_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Protect init");
}

void Protect_FeedRC(void)
{
    // 每次收到遥控数据就刷新心跳
    g_last_rc_time = esp_timer_get_time();
}

void Protect_Update(void)
{
    if (g_triggered) return;  // 已触发就不重复处理

    int64_t now = esp_timer_get_time();

    // ---------- 1. 遥控失联检测 ----------
    if (now - g_last_rc_time > RC_TIMEOUT_US) {
        g_reason    = PROTECT_RC_LOST;
        g_triggered = true;
        Motor_arm_change(false);
        ESP_LOGE(TAG, "RC lost! Disarmed.");
        return;
    }

    // ---------- 2. 低压检测（需要电池模块，暂时注释） ----------
    // if (Battery_GetVoltage() < LOW_BATTERY_VOLT) {
    //     g_reason    = PROTECT_LOW_BATTERY;
    //     g_triggered = true;
    //     Motor_arm_change(false);
    //     ESP_LOGE(TAG, "Low battery! Disarmed.");
    //     return;
    // }

    // ---------- 3. 翻机检测（依赖姿态） ----------
    // Attitude_t att = Flight_GetAttitude();
    // if (fabsf(att.roll) > FLIP_ANGLE_DEG || fabsf(att.pitch) > FLIP_ANGLE_DEG) {
    //     g_reason    = PROTECT_FLIPPED;
    //     g_triggered = true;
    //     Motor_arm_change(false);
    //     ESP_LOGE(TAG, "Flipped! Disarmed.");
    //     return;
    // }
}

bool Protect_Is_Triggered(void)
{
    return g_triggered;
}

ProtectReason_t Protect_GetReason(void)
{
    return g_reason;
}