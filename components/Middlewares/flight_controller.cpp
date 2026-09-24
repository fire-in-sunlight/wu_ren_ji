#include "flight_controller.h"
#include "config.h"
#include "motor.h"
#include "pid.h"
#include "protect.h"
#include "WiFi.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "FLIGHT";

// ==================== 控制参数 ====================
#define CONTROL_HZ           250                        // 控制频率 250Hz
#define CONTROL_DT           (1.0f / CONTROL_HZ)
#define CONTROL_PERIOD_MS    (1000 / CONTROL_HZ)

#define MOTOR_PWM_MAX        1023                       // 10位 LEDC
#define MOTOR_IDLE_DUTY      50                         // 怠速 5%（可按需关掉）

// PID 参数（先用保守值，慢慢调）
#define ROLL_KP              1.2f
#define ROLL_KI              0.05f
#define ROLL_KD              0.01f

#define PITCH_KP             1.2f
#define PITCH_KI             0.05f
#define PITCH_KD             0.01f

#define YAW_KP               2.0f
#define YAW_KI               0.0f
#define YAW_KD               0.0f

#define INTEGRAL_LIMIT       100.0f
#define OUTPUT_LIMIT         300.0f                     // PID 输出限幅（占空比单位）

// ==================== 静态状态 ====================
static PID_t          g_pid_roll;
static PID_t          g_pid_pitch;
static PID_t          g_pid_yaw;
static MPU6050_Calib_t g_calib;
static Attitude_t     g_attitude;
static FlightState_t  g_state;
static SemaphoreHandle_t g_state_mutex = NULL;

// ==================== 混控（X 型四轴）====================
// 假设：motor0=右前  motor1=左后  motor2=左前  motor3=右后
static void Motor_Mix(float throttle, float roll, float pitch, float yaw)
{
    float m0 = throttle + pitch + roll - yaw;
    float m1 = throttle - pitch - roll - yaw;
    float m2 = throttle + pitch - roll + yaw;
    float m3 = throttle - pitch + roll + yaw;

    float mix[4] = {m0, m1, m2, m3};

    for (int i = 0; i < 4; i++) {
        if (mix[i] < 0.0f) mix[i] = 0.0f;
        if (mix[i] > 1.0f) mix[i] = 1.0f;

        int duty = (int)(mix[i] * MOTOR_PWM_MAX);

        // 解锁后如果非零，抬到怠速（可按需删掉）
        // if (duty > 0 && duty < MOTOR_IDLE_DUTY) duty = MOTOR_IDLE_DUTY;

        Motor_set_speed(i, duty);
    }
}

// ==================== 飞控主任务 ====================
static void flight_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Flight task started on Core %d", xPortGetCoreID());

    // 1. 初始化传感器
    if (MPU6050_Init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed!");
        vTaskDelete(NULL);
        return;
    }

    // 2. 陀螺仪校准（保持静止！）
    ESP_LOGI(TAG, "Calibrating gyro, keep still...");
    vTaskDelay(pdMS_TO_TICKS(500));
    MPU6050_Calibrate(&g_calib, 200);
    ESP_LOGI(TAG, "Gyro offsets: gx=%.3f gy=%.3f gz=%.3f",
             g_calib.gx_offset, g_calib.gy_offset, g_calib.gz_offset);

    // 3. 初始化 PID
    PID_init(&g_pid_roll);
    PID_init(&g_pid_pitch);
    PID_init(&g_pid_yaw);

    // 4. 初始化电机（默认锁定，不动）
    Motor_init();
    Motor_arm_change(false);  // 确保上电锁定

    // 5. 初始化保护
    Protect_Init();

    // 6. 主循环
    int64_t last_time = esp_timer_get_time();
    RCCommand_t rc = {};
    Telemetry_t tlm = {};
    int64_t last_tlm = 0;

    while (1) {
        // ---- 计时 ----
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        if (dt <= 0.0f || dt > 0.1f) dt = CONTROL_DT;   // 保护

        // ---- 1. 读取传感器 ----
        MPU6050_RawData_t raw;
        if (MPU6050_ReadRaw(&raw) == ESP_OK) {
            float ax, ay, az, gx, gy, gz;
            MPU6050_Convert(&raw, &ax, &ay, &az, &gx, &gy, &gz);
            gx -= g_calib.gx_offset;
            gy -= g_calib.gy_offset;
            gz -= g_calib.gz_offset;

            Attitude_Update(&g_attitude, ax, ay, az, gx, gy, gz, dt);
        }

        // ---- 2. 取遥控指令 ----
        WiFi_GetRCCommand(&rc);

        // ---- 3. 处理解锁/上锁 ----
        static uint8_t last_arm = 0;
        if (rc.arm != last_arm) {
            last_arm = rc.arm;
            Motor_arm_change(rc.arm != 0);
            PID_reset(&g_pid_roll);
            PID_reset(&g_pid_pitch);
            PID_reset(&g_pid_yaw);
            ESP_LOGI(TAG, "Motor %s", rc.arm ? "ARMED" : "DISARMED");
        }

        // ---- 4. 只有解锁且遥控连接正常，才输出电机 ----
        if (Motor_is_armed() && !Protect_Is_Triggered()) {
            // 目标：当前角度归零（稳定模式）
            // 遥控的 roll/pitch/yaw 作为目标角度（-1~1 映射到 -30°~30°）
            float target_roll  = rc.roll  * 30.0f;
            float target_pitch = rc.pitch * 30.0f;
            float target_yaw   = rc.yaw   * 30.0f;   // 偏航角速度目标

            float out_roll  = PID_compute(&g_pid_roll,  target_roll,  g_attitude.roll,  dt);
            float out_pitch = PID_compute(&g_pid_pitch, target_pitch, g_attitude.pitch, dt);
            float out_yaw   = PID_compute(&g_pid_yaw,   target_yaw,   0.0f,             dt);

            // 归一化 PID 输出到 -1.0 ~ 1.0
            float r = out_roll  / OUTPUT_LIMIT;
            float p = out_pitch / OUTPUT_LIMIT;
            float y = out_yaw   / OUTPUT_LIMIT;

            Motor_Mix(rc.throttle, r, p, y);
        } else {
            Motor_stop();
        }

        // ---- 5. 安全检测 ----
        if (WiFi_Is_Connected()) {
            Protect_FeedRC();
        }
        Protect_Update();

        // ---- 6. 回传遥测（10Hz）----
        if (now - last_tlm > 100000) {
            last_tlm = now;
            tlm.roll              = g_attitude.roll;
            tlm.pitch             = g_attitude.pitch;
            tlm.yaw               = 0;
            tlm.voltage           = 0;
            tlm.armed             = Motor_is_armed() ? 1 : 0;
            tlm.protect_triggered = Protect_Is_Triggered() ? 1 : 0;
            WiFi_SendTelemetry(&tlm);
        }

        // ---- 7. 更新共享状态（加锁）----
        if (g_state_mutex) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_state.attitude     = g_attitude;
            g_state.throttle     = rc.throttle;
            g_state.target_roll  = rc.roll  * 30.0f;
            g_state.target_pitch = rc.pitch * 30.0f;
            g_state.target_yaw   = rc.yaw   * 30.0f;
            g_state.running      = Motor_is_armed();
            xSemaphoreGive(g_state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// ==================== 对外接口 ====================

void Flight_Init(void)
{
    g_state_mutex = xSemaphoreCreateMutex();
    g_attitude = (Attitude_t){0};
    g_state    = (FlightState_t){0};
}

void Flight_Start(void)
{
    xTaskCreatePinnedToCore(
        flight_task,
        "flight_task",
        8192,
        NULL,
        10,      // 优先级高于 WiFi
        NULL,
        1        // 🎯 固定在 Core 1
    );
}

void Flight_GetAttitude(Attitude_t *out)
{
    if (!out || !g_state_mutex) return;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    *out = g_attitude;
    xSemaphoreGive(g_state_mutex);
}

void Flight_GetState(FlightState_t *out)
{
    if (!out || !g_state_mutex) return;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    *out = g_state;
    xSemaphoreGive(g_state_mutex);
}