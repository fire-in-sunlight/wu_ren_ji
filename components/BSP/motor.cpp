#include "motor.h"
#include "config.h"
#include "driver/ledc.h"

static bool arm = false;

void Motor_init() {
    int motors[] = {motor_1_ledc, motor_2_ledc, motor_3_ledc, motor_4_ledc};
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = motors[i],
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = static_cast<ledc_channel_t>(i),
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }
}

void Motor_set_speed(int motor_index, int speed) {
    if(!arm) {
        Motor_stop();
        return;
    }

    if (motor_index < 0 || motor_index >= 4) {
        return;
    }
    if (speed < 0) speed = 0;
    if (speed > 1023) speed = 1023; // Assuming 10-bit resolution

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(motor_index), speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(motor_index));
}

void Motor_stop() {
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(i), 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(i));
    }
}

void Motor_arm_change(bool get_arm) {
    arm = get_arm;
    if (!arm) {
        Motor_stop();
    }
}

bool Motor_is_armed() {
    return arm;
}