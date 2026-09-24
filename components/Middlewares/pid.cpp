#include "pid.h"
#include "config.h"

void PID_init(PID_t *pid) {
    pid->kp = MOTOR_P;
    pid->ki = MOTOR_I;
    pid->kd = MOTOR_D;
    pid->integral = 0.0f;
    pid->integral_limit = 100.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->output_limit = 100.0f;
}

void PID_reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

float PID_compute(PID_t *pid, float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return pid->output;
    float error = setpoint - measured;
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float derivative = (error - pid->prev_error) / dt;

    pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    if (pid->output >  pid->output_limit) pid->output =  pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    pid->prev_error = error;

    return pid->output;
}