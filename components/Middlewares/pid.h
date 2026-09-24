#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float prev_error;
    float output;
    float output_limit;
} PID_t;

void PID_init(PID_t *pid);
void PID_reset(PID_t *pid);
float PID_compute(PID_t *pid, float setpoint, float measured, float dt);

#endif