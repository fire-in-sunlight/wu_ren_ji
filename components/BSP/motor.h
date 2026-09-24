#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

void Motor_init();
void Motor_set_speed(int motor_index, int speed);
void Motor_stop();
void Motor_arm_change(bool get_arm);
bool Motor_is_armed();

#endif