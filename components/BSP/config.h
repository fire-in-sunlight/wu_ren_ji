#ifndef CONFIG_H
#define CONFIG_H

#define motor_1_ledc 1
#define motor_2_ledc 2
#define motor_3_ledc 16
#define motor_4_ledc 17

#define Y0 22
#define Y1 23
#define Y2 25
#define Y3 10
#define Y4 9
#define Y5 11
#define Y6 13
#define Y7 21
#define Y8 47
#define Y9 45

#define ov3660_SDA 42
#define ov3660_SCL 41
#define VSYNC 6
#define HREF 7
#define MCLK 48
#define PCLK 14

#define MPU6050_SDA GPIO_NUM_4
#define MPU6050_SCL GPIO_NUM_5
#define MPU6050_ADDR 0x68 

#define USB_DP 19
#define USB_DN 20

#define TXD TXD0
#define RXD RXD0

#define MOTOR_P 1
#define MOTOR_I 1
#define MOTOR_D 1
#endif