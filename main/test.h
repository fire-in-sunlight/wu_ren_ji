#ifndef TEST_H
#define TEST_H

// 各个模块的独立测试函数
void Test_MPU6050(void);      // 传感器：打印 Roll/Pitch
void Test_Motor_Sweep(void);  // 电机：四路缓慢扫描（必须拆桨！）
void Test_PID(void);          // PID：模拟输入输出
void Test_WiFi(void);         // WiFi：打印连接状态和收到的 RC
void Test_All(void);          // 依次跑 MPU6050 + PID，不含电机

#endif // TEST_H