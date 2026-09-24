#ifndef PROTECT_H
#define PROTECT_H

#include <stdbool.h>

typedef enum {
    PROTECT_OK = 0,          // 正常
    PROTECT_RC_LOST,         // 遥控失联
    PROTECT_LOW_BATTERY,     // 低压
    PROTECT_FLIPPED,         // 翻机
    PROTECT_STALL,           // 堵转
} ProtectReason_t;

void Protect_Init(void);
void Protect_Update(void);        // 主循环里定期调用
void Protect_FeedRC(void);        // 每次收到遥控数据时调用（刷新心跳）
bool Protect_Is_Triggered(void);  // 查询是否已触发
ProtectReason_t Protect_GetReason(void); // 获取触发原因

#endif // PROTECT_H