#include "stm32_timestamp.h"
#include <stdio.h>

// 1. 获取当前毫秒时间戳 (封装 HAL 库，方便未来移植到其他平台)
uint32_t TimeStamp_Get_ms(void)
{
    return HAL_GetTick();
}

// 2. 计算时间差 (Delta Time)
uint32_t TimeStamp_GetDelta_ms(uint32_t start_time)
{
    // 在 C 语言中，只要两边都是无符号数，减法会自动处理内存溢出回绕
    // 即使 HAL_GetTick() 从 0xFFFFFFFF 溢出回 0，计算结果依然是准确的
    return (HAL_GetTick() - start_time);
}

// 3. 非阻塞超时检测 (状态机核心利器)
bool TimeStamp_CheckTimeout(uint32_t *start_time, uint32_t timeout_ms)
{
    if (TimeStamp_GetDelta_ms(*start_time) >= timeout_ms)
    {
        // 如果超时，立刻更新起始时间，为下一次周期做准备
        *start_time = HAL_GetTick();
        return true;
    }
    return false;
}

// 4. 格式化时间字符串输出 [HH:MM:SS.mmm]
void TimeStamp_GetFormattedStr(char* buffer, uint16_t max_len)
{
    if (buffer == NULL || max_len == 0) return;

    uint32_t current_tick = HAL_GetTick();
    
    // 计算时、分、秒、毫秒
    uint32_t ms  = current_tick % 1000;
    uint32_t sec = (current_tick / 1000) % 60;
    uint32_t min = (current_tick / 60000) % 60;
    uint32_t hr  = (current_tick / 3600000);

    // 格式化输出到传入的 buffer 中
    snprintf(buffer, max_len, "[%02lu:%02lu:%02lu.%03lu]", hr, min, sec, ms);
}