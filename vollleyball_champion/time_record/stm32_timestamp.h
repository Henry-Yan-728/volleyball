#ifndef STM32_TIMESTAMP_H
#define STM32_TIMESTAMP_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================
 * 1. 基础时间获取与计算
 * ========================================================== */

/**
 * @brief  获取系统当前时间戳 (毫秒)
 * @return 自系统启动以来的毫秒数
 */
uint32_t TimeStamp_Get_ms(void);

/**
 * @brief  计算距离指定时间点经过了多少毫秒
 * @note   利用无符号整数回绕特性，完美解决了 49.7 天后 HAL_GetTick() 溢出的问题
 * @param  start_time: 起始时间戳
 * @return 经过的毫秒差值 (Delta Time)
 */
uint32_t TimeStamp_GetDelta_ms(uint32_t start_time);


/* ==========================================================
 * 2. 非阻塞定时器 / 状态机超时检测
 * ========================================================== */

/**
 * @brief  非阻塞超时检测 (常用于 RTOS 或裸机大循环中的定时任务)
 * @param  start_time: 指向记录上一次触发时间的变量指针
 * @param  timeout_ms: 设定的超时周期 (毫秒)
 * @retval true:  已经超时 (会自动将 start_time 更新为当前时间)
 * @retval false: 尚未超时
 */
bool TimeStamp_CheckTimeout(uint32_t *start_time, uint32_t timeout_ms);


/* ==========================================================
 * 3. 格式化输出 (适配之前的多窗口串口调试助手)
 * ========================================================== */

/**
 * @brief  获取系统运行时间的格式化字符串 (常用于给日志打时间戳)
 * @param  buffer: 用于接收字符串的缓冲区
 * @param  max_len: 缓冲区的最大长度 (建议至少 16 字节)
 * @note   输出格式示例: "[00:01:23.456]" (时:分:秒.毫秒)
 */
void TimeStamp_GetFormattedStr(char* buffer, uint16_t max_len);

#endif /* STM32_TIMESTAMP_H */