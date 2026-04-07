#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include "main.h" // 包含 HAL 库头文件

// 初始化，传入你想用来打印的串口句柄 (例如 &huart1)
void Debug_Init(UART_HandleTypeDef *huart);

// 发送普通日志文本 (对应上位机 LOG 窗口)
void Debug_Log(const char* msg);

// 发送格式化变量到指定窗口 (上位机会自动创建对应名称的窗口)
// 例如: Debug_SendVar("PITCH_SPEED", 500.0f);
void Debug_SendVar(const char* channel_name, float value);

// 发送任意格式的字符串到指定窗口
void Debug_SendStr(const char* channel_name, const char* str);

#endif