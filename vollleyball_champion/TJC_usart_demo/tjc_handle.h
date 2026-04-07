/*
 * tjc_handle.h
 * 淘晶驰串口屏解析模块
 */

#ifndef __TJC_HANDLE_H
#define __TJC_HANDLE_H

#include "main.h" // 包含HAL库定义
#include "stdio.h"
#include "string.h"

 // 定义最大接收缓冲区长度，根据实际指令长度调整
#define TJC_RX_BUF_SIZE 64

// 定义数据结构体，用于存储解析后的结果
typedef struct {
    float target_x;       // 解析出的X坐标
    float target_y;       // 解析出的Y坐标
    uint8_t cmd_start;    // 收到start指令标志位 (1:收到)
    uint8_t cmd_end;      // 收到end指令标志位 (1:收到)
    uint8_t coords_updated; // 坐标更新标志位 (1:有新坐标)
} TJC_Status_TypeDef;

// 外部可调用的变量
extern TJC_Status_TypeDef TJC_Status;

// 函数声明
void TJC_UART_RxCpltCallback(UART_HandleTypeDef* huart); // 放在HAL_UART_RxCpltCallback中调用
void TJC_Parse_Process(void); // 放在主循环中调用，处理解析逻辑
void TJC_Init(UART_HandleTypeDef* huart); // 初始化接收

#endif