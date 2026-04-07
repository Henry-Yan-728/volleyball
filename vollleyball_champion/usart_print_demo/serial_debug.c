#include "serial_debug.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// 内部保存串口句柄指针
static UART_HandleTypeDef *debug_huart = NULL;

void Debug_Init(UART_HandleTypeDef *huart) {
    debug_huart = huart;
}

// 内部底层发送函数
static void Debug_Tx(const char* str) {
    if(debug_huart == NULL) return;
    HAL_UART_Transmit(debug_huart, (uint8_t*)str, strlen(str), 100);
}

void Debug_Log(const char* msg) {
    char buf[128];
    // 按照上位机协议格式打包: $LOG:xxxxx\r\n
    snprintf(buf, sizeof(buf), "$LOG:%s\r\n", msg);
    Debug_Tx(buf);
}

void Debug_SendVar(const char* channel_name, float value) {
    char buf[128];
    // 按照上位机协议格式打包: $频道名:数值\r\n
    snprintf(buf, sizeof(buf), "$%s:%.3f\r\n", channel_name, value);
    Debug_Tx(buf);
}

void Debug_SendStr(const char* channel_name, const char* str) {
    char buf[128];
    snprintf(buf, sizeof(buf), "$%s:%s\r\n", channel_name, str);
    Debug_Tx(buf);
}