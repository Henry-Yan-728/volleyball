/*
 * tjc_handle.c
 */

#include "tjc_handle.h"

 // 全局状态结构体实例
TJC_Status_TypeDef TJC_Status = { 0.0f, 0.0f, 0, 0, 0 };

// 内部变量
static uint8_t rx_byte;               // 临时存储接收到的1个字节
static uint8_t rx_buffer[TJC_RX_BUF_SIZE]; // 接收缓冲区
static uint8_t rx_index = 0;          // 缓冲区索引
static uint8_t msg_received_flag = 0; // 收到完整一包数据的标志

// 指向你在main中使用的UART句柄
static UART_HandleTypeDef* tjc_huart;

/**
 * @brief  初始化模块，开启接收中断
 * @param  huart: 串口句柄，如 &huart1
 */
void TJC_Init(UART_HandleTypeDef* huart) {
    tjc_huart = huart;
    // 开启接收中断，接收一个字节
    HAL_UART_Receive_IT(tjc_huart, &rx_byte, 1);
}

/**
 * @brief  串口接收回调处理，需放入 HAL_UART_RxCpltCallback 中
 */
void TJC_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == tjc_huart->Instance) {
        // 防止缓冲区溢出
        if (rx_index < TJC_RX_BUF_SIZE - 1) {
            // 将接收到的数据存入 buffer
            rx_buffer[rx_index++] = rx_byte;

            // 淘晶驰发送的指令以 \r\n (0x0D 0x0A) 结尾
            // 我们检测到 0x0A (换行符) 时认为接收结束
            if (rx_byte == 0x0A) {
                rx_buffer[rx_index] = '\0'; // 添加字符串结束符，方便处理
                msg_received_flag = 1;      // 标记收到完整消息
                // 注意：这里不重置 rx_index，留给主循环处理完后再重置
            }
        }
        else {
            // 缓冲区溢出处理，清空重新开始
            rx_index = 0;
            memset(rx_buffer, 0, TJC_RX_BUF_SIZE);
        }

        // 继续开启下一个字节的接收
        HAL_UART_Receive_IT(tjc_huart, &rx_byte, 1);
    }
}

/**
 * @brief  解析逻辑，建议放在 main 的 while(1) 中
 */
void TJC_Parse_Process(void) {
    if (msg_received_flag == 1) {
        // --- 开始解析 ---

        // 1. 检测 Start 指令
        if (strstr((char*)rx_buffer, "start") != NULL) {
            TJC_Status.cmd_start = 1;
            // 可以在这里清除 end 标志，或者做其他逻辑
            // printf("Command START received\n");
        }

        // 2. 检测 End 指令
        else if (strstr((char*)rx_buffer, "end") != NULL) {
            TJC_Status.cmd_end = 1;
            // printf("Command END received\n");
        }

        // 3. 检测坐标数据 (格式: X:12.34,Y:56.78)
        // 使用 sscanf 进行格式化提取
        else if (strstr((char*)rx_buffer, "X:") != NULL) {
            float temp_x, temp_y;
            // sscanf 返回成功匹配的参数个数
            int result = sscanf((char*)rx_buffer, "X:%f,Y:%f", &temp_x, &temp_y);

            if (result == 2) { // 成功提取到两个浮点数
                TJC_Status.target_x = temp_x;
                TJC_Status.target_y = temp_y;
                TJC_Status.coords_updated = 1; // 标记数据已更新
            }
        }

        // --- 解析结束，清理缓冲区 ---
        msg_received_flag = 0;
        rx_index = 0;
        memset(rx_buffer, 0, TJC_RX_BUF_SIZE);
    }
}