#include "CRC.h"


// CRC-16多项式
uint16_t crc_16 = 0x1021;

//                              数据          字节数
uint16_t CRC_16 (const uint8_t *data ,uint8_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ crc_16;
            }
            else {
                crc = crc << 1;
            }
        }
    }
//    HAL_UART_Transmit(&huart3, (uint8_t *)crc,4,HAL_MAX_DELAY);
    return crc;
}

#define message_Length 3
uint8_t dma_buffer[32];

//uart_status RemoteReceive(uint8_t *data) {
//    uint8_t received_data[12];
//    uint16_t received_crc;
////    printf("okk\n\r");
//    memcpy(received_data, data, message_Length);
//    received_crc = (data[3] << 8) | data[4];
//    uint16_t calculated_crc = CRC_16(received_data, message_Length);
//    printf("calculated_crc %04x\n\r", calculated_crc);
//    fflush(stdout);  // 添加此行以确保前一个printf的数据被发送出去
//    printf("received_crc  %04x \n\r", received_crc);
//    fflush(stdout);  // 同样地，为第二个printf添加此行
//    if (calculated_crc != received_crc) {
//        return Fail;
//    } else {
//        memset(data, 0, sizeof(Remote_temp));
//        memcpy(data, received_data, sizeof(received_data));
//        return Success;
//    }
//}


//// 指令的最小长度
//#define COMMAND_MIN_LENGTH 4

//// 循环缓冲区大小
//#define BUFFER_SIZE 128
//// 循环缓冲区
//uint8_t buffer[BUFFER_SIZE];
//// 循环缓冲区读索引
//uint8_t readIndex = 0;
//// 循环缓冲区写索引
//uint8_t writeIndex = 0;

///**
//* @brief 增加读索引
//* @param length 要增加的长度
//*/
//void Command_AddReadIndex(uint8_t length) {
//    readIndex += length;
//    readIndex %= BUFFER_SIZE;
//}

///**
//* @brief 读取第i位数据 超过缓存区长度自动循环
//* @param i 要读取的数据索引
//*/

//uint8_t Command_Read(uint8_t i) {
//    uint8_t index = i % BUFFER_SIZE;
//    return buffer[index];
//}

///**
//* @brief 计算未处理的数据长度
//* @return 未处理的数据长度
//* @retval 0 缓冲区为空
//* @retval 1~BUFFER_SIZE-1 未处理的数据长度
//* @retval BUFFER_SIZE 缓冲区已满
//*/
//// uint8_t Command_GetLength() {
////   // 读索引等于写索引时，缓冲区为空
////   if (readIndex == writeIndex) {
////     return 0;
////   }
////   // 如果缓冲区已满,返回BUFFER_SIZE
////   if (writeIndex + 1 == readIndex || (writeIndex == BUFFER_SIZE - 1 && readIndex == 0)) {
////     return BUFFER_SIZE;
////   }
////   // 如果缓冲区未满,返回未处理的数据长度
////   if (readIndex <= writeIndex) {
////     return writeIndex - readIndex;
////   } else {
////     return BUFFER_SIZE - readIndex + writeIndex;
////   }
//// }

//uint8_t Command_GetLength() {
//    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
//}


///**
//* @brief 计算缓冲区剩余空间
//* @return 剩余空间
//* @retval 0 缓冲区已满
//* @retval 1~BUFFER_SIZE-1 剩余空间
//* @retval BUFFER_SIZE 缓冲区为空
//*/
//uint8_t Command_GetRemain() {
//    return BUFFER_SIZE - Command_GetLength();
//}

///**
//* @brief 向缓冲区写入数据
//* @param data 要写入的数据指针
//* @param length 要写入的数据长度
//* @return 写入的数据长度
//*/
//uint8_t Command_Write(uint8_t *data, uint8_t length) {
//    // 如果缓冲区不足 则不写入数据 返回0
//    if (Command_GetRemain() < length) {
//        return 0;
//    }
//    // 使用memcpy函数将数据写入缓冲区
//    if (writeIndex + length < BUFFER_SIZE) {
//        memcpy(buffer + writeIndex, data, length);
//        writeIndex += length;
//    } else {
//        uint8_t firstLength = BUFFER_SIZE - writeIndex;
//        memcpy(buffer + writeIndex, data, firstLength);
//        memcpy(buffer, data + firstLength, length - firstLength);
//        writeIndex = length - firstLength;
//    }
////    HAL_UART_Transmit(&huart3, (uint8_t *)"08",2,HAL_MAX_DELAY);
//    return length;
//}

///**
//* @brief 尝试获取一条指令
//* @param command 指令存放指针
//* @return 获取的指令长度
//* @retval 0 没有获取到指令
//*/
//uint8_t Command_GetCommand(uint8_t *command) {
//    // 寻找完整指令
//    while (1) {
//        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
//        if (Command_GetLength() < COMMAND_MIN_LENGTH) {
//        return 0;
//        }
//        // 如果不是包头 则跳过 重新开始寻找
//        if (Command_Read(readIndex) != 0x61) {
//        Command_AddReadIndex(1);
//        continue;
//        }
//        // 如果缓冲区长度小于指令长度 则不可能有完整的指令
//        uint8_t length = Command_GetLength() ;
//        // 如果找到完整指令 则将指令写入command 返回指令长度
//        for (uint8_t i = 0; i < length; i++) {
//        command[i] = Command_Read(readIndex + i);
//        }
//        Command_AddReadIndex(length);
//         // 如果CRC不正确 则跳过 重新开始寻找
//        if (RemoteReceive(command) == Fail) {
//        Command_AddReadIndex(1);
//        continue;
//        }

//        return length;
//    }
//}



