//
// Created by 马皓然 on 2025/11/6.
//

#ifndef R1_CHASSIS_REMOTE_DRIVER_H
#define R1_CHASSIS_REMOTE_DRIVER_H
#include <stdint.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"

// --- 修改部分开始 ---
// 定义遥控器通道的范围参数
// 发送端发送范围 1000~2000，中值1500
#define RC_CHANNEL_OFFSET   1500
#define RC_CHANNEL_MAX      500.0f  // (2000-1500)
// --- 修改部分结束 ---

/*-- Remote control data structure --*/
typedef struct
{
    int16_t ch1; // X轴 (左右)
    int16_t ch2; // Y轴 (前后)
    int16_t cir; // 旋钮 (C)

    uint8_t sw1; // 对应 TX_buffer[13] (屏幕控制开启标志)
    uint8_t sw2; // 对应 TX_buffer[14]
    uint8_t sw3; // 对应 TX_buffer[15]
    uint8_t sw4; // 对应 TX_buffer[16]
    uint8_t sw5; // 对应 TX_buffer[17]

    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
    uint8_t button4;
    uint8_t button5;
    uint8_t button6;
} rc_info_t;

/**
 * @brief 底盘控制模式枚举
 */
typedef enum {
    CHASSIS_MODE_STANDBY = 0, 
    CHASSIS_MODE_AUTO    = 1, 
    CHASSIS_MODE_MANUAL  = 2,
    CHASSIS_MODE_SERVE   = 3,
    CHASSIS_MODE_SCREEN  = 4  // [新增] 屏幕控制模式
} chassis_mode_e;

// 遥控器数据工程量结构体
typedef struct {
    float vx;       // m/s
    float vy;       // m/s
    float vw;       // rad/s
    chassis_mode_e mode;   
    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
    uint8_t button4;
    uint8_t button5;
    uint8_t button6;
} remote_engineer_t;


extern rc_info_t rc;
extern remote_engineer_t remote_engineer;

extern osMutexId_t rc_mutexHandle;

void code_unzipread(uint8_t *code);
void Remote_Data_Convert(const rc_info_t *rc_data, remote_engineer_t *engineer_data);
BaseType_t Remote_GetEngineerData(remote_engineer_t *engineer_data);

#endif //R1_CHASSIS_REMOTE_DRIVER_H