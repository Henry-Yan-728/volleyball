//
// Created by 马皓然 on 2025/11/6.
//
#include "remote_driver.h"
#include <string.h> // for memset if needed

// 速度参数设定
#define MAX_CHASSIS_SPEED   1000.0f // mm/s 或者 m/s，根据你的底盘单位决定
#define MAX_CHASSIS_W_RAD   5.0f    // rad/s

rc_info_t rc;
remote_engineer_t remote_engineer;

/**
 * @brief  对遥控器数据进行解算 (严格匹配发送端的 code_zip 结构)
 * @param  code 接收到的8字节数据帧 (code_zip)
 */
void code_unzipread(uint8_t *code){
    
    // 1. 解压 X轴 (CH1)
    // Sender: code_zip[1] = x >> 3; code_zip[2] = (x & 7) << 5 ...
    uint16_t x_raw = (code[1] << 3) | (code[2] >> 5);
    rc.ch1 = (int16_t)x_raw - RC_CHANNEL_OFFSET; // 减去1500，得到 +/- 500

    // 2. 解压 Y轴 (CH2)
    // Sender: code_zip[2] low 5 bits | code_zip[3] high 6 bits
    uint16_t y_raw = ((code[2] & 0x1F) << 6) | (code[3] >> 2);
    rc.ch2 = (int16_t)y_raw - RC_CHANNEL_OFFSET;

    // 3. 解压 旋钮 (CIR/C)
    // Sender: code_zip[3] low 2 bits (high 2 of C) | code_zip[4] (mid 8 of C) | code_zip[5] high 1 bit (low 1 of C)
    uint16_t c_raw = ((code[3] & 0x03) << 9) | (code[4] << 1) | (code[5] >> 7);
    rc.cir = (int16_t)c_raw - RC_CHANNEL_OFFSET;

    // 4. 解压 开关量 (Switches)
    // 根据 main.c 的打包逻辑：
    // code_zip[5] 的结构: [Bit7: C_lsb] [Bit6-5: TX13(SW1)] [Bit4-3: TX14(SW2)] [Bit2: Unused/0] [Bit1: TX15(SW3)] [Bit0: TX16_Hi]
    
    rc.sw1 = (code[5] >> 5) & 0x03; // TX_buffer[13] -> 屏幕控制标志
    rc.sw2 = (code[5] >> 3) & 0x03; // TX_buffer[14]
    
    // TX_buffer[15] 在发送端放在了 Bit 1
    rc.sw3 = (code[5] >> 1) & 0x01; 

    // TX_buffer[16] (SW4) 被拆分了：高位在 code[5] Bit 0, 低位在 code[6] Bit 7
    uint8_t sw4_high = (code[5] & 0x01);
    uint8_t sw4_low  = (code[6] >> 7) & 0x01;
    rc.sw4 = (sw4_high << 1) | sw4_low;

    // TX_buffer[17] (SW5) 在 code[6] Bit 6-5
    rc.sw5 = (code[6] >> 5) & 0x03;

    // 5. 解压 按钮 (Buttons)
    rc.button1 = (code[6] >> 4) & 0x01; // TX18
    rc.button2 = (code[6] >> 3) & 0x01; // TX19
    rc.button3 = (code[6] >> 2) & 0x01; // TX20
    rc.button4 = (code[6] >> 1) & 0x01; // TX21
    rc.button5 = (code[6])      & 0x01; // TX22
    rc.button6 = (code[7] >> 7) & 0x01; // TX23
}

/**
 * @brief  工程量转换
 */
void Remote_Data_Convert(const rc_info_t *rc_data, remote_engineer_t *engineer_data) {
    // 归一化：将通道值映射到 [-1.0, 1.0]
    // 这里的 RC_CHANNEL_MAX 已经是 500.0f
    float ch1_norm = (float)rc_data->ch1 / RC_CHANNEL_MAX; 
    float ch2_norm = (float)rc_data->ch2 / RC_CHANNEL_MAX; 
    float cir_norm = (float)rc_data->cir / RC_CHANNEL_MAX; 

    // 限制范围，防止浮点数计算微小误差导致溢出
    if(ch1_norm > 1.0f) ch1_norm = 1.0f; else if(ch1_norm < -1.0f) ch1_norm = -1.0f;
    if(ch2_norm > 1.0f) ch2_norm = 1.0f; else if(ch2_norm < -1.0f) ch2_norm = -1.0f;

    // 转换为实际工程量速度
    engineer_data->vx = (int16_t)(ch1_norm * MAX_CHASSIS_SPEED);
    engineer_data->vy = (int16_t)(ch2_norm * MAX_CHASSIS_SPEED);
    engineer_data->vw = (int16_t)(cir_norm * MAX_CHASSIS_W_RAD);

    // --- 模式判断逻辑 ---
    
    // 模式和比例因子判断（由开关控制）
    if (rc_data->sw1 == 1 && rc_data->sw2 == 1)  {
        engineer_data->mode = CHASSIS_MODE_MANUAL;// 手动模式
    } else if (rc_data->sw1 == 2 && rc_data->sw2 == 2) {
        engineer_data->mode = CHASSIS_MODE_AUTO; // 自动模式
    } else if (rc_data->sw1 == 1 && rc_data->sw2 == 2) { 
			engineer_data->mode = CHASSIS_MODE_SERVE;
	}else {
        engineer_data->mode = CHASSIS_MODE_STANDBY; // 待  机模式
    }

    // 按钮赋值
    engineer_data->button1 = rc_data->button1;
    engineer_data->button2 = rc_data->button2;
    engineer_data->button3 = rc_data->button3;
    engineer_data->button4 = rc_data->button4;
    engineer_data->button5 = rc_data->button5;
    engineer_data->button6 = rc_data->button6;
}

/**
 * @brief  获取受保护的遥控器工程量数据
 */
BaseType_t Remote_GetEngineerData(remote_engineer_t *engineer_data) {
    if (osMutexAcquire(rc_mutexHandle, osWaitForever) == osOK) {
        *engineer_data = remote_engineer;
        osMutexRelease(rc_mutexHandle);
        return pdPASS;
    }
    return pdFAIL;
}