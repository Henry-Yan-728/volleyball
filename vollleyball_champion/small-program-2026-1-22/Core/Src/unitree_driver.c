#include "unitree_driver.h"
#include "fdcan.h"
#include <math.h> // 需要 fabs 函数
#include <stdio.h>

#define M_PI 3.14159f

// 引用外部定义的CAN句柄
extern FDCAN_HandleTypeDef hfdcan3;

// 定义全局变量 (保持您原有的结构)
int8_t motor_temperature[3];
float actual_torque[3], actual_speed[3], actual_position[3];

// ================== 内部变量：用于轨迹规划 ==================
// 假设最多支持4个电机 (ID 0-3)
static float g_ramp_current_pos[4] = {0.0f}; 
static uint32_t g_last_ramp_tick[4] = {0};
static uint8_t g_is_initialized[4] = {0}; // 标记是否初始化过

// ================== 原有底层驱动函数 ==================

HAL_StatusTypeDef sendCANControlMotor(uint32_t moduleId, uint32_t motorId, float torque, float speed, float position, int8_t *motor_temperature, float *actual_torque, float *actual_speed, float *actual_position)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];

  // 配置CAN发送帧的扩展ID
  TxHeader.Identifier = ((moduleId & 0x3) << 27) |
                        (0 << 26)|  // 下发标志
                        (0 << 24) |  // 数据内容：电机控制
                        (10 << 16) | // 控制模式10
                        ((motorId & 0xF) << 8) |
                        (1 << 12) | // motorStatus默认为1
                        (0 << 15) | // 超时标志
                        0;          // 预留

  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  int32_t Pset = (int32_t)(position * 32768 / (2 * M_PI));
  int16_t Wset = (int16_t)(speed * 256.0f / (2 * M_PI));
  int16_t Tset = (int16_t)(torque * 256.0f);
  
  TxData[0] = Pset & 0xFF;
  TxData[1] = (Pset >> 8) & 0xFF;
  TxData[2] = (Pset >> 16) & 0xFF;
  TxData[3] = (Pset >> 24) & 0xFF;
  TxData[4] = Wset & 0xFF;
  TxData[5] = (Wset >> 8) & 0xFF;
  TxData[6] = Tset & 0xFF;
  TxData[7] = (Tset >> 8) & 0xFF;

  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);

  if (status != HAL_OK) return status;

  // 注意：在RTOS中，不建议使用阻塞死循环等待接收。
  // 建议将接收放在中断回调中处理。为了保持原逻辑，此处暂时保留，但需谨慎。
  uint32_t timeout = HAL_GetTick() + 10; // 缩短超时时间防止卡死任务
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0) == 0)
  {
    if (HAL_GetTick() > timeout) return HAL_TIMEOUT;
  }

  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  status = HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader, RxData);

  if (status == HAL_OK)
  {
    uint8_t received_motor_id = RxHeader.Identifier >> 8 & 0x0F;
    if (received_motor_id != motorId) return HAL_ERROR;

    *motor_temperature = RxHeader.Identifier & 0xFF;
    uint16_t temp_torque = (RxData[6] | (RxData[7] << 8));
    uint16_t temp_speed = (RxData[4] | (RxData[5] << 8));
    uint32_t temp_position = (RxData[0] | (RxData[1] << 8) | (RxData[2] << 16) | (RxData[3] << 24));

    *actual_torque = (float)temp_torque / 256.0f;
    *actual_speed = (float)temp_speed / 256.0f * (2 * M_PI);
    *actual_position = (float)temp_position / 32768.0f * (2 * M_PI);
  }
  return status;
}

HAL_StatusTypeDef sendCANSetMotorKK(uint32_t moduleId, uint32_t motorId, float Kpos, float Kspd)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = {0};

  TxHeader.Identifier = ((moduleId & 0x3) << 27) |
                   (0 << 26) | 
                   (0 << 24) | 
                   (11 << 16) | // 控制模式11
                   ((motorId & 0xF) << 8) |
                   0;

  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  uint16_t KspdSet = (uint16_t)(Kspd * 1280.0f);
  uint16_t KposSet = (uint16_t)(Kpos * 1280.0f);
  
  TxData[0] = KspdSet & 0xFF;
  TxData[1] = (KspdSet >> 8) & 0xFF;
  TxData[2] = KposSet & 0xFF;
  TxData[3] = (KposSet >> 8) & 0xFF;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

void Unitree_Send_Cmd(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd)
{
  uint32_t moduleId = 3; 
  int8_t motor_temperature0;
  float actual_torque0, actual_speed0, actual_position0;

  // [修改] 优化带宽：仅当Kp/Kd改变时才发送设置指令
  // 静态变量保存上一次的设置值，初始化为负数以确保第一次一定会发送
  static float last_Kpos[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
  static float last_Kspd[4] = {-1.0f, -1.0f, -1.0f, -1.0f};

  if (motorId < 4) {
      if (last_Kpos[motorId] != Kpos || last_Kspd[motorId] != Kspd) {
          // 只有参数不同时才调用设置函数
          sendCANSetMotorKK(moduleId, motorId, Kpos, Kspd); 
          
          // 更新缓存
          last_Kpos[motorId] = Kpos;
          last_Kspd[motorId] = Kspd;
          
          // 如果总线负载极高，这里可考虑微小延时，但在200Hz控制下通常不需要
      }
  }

  HAL_StatusTypeDef status = sendCANControlMotor(moduleId, motorId, torque, 
                                            speed, position, &motor_temperature0, &actual_torque0, &actual_speed0, &actual_position0);
    
  if (motorId < 3) {
      motor_temperature[motorId] = motor_temperature0;
      actual_torque[motorId] = actual_torque0;
      actual_speed[motorId] = actual_speed0;
      actual_position[motorId] = actual_position0;
  }
}
void get_Unitree_pos(uint32_t motorId)
{
	 float torque =0.0f;
	float speed =0.0f;
	float position = 0.0f;
	float Kpos = 0.0f;
	float Kspd =0.0f;
  uint32_t moduleId = 3; 
  int8_t motor_temperature0;
  float actual_torque0, actual_speed0, actual_position0;

  // [修改] 优化带宽：仅当Kp/Kd改变时才发送设置指令
  // 静态变量保存上一次的设置值，初始化为负数以确保第一次一定会发送
  static float last_Kpos[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
  static float last_Kspd[4] = {-1.0f, -1.0f, -1.0f, -1.0f};

  if (motorId < 4) {
      if (last_Kpos[motorId] != Kpos || last_Kspd[motorId] != Kspd) {
          // 只有参数不同时才调用设置函数
          sendCANSetMotorKK(moduleId, motorId, Kpos, Kspd); 
          
          // 更新缓存
          last_Kpos[motorId] = Kpos;
          last_Kspd[motorId] = Kspd;
          
          // 如果总线负载极高，这里可考虑微小延时，但在200Hz控制下通常不需要
      }
  }

  HAL_StatusTypeDef status = sendCANControlMotor(moduleId, motorId, torque, 
                                            speed, position, &motor_temperature0, &actual_torque0, &actual_speed0, &actual_position0);
    
  if (motorId < 3) {
      motor_temperature[motorId] = motor_temperature0;
      actual_torque[motorId] = actual_torque0;
      actual_speed[motorId] = actual_speed0;
      actual_position[motorId] = actual_position0;
  }
	printf("%lf",actual_position[motorId]);
}
