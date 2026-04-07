#include "chassis_task.h"
#include "fdcan_bsp.h" 
#include <math.h>
#include <stdio.h>
// 引用 CAN1 句柄
extern FDCAN_HandleTypeDef hfdcan1;

// 辅助联合体：用于 float 转 字节
typedef union {
    float f;
    uint8_t b[4];
} FloatByte_t;

// =============================================================
//  内部函数：计算并发送单个轮子的指令
// =============================================================

uint8_t FDCAN1_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧ID */
	TxMessage.IdType				= EXTflag ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= len;					/* CAN FD数据长度（DLC宏） */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;	
	TxMessage.BitRateSwitch 		= FDCAN_BRS_ON;			/* CAN FD必须开启BRS */
	TxMessage.FDFormat 				= FDCAN_FD_CAN;			/* 启用CAN FD格式（支持>8字节） */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	
	TxMessage.MessageMarker 		= 0;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}
// =============================================================
//  内部函数：反正切计算 (范围 -180 到 180，Y轴为0度)
// =============================================================
static float Calc_Steering_Angle(float x, float y)
{
    // 如果速度极小，保持原角度或设为0
    if (fabsf(x) < 0.001f && fabsf(y) < 0.001f) return 0.0f;

    float theta = atan2f(y, x); // 标准 atan2 返回 (-PI, PI]
    
    // 转换为角度制
    float angle_deg = theta * 180.0f / 3.1415926f;
    
    // 转换坐标系：将 atan2 的 0度(X轴) 转换为 0度(Y轴)
    angle_deg = 90.0f - angle_deg; 

    // 归一化到 -180 ~ 180
    while (angle_deg > 180.0f)  angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;

    return angle_deg;
}

// =============================================================
//  公开函数实现
// =============================================================

void Chassis_Init(void)
{
    // 初始化代码，如需复位舵轮可在此添加
}

void Chassis_Stop(void)
{
    // 利用 Update 函数内部的机制：
    // 当输入 vx, vy 为 0 时，代码中的 Calc_Steering_Angle 会检测到小数值并返回 0.0f
    // 效果等同于上面的 memset 方案，但代码复用度更高
    Chassis_Update(0.0f, 0.0f, 0.0f);
}
// 核心解算函数 (三轮全向)
// vx: 前后速度 (+向前)
// vy: 左右速度 (+向左，注意根据实际遥控器定义，通常右手定则是左正)
// vr: 自转速度 (+逆时针)
void Chassis_Update(float vx, float vy, float vr)
{
    // 1. 几何参数定义
    // R 为底盘中心到轮子中心的距离 (单位：米，或者是归一化比例)
    float R = CHASSIS_RADIUS; 

    // 2. 运动学模型 (三轮等边三角形分布)
    // 假设：
    // Wheel 1: 正前方 (Head, 0度方向, 坐标 (0, R))
    // Wheel 2: 右后方 (-120度方向, 坐标 (R*sin(120), R*cos(120)) -> (0.866R, -0.5R))
    // Wheel 3: 左后方 (240度方向,  坐标 (-0.866R, -0.5R))
    
    // 注意：旋转产生的线速度方向与半径垂直
    // v_rot_x = -vr * y
    // v_rot_y =  vr * x

    // 预计算旋转分量
    float vr_R = vr * R;
    
    // --- 轮子 1 (正前) ---
    // 位置 (0, R) -> 旋转产生的速度向量 (-vr*R, 0)
    float vx_1 = vx - vr_R;
    float vy_1 = vy;

    // --- 轮子 2 (右后) ---
    // 位置 (0.866R, -0.5R) -> 旋转产生的速度向量 (-vr*(-0.5R), vr*(0.866R))
    // 即 (0.5 * vr * R, 0.866 * vr * R)
    float vx_2 = vx + 0.5f * vr_R;
    float vy_2 = vy + 0.866f * vr_R;

    // --- 轮子 3 (左后) ---
    // 位置 (-0.866R, -0.5R) -> 旋转产生的速度向量 (-vr*(-0.5R), vr*(-0.866R))
    // 即 (0.5 * vr * R, -0.866 * vr * R)
    float vx_3 = vx + 0.5f * vr_R;
    float vy_3 = vy - 0.866f * vr_R;

    // 3. 计算这一刻 3 个轮子需要的速度和角度
    float spd_1 = sqrtf(vx_1*vx_1 + vy_1*vy_1);
    float ang_1 = Calc_Steering_Angle(vx_1, vy_1);

    float spd_2 = sqrtf(vx_2*vx_2 + vy_2*vy_2);
    float ang_2 = Calc_Steering_Angle(vx_2, vy_2);

    float spd_3 = sqrtf(vx_3*vx_3 + vy_3*vy_3);
    float ang_3 = Calc_Steering_Angle(vx_3, vy_3);
		//printf("%f %f %f\r\n",ang_1,ang_2,ang_3);
    // 4. 发送指令
    // 请在 chassis_task.h 中将原来的 LF/LB... 改为 ID_WHEEL_1/2/3
typedef struct {
		uint8_t TxMessage[8];
} message_t;
     message_t Txdata[3];
FloatByte_t float_buf; 
		uint32_t spd1= *(uint32_t*)(&spd_1);
		uint32_t spd2= *(uint32_t*)(&spd_2);
		uint32_t spd3= *(uint32_t*)(&spd_3);
		uint32_t ang1= *(uint32_t*)(&ang_1);
		uint32_t ang2= *(uint32_t*)(&ang_2);
		uint32_t ang3= *(uint32_t*)(&ang_3);
		Txdata[0].TxMessage[0]  = (spd1 >> 24) & 0xFF;  // 提取24~31位
		Txdata[0].TxMessage[1]  = (spd1 >> 16) & 0xFF;  // 提取16~23位
		Txdata[0].TxMessage[2] = (spd1 >> 8)  & 0xFF;  // 提取8~15位
		Txdata[0].TxMessage[3] = spd1 & 0xFF;
		Txdata[0].TxMessage[4]  = (ang1 >> 24) & 0xFF;  // 提取24~31位
		Txdata[0].TxMessage[5]  = (ang1 >> 16) & 0xFF;  // 提取16~23位
		Txdata[0].TxMessage[6] = (ang1 >> 8)  & 0xFF;  // 提取8~15位
		Txdata[0].TxMessage[7] = ang1 & 0xFF;
		Txdata[1].TxMessage[0]  = (spd2 >> 24) & 0xFF;  // 提取24~31位
		Txdata[1].TxMessage[1]  = (spd2 >> 16) & 0xFF;  // 提取16~23位
		Txdata[1].TxMessage[2] = (spd2 >> 8)  & 0xFF;  // 提取8~15位
		Txdata[1].TxMessage[3] = spd2 & 0xFF;
		Txdata[1].TxMessage[4]  = (ang2 >> 24) & 0xFF;  // 提取24~31位
		Txdata[1].TxMessage[5]  = (ang2 >> 16) & 0xFF;  // 提取16~23位
		Txdata[1].TxMessage[6] = (ang2 >> 8)  & 0xFF;  // 提取8~15位
		Txdata[1].TxMessage[7] = ang2 & 0xFF;    
		Txdata[2].TxMessage[0]  = (spd3 >> 24) & 0xFF;  // 提取24~31位
		Txdata[2].TxMessage[1]  = (spd3 >> 16) & 0xFF;  // 提取16~23位
		Txdata[2].TxMessage[2] = (spd3 >> 8)  & 0xFF;  // 提取8~15位
		Txdata[2].TxMessage[3] = spd3 & 0xFF;
		Txdata[2].TxMessage[4]  = (ang3 >> 24) & 0xFF;  // 提取24~31位
		Txdata[2].TxMessage[5]  = (ang3 >> 16) & 0xFF;  // 提取16~23位
		Txdata[2].TxMessage[6] = (ang3 >> 8)  & 0xFF;  // 提取8~15位
		Txdata[2].TxMessage[7] = ang3 & 0xFF;    
		for(int i=1;i <= 3;i++)
		{
			uint32_t id = 0x10+i;
		FDCAN1_Transmit(Txdata[i-1].TxMessage, id, 8, 1);
		}
}