#include "chassis_task.h"
#include "fdcan_bsp.h" 
#include <math.h>
#include <stdio.h>
#include <string.h>
// 引用 CAN1 句柄
extern FDCAN_HandleTypeDef hfdcan1;

volatile uint32_t g_chassis_fdcan1_tx_ok_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_wait_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_drop_count = 0U;

typedef struct {
    uint8_t TxMessage[8];
} ChassisMessage_t;

// =============================================================
//  内部函数：计算并发送单个轮子的指令
// =============================================================

uint8_t FDCAN1_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
    FDCAN_TxHeaderTypeDef TxMessage;

    (void)len;

    if (TxData == NULL)
    {
        g_chassis_fdcan1_tx_drop_count++;
        return 1U;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        g_chassis_fdcan1_tx_wait_count++;
        g_chassis_fdcan1_tx_drop_count++;
        return 1U;
    }

    TxMessage.Identifier            = id;                   /* 设置发送帧ID */
    TxMessage.IdType                = EXTflag ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    TxMessage.TxFrameType           = FDCAN_DATA_FRAME;     /* 数据帧 */
    TxMessage.DataLength            = FDCAN_DLC_BYTES_8;    /* 经典CAN 8字节 */
    TxMessage.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch         = FDCAN_BRS_OFF;        /* 经典CAN关闭BRS */
    TxMessage.FDFormat              = FDCAN_CLASSIC_CAN;    /* 使用经典CAN格式 */
    TxMessage.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker         = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, TxData) == HAL_OK)
    {
        g_chassis_fdcan1_tx_ok_count++;
        return 0U;
    }

    g_chassis_fdcan1_tx_drop_count++;
    return 1U;
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

static uint32_t prv_float_to_u32(float value)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void prv_pack_u32_be(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)((value >> 24) & 0xFFU);
    dst[1] = (uint8_t)((value >> 16) & 0xFFU);
    dst[2] = (uint8_t)((value >> 8) & 0xFFU);
    dst[3] = (uint8_t)(value & 0xFFU);
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
    ChassisMessage_t Txdata[3];
    uint32_t spd1 = prv_float_to_u32(spd_1);
    uint32_t spd2 = prv_float_to_u32(spd_2);
    uint32_t spd3 = prv_float_to_u32(spd_3);
    uint32_t ang1 = prv_float_to_u32(ang_1);
    uint32_t ang2 = prv_float_to_u32(ang_2);
    uint32_t ang3 = prv_float_to_u32(ang_3);

    prv_pack_u32_be(&Txdata[0].TxMessage[0], spd1);
    prv_pack_u32_be(&Txdata[0].TxMessage[4], ang1);
    prv_pack_u32_be(&Txdata[1].TxMessage[0], spd2);
    prv_pack_u32_be(&Txdata[1].TxMessage[4], ang2);
    prv_pack_u32_be(&Txdata[2].TxMessage[0], spd3);
    prv_pack_u32_be(&Txdata[2].TxMessage[4], ang3);

    static uint8_t next_tx_index = 0U;
    const uint32_t ids[3] = {0x11U, 0x12U, 0x13U};

    for (uint8_t sent = 0U; sent < 3U; sent++)
    {
        uint8_t index = (uint8_t)((next_tx_index + sent) % 3U);

        if (FDCAN1_Transmit(Txdata[index].TxMessage, ids[index], 8U, 0U) != 0U)
        {
            break;
        }

        next_tx_index = (uint8_t)((index + 1U) % 3U);
    }
}

