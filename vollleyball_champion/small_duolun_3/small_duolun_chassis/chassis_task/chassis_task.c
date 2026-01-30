#include "chassis_task.h"
#include "control.h" // 引入控制头文件
#include "fdcan_bsp.h"
#include <math.h>
#include <stdio.h>

float vx = 0, vy = 0, vr = 0;

extern FDCAN_HandleTypeDef hfdcan1;

static uint32_t last_cmd_time = 0; // [新增] 记录上次收到指令的时间

// 辅助函数：反正切计算 (范围 -180 到 180，Y轴为0度)
static float Calc_Steering_Angle(float x, float y)
{
    // 如果速度极小，保持0 (或者保持上一时刻角度，但在update里没法存上一时刻，暂设0)
    if (fabsf(x) < 0.001f && fabsf(y) < 0.001f) return 0.0f;

    float theta = atan2f(y, x); 
    float angle_deg = theta * 180.0f / 3.1415926f;
    angle_deg = 90.0f - angle_deg; // 转换坐标系

    while (angle_deg > 180.0f)  angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;

    return angle_deg;
}


void Chassis_Stop(void)
{
    Chassis_Update(0.0f, 0.0f, 0.0f);
}

// vx, vy: m/s (前后，左右)
// vr: rad/s (自转)
void Chassis_Update(float vx, float vy, float vr)
{
    if (HAL_GetTick() - last_cmd_time > 5000) 
    { 
    // 超过 500ms 没收到指令
    vx = vy = vr = 0; // 强制停车
    }

    float R = CHASSIS_RADIUS; 
    float vr_R = vr * R;

    // --- 运动学解算 ---
    
    // Wheel 1 (正前)
    float vx_1 = vx - vr_R;
    float vy_1 = vy;

    // Wheel 2 (右后)
    float vx_2 = vx + 0.5f * vr_R;
    float vy_2 = vy + 0.866f * vr_R;

    // Wheel 3 (左后)
    float vx_3 = vx + 0.5f * vr_R;
    float vy_3 = vy - 0.866f * vr_R;

    // 计算速度和角度
    float spd_1 = sqrtf(vx_1*vx_1 + vy_1*vy_1);
    float ang_1 = Calc_Steering_Angle(vx_1, vy_1);

    float spd_2 = sqrtf(vx_2*vx_2 + vy_2*vy_2);
    float ang_2 = Calc_Steering_Angle(vx_2, vy_2);

    float spd_3 = sqrtf(vx_3*vx_3 + vy_3*vy_3);
    float ang_3 = Calc_Steering_Angle(vx_3, vy_3);

    // --- 下发控制指令 ---
    
    // 调用 control.c 中的封装函数
    // 索引 0对应前轮，1对应右后，2对应左后
    Steering_Wheel_Control(0, spd_1, ang_1);
    Steering_Wheel_Control(1, spd_2, ang_2);
    Steering_Wheel_Control(2, spd_3, ang_3);

    // 调试打印 (可选)
    // printf("1: %.2f/%.1f  2: %.2f/%.1f  3: %.2f/%.1f\r\n", spd_1, ang_1, spd_2, ang_2, spd_3, ang_3);
}

/**
 * @brief 底盘控制指令回调函数
 * @note  符合 FDCAN_Dispatch_t 中的函数指针定义
 */
static void Chassis_Command_Callback(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
{
    // 检查数据长度是否足够 (3个float = 12字节)
    // 注意：如果是 CAN FD，DLC 可能是 FDCAN_DLC_BYTES_12 或更大
    // 这里简单检查一下 header->DataLength 对应的字节数，或者直接解析
    
    uint32_t temp_u32;
    last_cmd_time = HAL_GetTick(); // 记录接收时间
    // 1. 解析 VX (假设大端模式传输: data[0]是高位)
    // 你的原始代码是 Big-Endian 组装，这里保持一致
    temp_u32 = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    vx = *((float*)&temp_u32); 

    // 2. 解析 VY
    temp_u32 = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | rx_data[7];
    vy = *((float*)&temp_u32);

    // 3. 解析 VR
    temp_u32 = (rx_data[8] << 24) | (rx_data[9] << 16) | (rx_data[10] << 8) | rx_data[11];
    vr = *((float*)&temp_u32);

}

void Chassis_Init(void)
{
    // === 注册底盘控制指令监听 ===
    // 这里的参数含义：
    // 1. &hfdcan1:        使用 CAN1 总线
    // 2. FDCAN_STANDARD_ID: 这是一个标准帧 ID (0x11 是 11位 ID)
    // 3. 0x11:            要监听的 CAN ID
    // 4. NULL:            不需要特定的实例指针 (因为 vx/vy/vr 是全局处理)
    // 5. Callback:        上面定义的函数
	FDCAN_Dispatch_t chassis;
	chassis.id_type=FDCAN_STANDARD_ID;
	chassis.id = 0x11;										
  chassis.instance_ptr = NULL;									
  chassis.handler = Chassis_Command_Callback;	// 回调函数句柄
  fdcan_bsp_register(&chassis,&hfdcan1);
    // 可以在这里做一些初始化
}

