#include "chassis_task.h"
#include "control.h"
#include "fdcan_bsp.h"
#include <math.h>

float vx = 0, vy = 0, vr = 0;
extern FDCAN_HandleTypeDef hfdcan1;
static uint32_t last_cmd_time = 0;

static float Calc_Steering_Angle(float x, float y) {
    if (fabsf(x) < 0.001f && fabsf(y) < 0.001f) return 0.0f;
    float theta = atan2f(y, x); 
    float angle_deg = theta * 180.0f / 3.1415926f;
    angle_deg = 90.0f - angle_deg; 
    while (angle_deg > 180.0f)  angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

void Chassis_Stop(void) {
    vx = 0; vy = 0; vr = 0;
    Chassis_Update(0.5f, 0.0f, 0.0f);
}

// 解算底盘运动
void Chassis_Update(float target_vx, float target_vy, float target_vr)
{
    float R = CHASSIS_RADIUS; 
    float vr_R = target_vr * R;

    // Wheel 1 (正前)
    float vx_1 = target_vx - vr_R;
    float vy_1 = target_vy;
    // Wheel 2 (右后)
    float vx_2 = target_vx + 0.5f * vr_R;
    float vy_2 = target_vy + 0.866f * vr_R;
    // Wheel 3 (左后)
    float vx_3 = target_vx + 0.5f * vr_R;
    float vy_3 = target_vy - 0.866f * vr_R;

    float spd_1 = sqrtf(vx_1*vx_1 + vy_1*vy_1);
    float ang_1 = Calc_Steering_Angle(vx_1, vy_1);
    float spd_2 = sqrtf(vx_2*vx_2 + vy_2*vy_2);
    float ang_2 = Calc_Steering_Angle(vx_2, vy_2);
    float spd_3 = sqrtf(vx_3*vx_3 + vy_3*vy_3);
    float ang_3 = Calc_Steering_Angle(vx_3, vy_3);

    Steering_Wheel_Control(0, spd_1, ang_1);
    Steering_Wheel_Control(1, spd_2, ang_2);
    Steering_Wheel_Control(2, spd_3, ang_3);
}

// 【新增】周期调用的任务，处理断联保护
void Chassis_Task_Loop(void)
{
    // 如果 500ms(不要设太长，5秒太长了会撞墙) 没收到指令，执行停车保护
    if (HAL_GetTick() - last_cmd_time > 500) { 
        Chassis_Stop();
    } else {
        Chassis_Update(vx, vy, vr);
    }
}

// 回调函数：只负责更新全局变量和刷新时间戳
static void Chassis_Command_Callback(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
{
    uint32_t temp_u32;
    last_cmd_time = HAL_GetTick(); // 刷新心跳时间

    temp_u32 = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    vx = *((float*)&temp_u32); 
    temp_u32 = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | rx_data[7];
    vy = *((float*)&temp_u32);
    temp_u32 = (rx_data[8] << 24) | (rx_data[9] << 16) | (rx_data[10] << 8) | rx_data[11];
    vr = *((float*)&temp_u32);
}

void Chassis_Init(void) {
	FDCAN_Dispatch_t chassis;
	chassis.id_type = FDCAN_STANDARD_ID;
	chassis.id = 0x11;										
    chassis.instance_ptr = NULL;									
    chassis.handler = Chassis_Command_Callback;	
    fdcan_bsp_register(&chassis, &hfdcan3);
}