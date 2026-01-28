#include "mechanism_task.h"
#include "unitree_driver.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#define M_PI 3.1415926535f
// =============================================================
//  初始化
// =============================================================
void Mechanism_Init(void)
{
    // 1. 初始化大疆电机 (会调用内部的 dji_motors_init)
    // 确保你已经把 dji_motor.c 里的初始化代码改好了
	
    dji_motors_init();
}

// =============================================================
//  垫球机构控制 (Position Control)
// =============================================================
void Mechanism_Cushion_SetAngle(float angle_rad, float kp)
{
    // 参数: id, torque, speed, pos, kp, kd
    // 纯位置控制：Torque=0, Speed=0, KP=设定值, KD=小阻尼
    float kd_default = 0.1f; 
    angle_rad = angle_rad  * M_PI * 6.33f/ 180.0f; // 角度转弧度
    // 向 3 个电机发送相同的指令
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_1, 0.0f, 0.0f, angle_rad, kp, kd_default);
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_2, 0.0f, 0.0f, angle_rad, kp, kd_default);
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_3, 0.0f, 0.0f, angle_rad, kp, kd_default);
	
}

void Mechanism_Zero()
{
	get_Unitree_pos(UNITREE_ID_CUSHION_1); 
	get_Unitree_pos(UNITREE_ID_CUSHION_2);
	get_Unitree_pos(UNITREE_ID_CUSHION_3);
	get_Unitree_pos(UNITREE_ID_SERVE);
}
// =============================================================
//  发球机构控制 (Velocity Control)
// =============================================================
void Mechanism_Serve_SetAngle(float angle_rad)
{
    // 纯速度控制：Torque=0, Speed=设定值, Pos=0(不重要), KP=0, KD=设定值(作为速度增益)
    // 注意：宇树电机混合模式下，速度控制通常把 KP 设为 0，KD 设为稍微大一点的值来跟踪速度
    float kp = 1.0f;
    float kd = 0.1f; 
	 angle_rad = angle_rad  * M_PI * 6.33f/ 180.0f; // 角度转弧度
    Unitree_Send_Cmd(UNITREE_ID_SERVE, 0.0f, 0.0f, angle_rad, kp, kd);
}

// =============================================================
//  俯仰机构控制 (DJI Position Control)
// =============================================================
void Mechanism_Pitch_SetAngle(float angle_deg)
{
    // 1. 获取电机实例
    DJI_Motor_Instance* pitch_motor = dji_motor_get_instance(DJI_INDEX_PITCH);
    
    if (pitch_motor != NULL)
    {
        // 2. 角度转编码器值
        // 假设初始位置为0，并且电机与机构是 1:1 连接 (如果带减速箱需乘以减速比)
        // dji_degree2encoder 已经在 dji_motor.c 中实现
        int32_t target_encoder = dji_degree2encoder(angle_deg);
        
        // 3. 发送位置指令
        dji_motor_set_location(pitch_motor, target_encoder);
    }
}