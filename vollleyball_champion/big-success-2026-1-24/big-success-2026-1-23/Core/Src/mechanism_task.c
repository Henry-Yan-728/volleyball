#include "mechanism_task.h"
#include "unitree_driver.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include <math.h>
// =============================================================
//  初始化
// =============================================================
		// 初始化：起始0度，目标0度，速度 0.05度/ms (即50度/秒，较慢平滑)
		Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.1f, 0}; 
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
	v_axis.target_angle = angle_deg;
}

/* ----------------- 2. 轨迹更新函数 ----------------- */
// 该函数需要在主循环或定时器中频繁调用 (建议 1ms 一次)
void Update_Virtual_Axis(void)
{
    uint32_t now = HAL_GetTick();
    
    // 简单的非阻塞延时，确保控制周期稳定
    if (now - v_axis.last_tick < 1) return; 
    v_axis.last_tick = now;

    // --- 轨迹插补算法 (Ramp) ---
    float error = v_axis.target_angle - v_axis.current_angle;
    
    // 如果误差很小，直接到达目标
    if (fabs(error) <= v_axis.velocity_deg_ms) {
        v_axis.current_angle = v_axis.target_angle;
    }
    else {
        // 根据方向增加或减少当前角度
        if (error > 0) v_axis.current_angle += v_axis.velocity_deg_ms;
        else           v_axis.current_angle -= v_axis.velocity_deg_ms;
    }

    // --- 同步分发给物理电机 ---
    DJI_Motor_Instance* m_right = dji_motor_get_instance(0); // 假设 ID 0x201
    DJI_Motor_Instance* m_left  = dji_motor_get_instance(1); // 假设 ID 0x202

    if (m_right && m_left)
    {
        // 转换角度为编码器数值 (8192 line)
        int32_t encoder_pos = dji_degree2encoder(v_axis.current_angle);

        // 【关键逻辑】实现同轴异向
        // 左电机：正向旋转 encoder_pos
        dji_motor_set_location(m_left, encoder_pos);
        
        // 右电机：反向旋转 -encoder_pos
        dji_motor_set_location(m_right, -encoder_pos);
    }
}
