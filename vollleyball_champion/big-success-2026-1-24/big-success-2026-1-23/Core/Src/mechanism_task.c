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

// 全局轨迹对象
static Cushion_Profile_t cushion = {0};

// 度转弧度宏
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

void Mechanism_Init(void)
{
    // 1. 初始化大疆电机 (会调用内部的 dji_motors_init)
    // 确保你已经把 dji_motor.c 里的初始化代码改好了
    dji_motors_init();

    // 初始化时设为静止在起始位
    cushion.is_moving = false;
    cushion.out_pos_rad = DEG2RAD(CUSHION_ANGLE_START);
    cushion.out_vel_rad = 0.0f;
    
    // 发送一次初始位置锁定
    Mechanism_Cushion_Return();
}

/**
 * @brief 触发击球：计算非对称三角形规划
 * @param peak_speed_deg_s 期望在97度时的最大速度 (度/秒)
 */
void Mechanism_Cushion_Trigger(float peak_speed_deg_s)
{
    float peak_vel_rad = DEG2RAD(peak_speed_deg_s);
    
    // 1. 设置关键点 (转为弧度)
    cushion.start_pos = DEG2RAD(CUSHION_ANGLE_START);
    cushion.hit_pos   = DEG2RAD(CUSHION_ANGLE_HIT);
    cushion.end_pos   = DEG2RAD(CUSHION_ANGLE_END);
    cushion.max_vel   = peak_vel_rad;

    // 2. 计算行程距离
    float dist_acc = cushion.hit_pos - cushion.start_pos; // 57 -> 97 (40度)
    float dist_dec = cushion.end_pos - cushion.hit_pos;   // 97 -> 114 (17度)
    
    // 3. 根据 v^2 = 2ax 计算加速度和减速度
    // a = v^2 / 2x
    // 注意：为了保证在97度刚好达到最大速度，必须严格遵守此公式
    if (dist_acc > 0 && dist_dec > 0) {
        cushion.accel = (peak_vel_rad * peak_vel_rad) / (2.0f * dist_acc);
        cushion.decel = (peak_vel_rad * peak_vel_rad) / (2.0f * dist_dec);
        
        // 计算两段时间 t = v / a
        cushion.total_time_acc = peak_vel_rad / cushion.accel;
        cushion.total_time_dec = peak_vel_rad / cushion.decel;
        
        cushion.current_time = 0.0f;
        cushion.is_returning = false;
        cushion.is_moving = true;
    }
}

/**
 * @brief 复位：使用普通平滑控制回到起点
 */
void Mechanism_Cushion_Return(void)
{
    cushion.is_moving = false; // 停止轨迹计算
    cushion.is_returning = true;
    
    float target_rad = DEG2RAD(CUSHION_ANGLE_START);
    
    // 使用带阻尼的位置控制慢慢回去
    // 宇树电机参数: Torque=0, Speed=0, Pos=Target, Kp=0.5, Kd=1.0
    float kp = 0.4f; 
    float kd = 0.1f;
    
    // 转换为转子端弧度发送
    float rotor_pos = target_rad * REDUCTION_RATIO;
    
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_1, 0.0f, 0.0f, rotor_pos, kp, kd);
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_2, 0.0f, 0.0f, rotor_pos, kp, kd);
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_3, 0.0f, 0.0f, rotor_pos, kp, kd);
}

/**
 * @brief 高频更新函数 (1ms 调用一次)
 * 实现非对称轨迹生成 + 前馈控制
 */
void Mechanism_Cushion_Update_Loop(uint32_t dt_ms)
{
    if (!cushion.is_moving) return;

    float dt = (float)dt_ms / 1000.0f;
    cushion.current_time += dt;
    
    float t = cushion.current_time;
    float pos = 0.0f;
    float vel = 0.0f;
    float torque_ff = 0.0f;
    
    // --- 第一阶段：加速 (0 ~ 97度) ---
    if (t <= cushion.total_time_acc)
    {
        // v = a * t
        vel = cushion.accel * t;
        // x = x0 + 0.5 * a * t^2
        pos = cushion.start_pos + 0.5f * cushion.accel * t * t;
        
        // 前馈力矩：提供加速所需的转矩 (F = ma => T = I * alpha)
        // 这里给一个正向的辅助力矩，数值需根据负载惯量调整，这里给个经验值
        torque_ff = 1.0f; 
    }
    // --- 第二阶段：减速 (97 ~ 114度) ---
    else if (t <= (cushion.total_time_acc + cushion.total_time_dec))
    {
        float t_rel = t - cushion.total_time_acc;
        
        // v = v_max - a_dec * t
        vel = cushion.max_vel - cushion.decel * t_rel;
        if (vel < 0) vel = 0;
        
        // x = x_hit + v_max*t - 0.5*a*t^2
        pos = cushion.hit_pos + (cushion.max_vel * t_rel) - (0.5f * cushion.decel * t_rel * t_rel);
        
        // 减速阶段力矩反向，帮助刹车
        torque_ff = -1.5f; 
    }
    // --- 第三阶段：结束 ---
    else
    {
        pos = cushion.end_pos;
        vel = 0.0f;
        torque_ff = 0.0f;
        cushion.is_moving = false; // 动作完成
    }

    // --- 硬件指令发送 ---
    // 刚度设置：在快速运动时，前馈(FF)主导，Kp适当减小防止震荡
    // 到达终点后，Kp增大锁住位置
    float kp, kd;
    
    if (cushion.is_moving) {
        kp = 2.0f;   // 运动中刚度
        kd = 0.05f;  // 运动中阻尼
    } else {
        kp = 2.5f;   // 停止时高刚度锁死
        kd = 0.5f;   // 停止时高阻尼消震
    }

    // 坐标系转换：输出端 -> 转子端
    float rotor_pos = pos * REDUCTION_RATIO;
    float rotor_vel = vel * REDUCTION_RATIO;
    
    // 发送指令 (三电机同步)
    // 注意：Unitree_Send_Cmd 内部应该是非阻塞的
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_1, torque_ff, rotor_vel, rotor_pos, kp, kd);
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_2, torque_ff, rotor_vel, rotor_pos, kp, kd);
    Unitree_Send_Cmd(UNITREE_ID_CUSHION_3, torque_ff, rotor_vel, rotor_pos, kp, kd);
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
    float kp = 1.8f;
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
