#include "Pan_Tilt_control.h"
#include "ramp.h"      // 引入规划器
#include "dji_motor.h"

// 定义两个轴的规划器
static Ramp_Handle_t yaw_ramp;
static Ramp_Handle_t pitch_ramp;

// 标记是否初始化过
static uint8_t is_control_init = 0;

// 控制循环周期 (假设你的控制任务是 1ms 运行一次)
#define CONTROL_DT 0.001f 

// 初始化函数 (需要在 main 中调用一次)
void gimbal_control_init(void)
{
    // Yaw轴 (GM6020): 直驱，编码器单位 8192/圈
    // 最大速度: 2圈/秒 (16000 count/s)
    // 加速度: 1秒达到最大速度 (16000 count/s^2)
    ramp_init(&yaw_ramp, 16000.0f, 15000.0f, 15000.0f); 

    // Pitch轴 (M2006): 减速比 36:1, 编码器单位非常大
    // 360度 = 8192 * 36 ≈ 294912 count
    // 最大速度: 30度/秒 ≈ 24576 count/s
    ramp_init(&pitch_ramp, 100000.0f, 80000.0f, 80000.0f);
    
    // 同步初始位置 (防止上电猛甩)
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);
    if(yaw_motor && yaw_motor->is_online) {
        yaw_ramp.current_pos = (float)yaw_motor->measure.total_angle;
        yaw_ramp.target_pos = yaw_ramp.current_pos;
    }
    
    is_control_init = 1;
}

/**
 * @brief 用户调用此函数设置目标角度 (输入)
 */
void gimbal_set_angle(float yaw_deg, float pitch_deg)
{
    // 1. 角度转编码器值 (目标值)
    int32_t yaw_target_cnt = dji_degree2encoder(yaw_deg); 
    // M2006 减速比 36
    int32_t pitch_target_cnt = dji_degree2encoder(pitch_deg) * 36.0f;

    // 2. 将目标值喂给规划器，而不是直接给电机
    ramp_set_target(&yaw_ramp, (float)yaw_target_cnt);
    ramp_set_target(&pitch_ramp, (float)pitch_target_cnt);
}

/**
 * @brief 核心控制任务 loop
 * @note  必须在定时器中断 (如 1ms) 或主循环中以固定频率调用
 */
void gimbal_control_loop(void)
{
    if (!is_control_init) return;

    // 1. 计算这一毫秒应该在哪 (规划器计算)
    float yaw_setpoint = ramp_calc(&yaw_ramp, CONTROL_DT);
    float pitch_setpoint = ramp_calc(&pitch_ramp, CONTROL_DT);

    // 2. 获取电机句柄
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);
    DJI_Motor_Instance* pitch_motor = dji_motor_get_instance(GIMBAL_MOTOR_PITCH_ID);

    // 3. 将规划出的“平滑位置”传给 PID 控制器
    if (yaw_motor && yaw_motor->is_online) {
        dji_motor_set_location(yaw_motor, (int32_t)yaw_setpoint);
    }

    if (pitch_motor && pitch_motor->is_online) {
        dji_motor_set_location(pitch_motor, (int32_t)pitch_setpoint);
    }
}