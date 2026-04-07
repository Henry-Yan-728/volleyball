#include "Pan_Tilt_control.h"
#include "ramp.h"      
#include "dji_motor.h"
#include "stdio.h"

// 定义两个轴的规划器
static Ramp_Handle_t yaw_ramp;
static Ramp_Handle_t pitch_ramp;

// 标记是否初始化过
static uint8_t is_control_init = 0;

// 控制循环周期 
#define CONTROL_DT 0.001f 

// 初始化函数 
void gimbal_control_init(void)
{
//    // Yaw轴 (GM6020): 直驱
////    ramp_init(&yaw_ramp, 26000.0f, 15000.0f, 15000.0f);
//    ramp_init(&yaw_ramp, 80000.0f, 60000.0f, 40000.0f);

//    // Pitch轴 (M2006): 减速比 36:1
////    ramp_init(&pitch_ramp, 200000.0f, 160000.0f, 160000.0f);
//    // 加速度建议设为速度的 1.5~2 倍，响应更跟手
//// 速度设为 800000 (约等于电机转子 5800 RPM，非常接近你的 PID 限制 6000)
//    ramp_init(&pitch_ramp, 1500000.0f, 800000.0f, 1200000.0f);    
//    // 同步初始位置 
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);
////    if(yaw_motor && yaw_motor->is_online) {
////        yaw_ramp.current_pos = (float)yaw_motor->measure.total_angle;
////        yaw_ramp.target_pos = yaw_ramp.current_pos;
////    }
//    
//    is_control_init = 1;
	dji_motor_set_location(yaw_motor,-2597);
}

/**
 * @brief 用户调用此函数设置目标角度 (输入为角度制)
 */
void gimbal_set_angle(float yaw_deg, float pitch_deg)
{
	    // 2. 获取电机句柄
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);
    DJI_Motor_Instance* pitch_motor = dji_motor_get_instance(GIMBAL_MOTOR_PITCH_ID);
//    // 2. 将目标值喂给规划器
//    ramp_set_target(&yaw_ramp, (float)yaw_target_cnt);
//    ramp_set_target(&pitch_ramp, (float)pitch_target_cnt);
	  dji_motor_set_speed(yaw_motor,yaw_deg);
		dji_motor_set_speed(pitch_motor,pitch_deg);
}

/**
 * @brief 核心控制任务 loop
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