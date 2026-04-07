#include "Pan_Tilt_control.h"

#include "chassis_cybergear.h"
#include "dji_motor.h"
#include "cybergear_speed_planner.h"

/**
 * @brief 云台控制初始化
 */
void gimbal_control_init(void)
{
    // 速度模式下不需要同步当前位置，只需确保初始速度为 0 即可防止疯转
    gimbal_set_speed(0, 0);
}

/**
 * @brief 设置云台目标速度
 * @param yaw_speed_rpm   Yaw轴目标速度 (RPM)
 * @param pitch_speed_rpm Pitch轴目标速度 (RPM)
 */
void gimbal_set_speed(int16_t yaw_speed_rpm, int16_t pitch_speed_rpm)
{
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);

    // 直接调用大疆电机底层的速度控制接口
    if (yaw_motor) {
        dji_motor_set_speed(yaw_motor, yaw_speed_rpm);
    }
        chassis_control(pitch_speed_rpm);
}

/**
 * @brief 获取云台当前角度
 * @param yaw_angle_deg   输出参数：Yaw轴当前角度(度)
 * @param pitch_angle_deg 输出参数：Pitch轴当前角度(度)
 */
void gimbal_get_angles(float* yaw_angle_deg, float* pitch_angle_deg)
{
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);
    DJI_Motor_Instance* pitch_motor = dji_motor_get_instance(GIMBAL_MOTOR_PITCH_ID);

    if (yaw_angle_deg && yaw_motor) {
        // Yaw 是 GM6020 直驱，减速比 1.0
        *yaw_angle_deg = dji_encoder_to_angle(yaw_motor->measure.total_angle, RATIO_GM6020);
    }
    
    if (pitch_angle_deg && pitch_motor) {
        // Pitch 是 M2006，减速比 36.0
        *pitch_angle_deg = chassis_get_angle_rad();
    }
}