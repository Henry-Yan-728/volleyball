/**
  ******************************************************************************
  * @file    : chassis_cybergear.c
  * @brief   : 小米CyberGear底盘控制封装实现
  ******************************************************************************
  */

#include "chassis_cybergear.h"
#include "trajectory_planner.h"
#include "FreeRTOS.h"
#include "task.h"

/* 初始化延时（ms），保证每条 CAN 指令被电机正确处理 */
#define INIT_DELAY_MS   (10U)

/* 模块私有变量 */
static CyberGear_Motor_Instance *s_motor = NULL;

/* ------------------------------------------------------------------ */

int8_t chassis_cybergear_init(uint8_t motor_index)
{
    /* 1. 获取电机实例 */
    s_motor = cybergear_motor_get_instance(motor_index);
    if (s_motor == NULL)
        return -1;

    /* 2. 先发 stop，确保电机从任何状态复位到安全状态 */
    cybergear_motor_stop(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    /* 3. 切换到位置控制模式 */
    cybergear_motor_set_mode(s_motor, MOTOR_CONTROL_MODE_POSITION);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    /* 4. 使能电机 */
    cybergear_motor_enable(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    /* 5. 启动轨迹规划器 */
    int8_t ret = trajectory_planner_start(s_motor);
    if (ret != 0)
    {
        cybergear_motor_stop(s_motor);
        s_motor = NULL;
        return -1;
    }

    return 0;
}

/* ------------------------------------------------------------------ */

void chassis_control(float angle_deg)
{
    if (s_motor == NULL) return;

    /* 度 → 弧度，底层会自动截断超范围值 */
    float target_rad = cybergear_motor_degree2rad(angle_deg);

    trajectory_planner_set_target(target_rad);
}

/* ------------------------------------------------------------------ */

void chassis_control_speed(float speed_rad_s)
{
    if (s_motor == NULL) return;

    /* 硬限幅，防止超出电机额定范围 */
    if (speed_rad_s >  30.0f) speed_rad_s =  30.0f;
    if (speed_rad_s < -30.0f) speed_rad_s = -30.0f;

    cybergear_motor_set_speed(s_motor, speed_rad_s);
}

/* ------------------------------------------------------------------ */

void chassis_cybergear_stop(void)
{
    if (s_motor == NULL) return;

    /* 先将规划目标清零（让规划器平滑减速到 0），再发 stop 硬停 */
    trajectory_planner_set_target(s_motor->measure.angle);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));
    cybergear_motor_stop(s_motor);
}

/* ------------------------------------------------------------------ */

float chassis_get_angle_rad(void)
{
    if (s_motor == NULL) return 0.0f;
    return s_motor->measure.angle;
}
