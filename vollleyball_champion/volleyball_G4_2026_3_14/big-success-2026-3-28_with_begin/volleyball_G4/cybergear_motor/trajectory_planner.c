/**
  ******************************************************************************
  * @file           : trajectory_planner.c
  * @brief          : 底盘旋转平滑轨迹规划器 (速控模式重构版)
  ******************************************************************************
  */

#include "trajectory_planner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cybergear_motor.h"

/* ------------------------- 私有宏定义 ------------------------- */

// 控制周期 (10ms -> 100Hz)
#define PLANNER_CONTROL_PERIOD_MS   (10)
#define PLANNER_TASK_STACK_SIZE     (256) 
#define PLANNER_TASK_PRIORITY       (tskIDLE_PRIORITY + 3)

// --- 规划器调参区 ---
#define PLANNER_MAX_VELOCITY        (5.00f) // 最大平滑速度 (rad/s)
#define PLANNER_MAX_ACCELERATION    (2.00f) // 最大加速度 (rad/s^2)
#define PLANNER_POS_KP              (15.0f) // 接近目标时的位置环比例增益

/* ----------------------- 模块私有变量 ----------------------- */

static TaskHandle_t xPlannerTaskHandle = NULL;
static SemaphoreHandle_t g_target_mutex = NULL; // 重命名以避免与CAN锁混淆

static float g_final_target_pos = 0.0f;
static CyberGear_Motor_Instance* g_motor = NULL;

static float g_current_ramped_vel = 0.0f;   // 规划器输出的【速度】

// 辅助函数：获取符号
static float sign(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/* --------------------- 私有任务函数 --------------------- */

static void prv_planner_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(PLANNER_CONTROL_PERIOD_MS);
    const float dt = (float)PLANNER_CONTROL_PERIOD_MS / 1000.0f; 
    const float accel_step = PLANNER_MAX_ACCELERATION * dt;      

    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. 安全获取目标位置
        float local_target;
        xSemaphoreTake(g_target_mutex, portMAX_DELAY);
        local_target = g_final_target_pos;
        xSemaphoreGive(g_target_mutex);
        
        // 2. 获取电机真实的当前位置 (注意: 电机本体反馈范围是 ±4π)
        // 如果需要无限圈数，需在此处增加 unwrap 展开逻辑
        float current_actual_pos = g_motor->measure.angle;
        
        // 3. 计算位置误差
        float pos_error = local_target - current_actual_pos;
        float target_vel = 0.0f;

        // 4. 【核心闭环逻辑】生成期望速度
        float small_err_threshold = PLANNER_MAX_ACCELERATION / (PLANNER_POS_KP * PLANNER_POS_KP); 
        
        if (fabsf(pos_error) < 0.05f) 
        {
            // 误差极小时，直接给0，防止电机在目标点附近细微震荡发热
            target_vel = 0.0f;
        }
        else if (fabsf(pos_error) < small_err_threshold) 
        {
            // 线性区：使用 P 控制平滑贴近目标
            target_vel = pos_error * PLANNER_POS_KP;
        }
        else 
        {
            // 刹车区/巡航区：基于运动学的根号速度曲线 v = sqrt(2as)
            float max_stop_v = sqrtf(2.0f * PLANNER_MAX_ACCELERATION * fabsf(pos_error));
            // 限制最大巡航速度
            float vel_limit = (max_stop_v < PLANNER_MAX_VELOCITY) ? max_stop_v : PLANNER_MAX_VELOCITY;
            target_vel = sign(pos_error) * vel_limit;
        }

        // 5. 速度斜坡 (限制加速度)
        float vel_error = target_vel - g_current_ramped_vel;
        
        if (fabsf(vel_error) > accel_step) 
        {
            g_current_ramped_vel += sign(vel_error) * accel_step;
        } 
        else 
        {
            g_current_ramped_vel = target_vel;
        }

        // 6. 发送速度指令 (使用速控模式API)
        cybergear_motor_set_speed(g_motor, g_current_ramped_vel);
    }
}

/* ----------------------- 公共接口函数 ----------------------- */

int8_t trajectory_planner_init(void)
{
    g_target_mutex = xSemaphoreCreateMutex();
    if (g_target_mutex == NULL) return -1; 

    BaseType_t xReturned = xTaskCreate(
                                    prv_planner_task,
                                    "PlannerTask",
                                    PLANNER_TASK_STACK_SIZE,
                                    NULL,
                                    PLANNER_TASK_PRIORITY,
                                    &xPlannerTaskHandle
                                    );

    if (xReturned != pdPASS) return -1;

    if (xPlannerTaskHandle != NULL) vTaskSuspend(xPlannerTaskHandle);
    
    return 0; 
}

int8_t trajectory_planner_start(CyberGear_Motor_Instance* motor)
{
    if (motor == NULL || xPlannerTaskHandle == NULL || g_target_mutex == NULL) return -1; 

    g_motor = motor;

    // --- 【关键修改点】---
    // 强制将电机设置为速度模式，并重新使能
    cybergear_motor_set_mode(g_motor, MOTOR_CONTROL_MODE_SPEED);
    cybergear_motor_enable(g_motor);

    // 同步初始目标为当前位置，防止启动瞬间乱转
    xSemaphoreTake(g_target_mutex, portMAX_DELAY);
    g_final_target_pos = g_motor->measure.angle;
    xSemaphoreGive(g_target_mutex);

    g_current_ramped_vel = 0.0f; // 速度清零

    vTaskResume(xPlannerTaskHandle);
    return 0;
}

void trajectory_planner_set_target(float target_position)
{
    if (g_target_mutex != NULL)
    {
        xSemaphoreTake(g_target_mutex, portMAX_DELAY);
        g_final_target_pos = target_position;
        xSemaphoreGive(g_target_mutex);
    }
}