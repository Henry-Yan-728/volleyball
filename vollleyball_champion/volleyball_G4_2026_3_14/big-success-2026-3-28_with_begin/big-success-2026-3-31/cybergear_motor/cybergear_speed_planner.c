/**
  ******************************************************************************
  * @file    : cybergear_speed_planner.c
  * @brief   : 小米 CyberGear 电机速控规划器实现
  ******************************************************************************
  */

#include "cybergear_speed_planner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>

/* ----------------------- 私有宏 ----------------------- */

#define TASK_STACK_SIZE   (256U)
#define TASK_PRIORITY     (tskIDLE_PRIORITY + 3)

/* ----------------------- 私有变量 ----------------------- */

static TaskHandle_t        s_task_handle   = NULL;
static SemaphoreHandle_t   s_target_mutex  = NULL;

static CyberGear_Motor_Instance *s_motor                = NULL;
static volatile float            s_target_speed         = 0.0f;  // 用户期望速度 (rad/s)
static volatile float            s_current_speed        = 0.0f;  // 规划器当前输出速度 (rad/s)
static volatile uint8_t          s_emergency            = 0;     // 紧急停止标志

/* ----------------------- 私有函数 ----------------------- */

static float prv_clamp(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

static void prv_speed_planner_task(void *pvParameters)
{
    (void)pvParameters;

    const TickType_t period_ticks = pdMS_TO_TICKS(SPEED_PLANNER_CONTROL_PERIOD);
    const float      dt           = (float)SPEED_PLANNER_CONTROL_PERIOD / 1000.0f;
    const float      accel_step   = SPEED_PLANNER_MAX_ACCEL * dt;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, period_ticks);

        /* --- 紧急停止处理 --- */
        if (s_emergency)
        {
            s_current_speed = 0.0f;
            s_emergency = 0;            /* 先清标志，再挂起，防止 resume 后重复触发 */
            cybergear_motor_set_speed(s_motor, 0.0f);
            cybergear_motor_stop(s_motor);
            /* 任务挂起自身，等待重新调用 start 时 resume */
            vTaskSuspend(NULL);
            /* resume 后从此处继续，重置 xLastWakeTime 避免 delay 溢出补偿 */
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        /* --- 读取目标速度（线程安全）--- */
        float local_target;
        xSemaphoreTake(s_target_mutex, portMAX_DELAY);
        local_target = s_target_speed;
        xSemaphoreGive(s_target_mutex);

        /* --- 加速度限幅斜率规划 --- */
        float error = local_target - s_current_speed;

        if (error > accel_step)
        {
            s_current_speed += accel_step;          /* 加速 */
        }
        else if (error < -accel_step)
        {
            s_current_speed -= accel_step;          /* 减速 */
        }
        else
        {
            s_current_speed = local_target;         /* 已到达目标，保持 */
        }

        /* --- 硬限幅保护 --- */
        s_current_speed = prv_clamp(s_current_speed,
                                    -SPEED_PLANNER_MAX_SPEED,
                                     SPEED_PLANNER_MAX_SPEED);

        /* --- 下发给电机 --- */
        cybergear_motor_set_speed(s_motor, s_current_speed);
    }
}

/* ----------------------- 公共接口实现 ----------------------- */

int8_t cybergear_speed_planner_init(void)
{
    /* 互斥锁 */
    s_target_mutex = xSemaphoreCreateMutex();
    if (s_target_mutex == NULL) return -1;

    /* 创建任务，初始挂起，等待 start 时再恢复 */
    BaseType_t ret = xTaskCreate(prv_speed_planner_task,
                                 "SpeedPlanner",
                                 TASK_STACK_SIZE,
                                 NULL,
                                 TASK_PRIORITY,
                                 &s_task_handle);
    if (ret != pdPASS) return -1;

    vTaskSuspend(s_task_handle);
    return 0;
}

int8_t cybergear_speed_planner_start(CyberGear_Motor_Instance *motor)
{
    if (motor == NULL || s_task_handle == NULL || s_target_mutex == NULL)
        return -1;

    s_motor = motor;
    s_emergency = 0;

    /* 目标速度清零，防止启动瞬间乱转 */
    xSemaphoreTake(s_target_mutex, portMAX_DELAY);
    s_target_speed = 0.0f;
    xSemaphoreGive(s_target_mutex);

    s_current_speed = 0.0f;

    /* 切换到速度模式并使能 */
    cybergear_motor_set_mode(s_motor, MOTOR_CONTROL_MODE_SPEED);
    cybergear_motor_enable(s_motor);

    vTaskResume(s_task_handle);
    return 0;
}

void cybergear_speed_planner_set_target_speed(float target_speed_rad_s)
{
    if (s_target_mutex == NULL) return;

    /* 限幅后写入 */
    float clamped = prv_clamp(target_speed_rad_s,
                              -SPEED_PLANNER_MAX_SPEED,
                               SPEED_PLANNER_MAX_SPEED);

    xSemaphoreTake(s_target_mutex, portMAX_DELAY);
    s_target_speed = clamped;
    xSemaphoreGive(s_target_mutex);
}

float cybergear_speed_planner_get_current_speed(void)
{
    return s_current_speed;
}

void cybergear_speed_planner_emergency_stop(void)
{
    /* 先清零目标，再设置标志，任务下次唤醒时立即处理 */
    if (s_target_mutex != NULL)
    {
        xSemaphoreTake(s_target_mutex, portMAX_DELAY);
        s_target_speed = 0.0f;
        xSemaphoreGive(s_target_mutex);
    }
    s_emergency = 1;
}
