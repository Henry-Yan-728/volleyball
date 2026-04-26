/**
  ******************************************************************************
  * @file    : cybergear_speed_planner.c
  * @brief   : 灏忕背 CyberGear 鐢垫満閫熸帶瑙勫垝鍣ㄥ疄鐜?  ******************************************************************************
  */

#include "cybergear_speed_planner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>

/* ----------------------- 绉佹湁瀹?----------------------- */

#define TASK_STACK_SIZE   (384U)
#define TASK_PRIORITY     (tskIDLE_PRIORITY + 3)

/* ----------------------- 绉佹湁鍙橀噺 ----------------------- */

static TaskHandle_t        s_task_handle   = NULL;
static SemaphoreHandle_t   s_target_mutex  = NULL;

static CyberGear_Motor_Instance *s_motor                = NULL;
static volatile float            s_target_speed         = 0.0f;  // 鐢ㄦ埛鏈熸湜閫熷害 (rad/s)
static volatile float            s_current_speed        = 0.0f;  // 瑙勫垝鍣ㄥ綋鍓嶈緭鍑洪€熷害 (rad/s)
static volatile uint8_t          s_emergency            = 0;     // 绱ф€ュ仠姝㈡爣蹇?
/* ----------------------- 绉佹湁鍑芥暟 ----------------------- */

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

        /* --- 绱ф€ュ仠姝㈠鐞?--- */
        if (s_emergency)
        {
            s_current_speed = 0.0f;
            s_emergency = 0;            /* 鍏堟竻鏍囧織锛屽啀鎸傝捣锛岄槻姝?resume 鍚庨噸澶嶈Е鍙?*/
            cybergear_motor_set_speed(s_motor, 0.0f);
            cybergear_motor_stop(s_motor);
            /* 浠诲姟鎸傝捣鑷韩锛岀瓑寰呴噸鏂拌皟鐢?start 鏃?resume */
            vTaskSuspend(NULL);
            /* resume 鍚庝粠姝ゅ缁х画锛岄噸缃?xLastWakeTime 閬垮厤 delay 婧㈠嚭琛ュ伩 */
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        /* --- 璇诲彇鐩爣閫熷害锛堢嚎绋嬪畨鍏級--- */
        float local_target;
        xSemaphoreTake(s_target_mutex, portMAX_DELAY);
        local_target = s_target_speed;
        xSemaphoreGive(s_target_mutex);

        /* --- 鍔犻€熷害闄愬箙鏂滅巼瑙勫垝 --- */
        float error = local_target - s_current_speed;

        if (error > accel_step)
        {
            s_current_speed += accel_step;          /* 鍔犻€?*/
        }
        else if (error < -accel_step)
        {
            s_current_speed -= accel_step;          /* 鍑忛€?*/
        }
        else
        {
            s_current_speed = local_target;         /* 宸插埌杈剧洰鏍囷紝淇濇寔 */
        }

        /* --- 纭檺骞呬繚鎶?--- */
        s_current_speed = prv_clamp(s_current_speed,
                                    -SPEED_PLANNER_MAX_SPEED,
                                     SPEED_PLANNER_MAX_SPEED);

        /* --- 涓嬪彂缁欑數鏈?--- */
        cybergear_motor_set_speed(s_motor, s_current_speed);
    }
}

/* ----------------------- 鍏叡鎺ュ彛瀹炵幇 ----------------------- */

int8_t cybergear_speed_planner_init(void)
{
    /* 浜掓枼閿?*/
    s_target_mutex = xSemaphoreCreateMutex();
    if (s_target_mutex == NULL) return -1;

    /* 鍒涘缓浠诲姟锛屽垵濮嬫寕璧凤紝绛夊緟 start 鏃跺啀鎭㈠ */
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

    /* 鐩爣閫熷害娓呴浂锛岄槻姝㈠惎鍔ㄧ灛闂翠贡杞?*/
    xSemaphoreTake(s_target_mutex, portMAX_DELAY);
    s_target_speed = 0.0f;
    xSemaphoreGive(s_target_mutex);

    s_current_speed = 0.0f;

    /* 鍒囨崲鍒伴€熷害妯″紡骞朵娇鑳?*/
    cybergear_motor_set_mode(s_motor, MOTOR_CONTROL_MODE_SPEED);
    cybergear_motor_enable(s_motor);

    vTaskResume(s_task_handle);
    return 0;
}

void cybergear_speed_planner_set_target_speed(float target_speed_rad_s)
{
    if (s_target_mutex == NULL) return;

    /* 闄愬箙鍚庡啓鍏?*/
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
    /* 鍏堟竻闆剁洰鏍囷紝鍐嶈缃爣蹇楋紝浠诲姟涓嬫鍞ら啋鏃剁珛鍗冲鐞?*/
    if (s_target_mutex != NULL)
    {
        xSemaphoreTake(s_target_mutex, portMAX_DELAY);
        s_target_speed = 0.0f;
        xSemaphoreGive(s_target_mutex);
    }
    s_emergency = 1;
}

