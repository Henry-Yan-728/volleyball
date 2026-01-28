#include "motor_logic.h"
#include "cmsis_os.h"      
#include <stdio.h>
#include <stdlib.h>

// --- 外部依赖声明 ---
extern float actual_position[3]; 
extern void motor_control(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd);
#define CONTROL_TASK_FREQUENCY_MS 10 
// --- 私有配置参数 ---
static const float KPOS_GAIN = 0.7f; 
static const float KSPD_GAIN = 0.2f;  
static const float DESIRED_TORQUE = 0.0f; 

// --- 私有状态变量 ---
static TrapezoidProfile motor_profile;
static bool is_motion_active = false; 
static TickType_t motion_start_tick = 0;
static float position_offset = 0.0f; 
static uint32_t current_motor_id = 1; 

// ==========================================
// part 1: 纯数学算法实现 (私有函数)
// ==========================================
static void Profile_Init(TrapezoidProfile *profile, float current_pos, float target_pos, float max_v, float max_a, float max_d)
{
    profile->start_position = current_pos;
    profile->target_position = target_pos;
    profile->max_speed = fabsf(max_v);
    profile->acceleration = fabsf(max_a);
    profile->deceleration = fabsf(max_d);
    profile->is_finished = false;

    float distance = target_pos - current_pos;
    profile->total_distance = fabsf(distance);
    profile->direction = (distance >= 0) ? 1.0f : -1.0f;

    if (profile->acceleration <= 0.0f || profile->total_distance < 0.0001f) {
        profile->t_total = 0.0f;
        profile->is_finished = true;
        return;
    }

    // 计算三角形曲线的最大速度
    float v_limit = sqrtf(profile->total_distance * profile->acceleration); // 简化为加减速相同的情况
    
    // 实际达到的峰值速度
    profile->peak_speed = fminf(profile->max_speed, v_limit);

    // 计算时间
    profile->t_accel = profile->peak_speed / profile->acceleration;
    float dist_accel = 0.5f * profile->acceleration * profile->t_accel * profile->t_accel;
    float dist_const = profile->total_distance - 2 * dist_accel; // 假设加减速对称

    profile->t_const = (dist_const > 0) ? (dist_const / profile->peak_speed) : 0.0f;
    profile->t_total = 2 * profile->t_accel + profile->t_const;
}

static void Profile_Update(TrapezoidProfile *profile, uint32_t elapsed_time_ms, float *out_position, float *out_speed)
{
    float t = (float)elapsed_time_ms / 1000.0f;
    float t_accel = profile->t_accel;
    float t_const = profile->t_const;
    float t_decel_start = t_accel + t_const;
    
    if (profile->is_finished || t >= profile->t_total) {
        *out_position = profile->target_position;
        *out_speed = 0.0f;
        profile->is_finished = true;
        return;
    }
    
    float pos = 0.0f;
    float speed = 0.0f;
    float peak_v = profile->peak_speed;
    float A = profile->acceleration;
    
    if (t < t_accel) {
        // 加速
        speed = A * t;
        pos = 0.5f * A * t * t;
    } else if (t < t_decel_start) {
        // 匀速
        float P_accel = 0.5f * A * t_accel * t_accel;
        speed = peak_v;
        pos = P_accel + peak_v * (t - t_accel);
    } else {
        // 减速
        float t_rel = t - t_decel_start;
        float P_decel_start = profile->total_distance - 0.5f * A * (profile->t_total - t_decel_start) * (profile->t_total - t_decel_start);
        speed = peak_v - A * t_rel;
        pos = P_decel_start + peak_v * t_rel - 0.5f * A * t_rel * t_rel;
        if (speed < 0) speed = 0;
    }

    *out_position = profile->start_position + pos * profile->direction;
    *out_speed = speed * profile->direction;
}

// ==========================================
// part 2: 业务逻辑与硬件控制 (公开函数)
// ==========================================

void Motor_InitZeroing(uint32_t motorId)
{
    if (motorId >= 3) motorId = 0;
    current_motor_id = motorId;
    
    osDelay(500); // 等待硬件稳定

    printf("Starting Zeroing for Motor %u...\n", (unsigned int)current_motor_id);

    // 连续发送命令锁定位置，获取稳定的 position_offset
    for (int i = 0; i < 100; i++) { 
        float current_abs_pos = actual_position[current_motor_id]; 
        motor_control(current_motor_id, DESIRED_TORQUE, 0.0f, current_abs_pos, KPOS_GAIN, KSPD_GAIN);
        osDelay(10); 
    }
    
    position_offset = actual_position[current_motor_id]; 
    printf("Motor %u zeroed. Offset: %.3f rad\n", (unsigned int)current_motor_id, position_offset);
}

void Motor_MoveTo(float target_pos, float max_v, float max_a)
{
    float current_pos_absolute = (current_motor_id < 3) ? actual_position[current_motor_id] : 0.0f;
    float current_pos_relative = current_pos_absolute - position_offset;

    // 初始化曲线 (假设加减速相同)
    Profile_Init(&motor_profile, current_pos_relative, target_pos, max_v, max_a, max_a);
    
    motion_start_tick = osKernelGetTickCount();
    is_motion_active = true;
    
    printf("Move Start -> Target: %.3f rad\n", target_pos);
}

void Motor_UpdateLoop(uint32_t elapsed_ms_period)
{
    if (!is_motion_active) return;

    // 1. 计算时间
    uint32_t elapsed_time_ticks = osKernelGetTickCount() - motion_start_tick;
    // 假设 tick 是 1ms
    uint32_t elapsed_time_ms = elapsed_time_ticks; 

    float desired_position_relative = 0.0f;
    float desired_speed = 0.0f;

    // 2. 算法计算
    Profile_Update(&motor_profile, elapsed_time_ms, &desired_position_relative, &desired_speed);

    // 3. 转为绝对位置
    float desired_position_absolute = desired_position_relative + position_offset;

    // 4. 发送硬件指令
    motor_control(current_motor_id, DESIRED_TORQUE, desired_speed, desired_position_absolute, KPOS_GAIN, KSPD_GAIN);

    // 5. 检查是否结束
    if (motor_profile.is_finished)
    {
        printf("Motion Finished.\n");
        is_motion_active = false;
    }
}

bool Motor_IsMoving(void)
{
    return is_motion_active;
}

///* USER CODE BEGIN Header_StartTask02 */
///**
//* @brief  控制循环任务：负责高频执行 Update,主程序里面单开一个任务
//*/
///* USER CODE END Header_StartTask02 */
//void StartTask02(void *argument)
//{
//  /* USER CODE BEGIN StartTask02 */
//  TickType_t xLastWakeTime;
//  const uint32_t xFrequency_ticks = pdMS_TO_TICKS(CONTROL_TASK_FREQUENCY_MS);
//  xLastWakeTime = osKernelGetTickCount(); 

//  /* Infinite loop */
//  for(;;)
//  {
//    xLastWakeTime += xFrequency_ticks;
//    osDelayUntil(xLastWakeTime);

//    // 只需要调用这一行
//    Motor_UpdateLoop(CONTROL_TASK_FREQUENCY_MS);
//  }
//  /* USER CODE END StartTask02 */
//}
