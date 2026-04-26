#include "robot_data.h"
#include "fdcan_bsp.h"
#include <math.h>
#include <string.h> // for memcpy
#include "mechanism_task.h"
// 引用 CAN 句柄
extern FDCAN_HandleTypeDef hfdcan3; 

// 实例化全局变量
Robot_Pose_t   g_robot_pose;
Robot_Target_t g_robot_target;

// 定义相关 CAN ID (建议放在头文件中，这里为了演示直接定�?
#define CAN_ID_POSE_PART1  0xAA  // x, y
#define CAN_ID_POSE_PART2  0xAB  // angle, vx
#define CAN_ID_POSE_PART3  0xAC  // vy, vr
#define CAN_ID_PC_TARGET   0x100 // target_x, target_y
#define CAN_ID_PC_PAN_TILT 0x200 // YAW.PITCH

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

static uint8_t s_pose_angle_initialized = 0U;
static float s_pose_last_wrapped_angle = 0.0f;
static float s_pose_unwrapped_angle = 0.0f;

static float prv_unwrap_pose_angle(float wrapped_angle)
{
    float delta = 0.0f;

    if (s_pose_angle_initialized == 0U)
    {
        s_pose_angle_initialized = 1U;
        s_pose_last_wrapped_angle = wrapped_angle;
        s_pose_unwrapped_angle = wrapped_angle;
        return s_pose_unwrapped_angle;
    }

    delta = wrapped_angle - s_pose_last_wrapped_angle;
    while (delta > M_PI)
    {
        delta -= 2.0f * M_PI;
    }
    while (delta < -M_PI)
    {
        delta += 2.0f * M_PI;
    }

    s_pose_last_wrapped_angle = wrapped_angle;
    s_pose_unwrapped_angle += delta;
    return s_pose_unwrapped_angle;
}

// =============================================================
//  静态回调函�?
// =============================================================

/**
 * @brief 处理位姿第一部分 (ID: 0xAA)
 * @note  数据: [x, y] (8 bytes)
 */
static void Callback_Pose_Part1_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&g_robot_pose.x, &rx_data[0], 4);
        memcpy(&g_robot_pose.y, &rx_data[4], 4);
    }
}

/**
 * @brief 处理位姿第二部分 (ID: 0xAB)
 * @note  数据: [angle, vx] (8 bytes)
 */
static void Callback_Pose_Part2_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        float wrapped_angle = 0.0f;

        memcpy(&wrapped_angle,      &rx_data[0], 4);
        g_robot_pose.angle = prv_unwrap_pose_angle(wrapped_angle);
        memcpy(&g_robot_pose.vx,    &rx_data[4], 4);
    }
}

/**
 * @brief 处理位姿第三部分 (ID: 0xAC)
 * @note  数据: [vy, vr] (8 bytes)
 * @note  这是最后一帧，更新时间�?
 */
static void Callback_Pose_Part3_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&g_robot_pose.vy, &rx_data[0], 4);
        memcpy(&g_robot_pose.vr, &rx_data[4], 4);
        
        // 假设收到最后一帧代表一组数据更新完�?
        g_robot_pose.update_tick = HAL_GetTick(); 
    }
}
/**
 * @brief 处理上位机目标点数据 (ID: 0x100)
 * @note  数据长度 8字节，格式：[target_x, target_y]
 */
static void Callback_PC_Target_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    // 检查数据长�?(8字节)
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&g_robot_target.target_x, &rx_data[0], 4);
        memcpy(&g_robot_target.target_y, &rx_data[4], 4);
        g_robot_target.update_tick = HAL_GetTick();
        g_robot_target.is_valid = 1U;
        g_robot_target.is_updated = 1U;
    }
}


static void Callback_PC_Pan_Tilt_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    // 协议定义 DLC �?8，但有效数据可能只有�?字节
    // 即使 DLC=8，我们也只解析前4字节
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_4) 
    {
        int16_t yaw_cdeg;
        int16_t pitch_cdeg;

        // 1. 提取 int16 数据 (小端�?
        memcpy(&yaw_cdeg,   &rx_data[0], 2);
        memcpy(&pitch_cdeg, &rx_data[2], 2);

        // 2. 转换�?float 角度 (centi-deg -> deg)
        // 注意：robot_data.c 需要引�?mechanism_task.h 或声明这两个外部变量
        extern float current_yaw_speed;
        extern float current_pitch_target_deg;

        current_yaw_speed   = (float)yaw_cdeg / 100.0f;
        current_pitch_target_deg = (float)pitch_cdeg / 100.0f;
    }
}

// =============================================================
//  初始化函�?
// =============================================================

void Robot_Data_Init(void)
{
    // 1. 清空数据结构
    memset(&g_robot_pose, 0, sizeof(g_robot_pose));
    memset(&g_robot_target, 0, sizeof(g_robot_target));
    s_pose_angle_initialized = 0U;
    s_pose_last_wrapped_angle = 0.0f;
    s_pose_unwrapped_angle = 0.0f;

    // 2. 注册位姿监听 Part 1 (0xAA)
    static FDCAN_Dispatch_t disp_pos1;
    disp_pos1.id_type = FDCAN_STANDARD_ID;
    disp_pos1.id = CAN_ID_POSE_PART1;
    disp_pos1.instance_ptr = NULL;
    disp_pos1.handler = Callback_Pose_Part1_Handler;
    fdcan_bsp_register(&disp_pos1, &hfdcan3);

    // 3. 注册位姿监听 Part 2 (0xAB)
    static FDCAN_Dispatch_t disp_pos2;
    disp_pos2.id_type = FDCAN_STANDARD_ID;
    disp_pos2.id = CAN_ID_POSE_PART2;
    disp_pos2.instance_ptr = NULL;
    disp_pos2.handler = Callback_Pose_Part2_Handler;
    fdcan_bsp_register(&disp_pos2, &hfdcan3);

    // 4. 注册位姿监听 Part 3 (0xAC)
    static FDCAN_Dispatch_t disp_pos3;
    disp_pos3.id_type = FDCAN_STANDARD_ID;
    disp_pos3.id = CAN_ID_POSE_PART3;
    disp_pos3.instance_ptr = NULL;
    disp_pos3.handler = Callback_Pose_Part3_Handler;
    fdcan_bsp_register(&disp_pos3, &hfdcan3);

    // 5. 注册上位机目标点监听 (0x100)
    static FDCAN_Dispatch_t disp_pc;
    disp_pc.id_type = FDCAN_STANDARD_ID;
    disp_pc.id = CAN_ID_PC_TARGET;
    disp_pc.instance_ptr = NULL;
    disp_pc.handler = Callback_PC_Target_Handler;
    fdcan_bsp_register(&disp_pc, &hfdcan3);
		
		    // 6. 注册上位机目标点监听 (0x200)
    static FDCAN_Dispatch_t disp_pc_pan_tlit; // 声明了新变量
    disp_pc_pan_tlit.id_type = FDCAN_STANDARD_ID;
    disp_pc_pan_tlit.id = CAN_ID_PC_PAN_TILT;
    disp_pc_pan_tlit.instance_ptr = NULL;
    disp_pc_pan_tlit.handler = Callback_PC_Pan_Tilt_Handler;
    fdcan_bsp_register(&disp_pc_pan_tlit, &hfdcan3);
}
