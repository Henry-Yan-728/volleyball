#include "robot_data.h"
#include "fdcan_bsp.h"
#include <string.h> // for memcpy

// 引用 CAN2 句柄 (CubeMX生成的)
extern FDCAN_HandleTypeDef hfdcan2; 

// 实例化全局变量
Robot_Pose_t   g_robot_pose;
Robot_Target_t g_robot_target;

// =============================================================
//  静态回调函数 (底层收到数据后会自动调用这里)
// =============================================================

/**
 * @brief 处理定位板数据 (ID: 0xAA)
 * @note  数据长度 24字节，格式：[x, y, ang, vx, vy, vr] (均为 float)
 */
static void Callback_Position_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    // 检查数据长度是否足够 (24字节)
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_24)
    {
        // 直接内存拷贝，解析 6 个 float
        memcpy(&g_robot_pose.x,     &rx_data[0],  4);
        memcpy(&g_robot_pose.y,     &rx_data[4],  4);
        memcpy(&g_robot_pose.angle, &rx_data[8],  4);
        memcpy(&g_robot_pose.vx,    &rx_data[12], 4);
        memcpy(&g_robot_pose.vy,    &rx_data[16], 4);
        memcpy(&g_robot_pose.vr,    &rx_data[20], 4);
        
        g_robot_pose.update_tick = HAL_GetTick(); // 记录更新时间
    }
}

/**
 * @brief 处理上位机目标点数据 (ID: 0x100)
 * @note  数据长度 8字节，格式：[target_x, target_y]
 */
static void Callback_PC_Target_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&g_robot_target.target_x, &rx_data[0], 4);
        memcpy(&g_robot_target.target_y, &rx_data[4], 4);
        
        g_robot_target.is_updated = 1; // 标记收到新目标
    }
}

// =============================================================
//  初始化函数
// =============================================================

void Robot_Data_Init(void)
{
    // 1. 清空数据结构
    memset(&g_robot_pose, 0, sizeof(g_robot_pose));
    memset(&g_robot_target, 0, sizeof(g_robot_target));

    // 2. 注册定位板监听 (CAN2, ID 0xAA)
    static FDCAN_Dispatch_t disp_pos;
    disp_pos.id_type = FDCAN_STANDARD_ID;
    disp_pos.id = CAN_ID_POSITION_DATA;
    disp_pos.instance_ptr = NULL; // 不需要特定实例
    disp_pos.handler = Callback_Position_Handler;
    
    fdcan_bsp_register(&disp_pos, &hfdcan2);

    // 3. 注册上位机监听 (CAN2, ID 0x100)
    static FDCAN_Dispatch_t disp_pc;
    disp_pc.id_type = FDCAN_STANDARD_ID;
    disp_pc.id = CAN_ID_PC_SET_TARGET;
    disp_pc.instance_ptr = NULL;
    disp_pc.handler = Callback_PC_Target_Handler;
    
    fdcan_bsp_register(&disp_pc, &hfdcan2);
}