#include "robot_data.h"

#include <string.h>

#include "fdcan_bsp.h"
#include "mechanism_task.h"

extern FDCAN_HandleTypeDef hfdcan3;

#define ROBOT_POSE_PART1_MASK    0x01U
#define ROBOT_POSE_PART2_MASK    0x02U
#define ROBOT_POSE_PART3_MASK    0x04U
#define ROBOT_POSE_PART_ALL_MASK (ROBOT_POSE_PART1_MASK | ROBOT_POSE_PART2_MASK | ROBOT_POSE_PART3_MASK)

Robot_Pose_t g_robot_pose;
Robot_Target_t g_robot_target;
volatile uint32_t g_pan_tilt_cmd_update_tick = 0U;

static Robot_Pose_t s_robot_pose_staging;
static uint8_t s_robot_pose_parts_mask = 0U;

static void prv_commit_robot_pose_if_ready(void)
{
    if (s_robot_pose_parts_mask == ROBOT_POSE_PART_ALL_MASK)
    {
        s_robot_pose_staging.update_tick = HAL_GetTick();
        g_robot_pose = s_robot_pose_staging;
        s_robot_pose_parts_mask = 0U;
    }
}

static void Callback_Pose_Part1_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        s_robot_pose_parts_mask = 0U;
        memcpy(&s_robot_pose_staging.x, &rx_data[0], 4);
        memcpy(&s_robot_pose_staging.y, &rx_data[4], 4);
        s_robot_pose_parts_mask |= ROBOT_POSE_PART1_MASK;
        prv_commit_robot_pose_if_ready();
    }
}

static void Callback_Pose_Part2_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&s_robot_pose_staging.angle, &rx_data[0], 4);
        memcpy(&s_robot_pose_staging.vx, &rx_data[4], 4);
        s_robot_pose_parts_mask |= ROBOT_POSE_PART2_MASK;
        prv_commit_robot_pose_if_ready();
    }
}

static void Callback_Pose_Part3_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&s_robot_pose_staging.vy, &rx_data[0], 4);
        memcpy(&s_robot_pose_staging.vr, &rx_data[4], 4);
        s_robot_pose_parts_mask |= ROBOT_POSE_PART3_MASK;
        prv_commit_robot_pose_if_ready();
    }
}

static void Callback_PC_Target_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&g_robot_target.target_x, &rx_data[0], 4);
        memcpy(&g_robot_target.target_y, &rx_data[4], 4);
        g_robot_target.is_updated = 1U;
    }
}

static void Callback_PC_Pan_Tilt_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    int16_t yaw_cdeg;
    int16_t pitch_cdeg;
    extern volatile float current_yaw_speed;
    extern volatile float current_pitch_target_deg;

    (void)instance_ptr;

    if (rx_header->DataLength < FDCAN_DLC_BYTES_4)
    {
        return;
    }

    memcpy(&yaw_cdeg, &rx_data[0], sizeof(yaw_cdeg));
    memcpy(&pitch_cdeg, &rx_data[2], sizeof(pitch_cdeg));

    current_yaw_speed = (float)yaw_cdeg / 100.0f;
    current_pitch_target_deg = (float)pitch_cdeg / 100.0f;
    g_pan_tilt_cmd_update_tick = HAL_GetTick();
}

void Robot_Data_Init(void)
{
    memset(&g_robot_pose, 0, sizeof(g_robot_pose));
    memset(&g_robot_target, 0, sizeof(g_robot_target));
    memset(&s_robot_pose_staging, 0, sizeof(s_robot_pose_staging));
    s_robot_pose_parts_mask = 0U;
    g_pan_tilt_cmd_update_tick = 0U;
}

void Robot_Data_Register_Dispatches(void)
{
    static FDCAN_Dispatch_t disp_pos1 = {
        .id_type = FDCAN_STANDARD_ID,
        .id = CAN_ID_POSE_PART1,
        .mask = 0U,
        .instance_ptr = NULL,
        .handler = Callback_Pose_Part1_Handler,
    };
    static FDCAN_Dispatch_t disp_pos2 = {
        .id_type = FDCAN_STANDARD_ID,
        .id = CAN_ID_POSE_PART2,
        .mask = 0U,
        .instance_ptr = NULL,
        .handler = Callback_Pose_Part2_Handler,
    };
    static FDCAN_Dispatch_t disp_pos3 = {
        .id_type = FDCAN_STANDARD_ID,
        .id = CAN_ID_POSE_PART3,
        .mask = 0U,
        .instance_ptr = NULL,
        .handler = Callback_Pose_Part3_Handler,
    };
    static FDCAN_Dispatch_t disp_pc_target = {
        .id_type = FDCAN_STANDARD_ID,
        .id = CAN_ID_PC_SET_TARGET,
        .mask = 0U,
        .instance_ptr = NULL,
        .handler = Callback_PC_Target_Handler,
    };
    static FDCAN_Dispatch_t disp_pc_pan_tilt = {
        .id_type = FDCAN_STANDARD_ID,
        .id = CAN_ID_PC_PAN_TILT,
        .mask = 0U,
        .instance_ptr = NULL,
        .handler = Callback_PC_Pan_Tilt_Handler,
    };

    fdcan_bsp_register(&disp_pos1, &hfdcan3);
    fdcan_bsp_register(&disp_pos2, &hfdcan3);
    fdcan_bsp_register(&disp_pos3, &hfdcan3);
    fdcan_bsp_register(&disp_pc_target, &hfdcan3);
    fdcan_bsp_register(&disp_pc_pan_tilt, &hfdcan3);
}
