#include "robot_data.h"

#include <string.h>

#include "fdcan_bsp.h"
#include "mechanism_task.h"

extern FDCAN_HandleTypeDef hfdcan3;

Robot_Pose_t g_robot_pose;
Robot_Target_t g_robot_target;

static Robot_Pose_t s_robot_pose_staging;

static void Callback_Pose_Part1_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&s_robot_pose_staging.x, &rx_data[0], 4);
        memcpy(&s_robot_pose_staging.y, &rx_data[4], 4);
    }
}

static void Callback_Pose_Part2_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&s_robot_pose_staging.angle, &rx_data[0], 4);
        memcpy(&s_robot_pose_staging.vx, &rx_data[4], 4);
    }
}

static void Callback_Pose_Part3_Handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    (void)instance_ptr;

    if (rx_header->DataLength >= FDCAN_DLC_BYTES_8)
    {
        memcpy(&s_robot_pose_staging.vy, &rx_data[0], 4);
        memcpy(&s_robot_pose_staging.vr, &rx_data[4], 4);
        s_robot_pose_staging.update_tick = HAL_GetTick();
        g_robot_pose = s_robot_pose_staging;
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
}

void Robot_Data_Init(void)
{
    memset(&g_robot_pose, 0, sizeof(g_robot_pose));
    memset(&s_robot_pose_staging, 0, sizeof(s_robot_pose_staging));
    memset(&g_robot_target, 0, sizeof(g_robot_target));
}

void Robot_Data_GetPoseSnapshot(Robot_Pose_t *pose_out)
{
    uint32_t primask;

    if (pose_out == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *pose_out = g_robot_pose;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Robot_Data_GetTargetSnapshot(Robot_Target_t *target_out)
{
    uint32_t primask;

    if (target_out == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *target_out = g_robot_target;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Robot_Data_SetTarget(float target_x, float target_y)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    g_robot_target.target_x = target_x;
    g_robot_target.target_y = target_y;
    g_robot_target.is_updated = 1U;
    if (primask == 0U) {
        __enable_irq();
    }
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
