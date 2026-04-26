#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include "main.h"

#define CAN_ID_POSE_PART1        0x0AAU
#define CAN_ID_POSE_PART2        0x0ABU
#define CAN_ID_POSE_PART3        0x0ACU
#define CAN_ID_PC_SET_TARGET     0x100U
#define CAN_ID_PC_PAN_TILT       0x200U
#define CAN_ID_PC_FEEDBACK       0x300U

typedef struct {
    float x;
    float y;
    float angle;
    float vx;
    float vy;
    float vr;
    uint32_t update_tick;
} Robot_Pose_t;

typedef struct {
    float target_x;
    float target_y;
    uint8_t is_updated;
} Robot_Target_t;

extern Robot_Pose_t g_robot_pose;
extern Robot_Target_t g_robot_target;
extern volatile uint32_t g_pan_tilt_cmd_update_tick;

void Robot_Data_Init(void);
void Robot_Data_Register_Dispatches(void);

#endif // ROBOT_DATA_H
