#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

#define CHASSIS_RADIUS 0.459f

/* Keep main controller command ID unchanged for compatibility */
#define CHASSIS_CMD_CAN_ID    0x11U
#define CHASSIS_CMD_VEL_SCALE 1000.0f

#define CHASSIS_STEERING_WHEEL_COUNT 3U

/* Steering motor instance mapping in dji_motor.c */
#define CHASSIS_STEER_MOTOR_IDX_W1 0U
#define CHASSIS_STEER_MOTOR_IDX_W2 1U
#define CHASSIS_STEER_MOTOR_IDX_W3 2U

/* VESC controller ID mapping (can be adjusted to avoid conflicts) */
#define CHASSIS_DRIVE_VESC_ID_W1   0U
#define CHASSIS_DRIVE_VESC_ID_W2   1U
#define CHASSIS_DRIVE_VESC_ID_W3   2U

void Chassis_Init(void);
void Chassis_Update(float vx, float vy, float vr);
void Chassis_Stop(void);

#endif // CHASSIS_TASK_H
