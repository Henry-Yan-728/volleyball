#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

#define CHASSIS_RADIUS 0.449f

#define CHASSIS_WHEEL_1_CAN_ID 0x11U
#define CHASSIS_WHEEL_2_CAN_ID 0x12U
#define CHASSIS_WHEEL_3_CAN_ID 0x13U

void Chassis_Init(void);
void Chassis_Update(float vx, float vy, float vr);
void Chassis_Stop(void);

#endif // CHASSIS_TASK_H
