#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

#define CHASSIS_RADIUS 0.459f

/* Main controller -> steering board command frame (Classic CAN, Std ID, 8 bytes) */
#define CHASSIS_CMD_CAN_ID    0x11U
#define CHASSIS_CMD_VEL_SCALE 1000.0f

void Chassis_Init(void);
void Chassis_Update(float vx, float vy, float vr);
void Chassis_Stop(void);

#endif // CHASSIS_TASK_H
