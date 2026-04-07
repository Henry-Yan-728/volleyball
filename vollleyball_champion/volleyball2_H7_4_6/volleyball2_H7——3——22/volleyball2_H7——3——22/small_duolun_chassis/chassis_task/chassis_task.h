#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

#define CHASSIS_RADIUS 0.459f

/* Main controller -> steering board command frame (Classic CAN, Std ID, 8 bytes) */
#define CHASSIS_CMD_CAN_ID      0x11U
#define CHASSIS_CMD_VEL_SCALE   10000.0f
#define CHASSIS_CMD_TIMEOUT_MS  1000

/* Auto-center thresholds when command is near stop */
#define CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD   0.02f
#define CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD  0.02f
#define CHASSIS_AUTO_CENTER_ANGLE_DEG          0.0f

/* Keep pure spin command from falling into wheel speed deadband */
#define CHASSIS_SPIN_MIN_WHEEL_SPEED           0.06f

extern volatile float vx;
extern volatile float vy;
extern volatile float vr;

void Chassis_Init(void);
void Chassis_Update(float vx, float vy, float vr);
void Chassis_Stop(void);
void Chassis_Task_Loop(void);

#endif // CHASSIS_TASK_H
