#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "../../../shared/chassis_protocol.h"

#define CHASSIS_RADIUS 0.459f
#define CHASSIS_HOST_LINEAR_MM_PER_M     1000.0f
#define CHASSIS_TX_DROP_STREAK_LIMIT     5U

/*
 * Stage-3 final fusion switch:
 *   1: H7 directly runs the small-duolun chassis pipeline.
 *   0: Keep the phase-2 H7 -> G4 0x11 command link.
 */
#ifndef CHASSIS_LOCAL_CONTROL_ENABLE
#define CHASSIS_LOCAL_CONTROL_ENABLE      1U
#endif

#define CHASSIS_STEERING_OFFSET_WHEEL_0_DEG  0.0f
#define CHASSIS_STEERING_OFFSET_WHEEL_1_DEG -60.0f
#define CHASSIS_STEERING_OFFSET_WHEEL_2_DEG  60.0f
#define CHASSIS_CMD_TIMEOUT_MS               100U
#define CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD 0.02f
#define CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD 0.02f
#define CHASSIS_AUTO_CENTER_ANGLE_DEG        0.0f
#define CHASSIS_SPIN_MIN_WHEEL_SPEED         0.06f
#define CHASSIS_MAX_WHEEL_SPEED_MS           3.0f

/* Main controller -> steering board command frame (Classic CAN, Std ID, 8 bytes) */
#define CHASSIS_CMD_CAN_ID    CHASSIS_PROTOCOL_CAN_ID
#define CHASSIS_CMD_VEL_SCALE CHASSIS_PROTOCOL_LINEAR_SCALE

/*
 * H7 upper layers still generate linear speed in mm/s.
 * This module converts to the shared wire protocol's m/s contract.
 * Angular speed is already expressed in rad/s.
 */

void Chassis_Init(void);
void Chassis_Update(float vx, float vy, float vr);
void Chassis_Stop(void);
void Chassis_Task_Loop(void);
void Chassis_SafetyTick(void);

#endif // CHASSIS_TASK_H
