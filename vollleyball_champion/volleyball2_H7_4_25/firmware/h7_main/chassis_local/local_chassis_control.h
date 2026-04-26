#ifndef LOCAL_CHASSIS_CONTROL_H
#define LOCAL_CHASSIS_CONTROL_H

#include "main.h"

#define LOCAL_STEERING_WHEEL_COUNT 3U

void LocalChassisControl_Init(void);
void LocalChassisControl_SetWheel(uint8_t wheel_index, float target_speed_ms, float target_angle_deg);
void LocalChassisControl_AutoCenterWheel(uint8_t wheel_index, float center_angle_deg);

#endif /* LOCAL_CHASSIS_CONTROL_H */
