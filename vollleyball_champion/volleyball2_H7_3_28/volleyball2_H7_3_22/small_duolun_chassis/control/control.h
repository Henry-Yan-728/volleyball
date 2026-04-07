#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

#define STEERING_GEAR_RATIO   108

/* Wheel diameter is 46 mm -> radius = 0.023 m */
#define WHEEL_RADIUS          0.023f
#define DRIVE_GEAR_RATIO      1.0f

#define SPEED_DEADBAND        0.05f
#define STEERING_WHEEL_COUNT  3

/*
 * Ramp is applied on speed command in m/s before motor RPM conversion.
 * With SCALE=1000:
 * step=1 means 0.001 m/s each control period.
 */
/* 1 ms loop: larger accel_step => harder launch; smaller decel_step => softer braking */
#define DRIVE_RAMP_ACCEL_STEP 70
#define DRIVE_RAMP_DECEL_STEP 3
#define DRIVE_RAMP_JERK_STEP  2

void control_init(void);

void Steering_Wheel_Control(uint8_t wheel_index, float target_speed, float target_angle_deg);
void Steering_Wheel_AutoCenter(uint8_t wheel_index, float center_angle_deg);

#endif // CONTROL_H
