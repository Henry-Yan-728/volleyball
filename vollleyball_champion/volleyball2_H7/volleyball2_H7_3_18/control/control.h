#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

/* Steering: 2006 (36:1) + external 3:1 transmission -> 108:1 overall */
#define STEERING_GEAR_RATIO  108.0f

/* Drive wheel / VESC parameters */
#define WHEEL_RADIUS         0.023f
#define DRIVE_GEAR_RATIO     1.0f
#define VESC_POLE_PAIRS      12

/* Ignore tiny speed command to avoid wheel jitter */
#define SPEED_DEADBAND       0.05f

#define STEERING_WHEEL_COUNT 3U

void Steering_Wheel_Control(uint8_t wheel_index, float target_speed, float target_angle_deg);
void Steering_Wheel_AutoCenter(uint8_t wheel_index, float center_angle_deg);

#endif /* CONTROL_H */
