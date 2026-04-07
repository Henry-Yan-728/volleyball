#ifndef MOTOR_RAMP_H
#define MOTOR_RAMP_H

#include <stdint.h>
#include "main.h"

typedef struct {
    int32_t target_val;   // Target speed command
    int32_t current_val;  // Smoothed speed output
    int32_t accel_step;   // Max accelerate step per loop
    int32_t decel_step;   // Max decelerate step per loop
    int32_t jerk_step;    // Max step change per loop
    int32_t current_step; // Current step applied this loop
} RampController_t;

void Motor_Ramp_Init(RampController_t *ramp, int32_t accel, int32_t decel);
void Motor_Ramp_SetJerk(RampController_t *ramp, int32_t jerk);
void Motor_Ramp_SetTarget(RampController_t *ramp, int32_t target);
int32_t Motor_Ramp_Calc(RampController_t *ramp);
void Motor_Ramp_EStop(RampController_t *ramp);

#endif /* MOTOR_RAMP_H */
