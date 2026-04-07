/**
  ******************************************************************************
  * @file    : chassis_cybergear.h
  * @brief   : Xiaomi CyberGear chassis control interface
  ******************************************************************************
  * Usage:
  * 1. Call `chassis_cybergear_init()` once after scheduler and CAN are ready.
  * 2. Init only does safe stop + read current angle + set current as zero (comm type 6).
  * 3. No mode/enable is applied during boot.
  * 4. First control command will lazily switch mode and enable motor.
  ******************************************************************************
  */

#ifndef CHASSIS_CYBERGEAR_H
#define CHASSIS_CYBERGEAR_H

#include "cybergear_motor.h"

int8_t chassis_cybergear_init(uint8_t motor_index);
void chassis_control(float angle_deg);
void chassis_control_speed(float speed_rad_s);
void chassis_cybergear_stop(void);
float chassis_get_angle_rad(void);

#endif // CHASSIS_CYBERGEAR_H
