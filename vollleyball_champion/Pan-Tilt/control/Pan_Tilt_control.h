#ifndef CONTROL_H
#define CONTROL_H

void gimbal_set_angle(float yaw_deg, float pitch_deg);
void gimbal_control_init(void);
void gimbal_control_loop(void);



#endif 
