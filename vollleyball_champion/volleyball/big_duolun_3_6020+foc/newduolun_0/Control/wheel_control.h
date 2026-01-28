#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

extern float vel;
extern float final_delta_angle;
extern float target_angle_pos;
extern int get_setloc_flag_s;
extern float now_angle_pos;
typedef float elem_type;


elem_type get_delta_angle(float target_angle,float now_angle,int* direction);
void control();

#endif
