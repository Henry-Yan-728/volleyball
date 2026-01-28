#include "wheel_control.h"
#include "vesc.h"
#include "dji_6020_motor.h"
#include "can_database.h"
#include "rc_foc_motor.h"

float vel=0;//航向速度
float final_delta_angle=0;//控制舵向方向
float target_angle_pos=0;
float now_angle_pos=0;
int direction=1;
int get_setloc_flag_s=1;


elem_type get_abs(elem_type x){
	x=(x>=0)?x:-x;
	return x;
}


elem_type get_delta_angle(float target_angle,float now_angle,int* direction){//输入为角度制下的角度
	elem_type delta_angle=0;//返回的角度差值
	
	delta_angle=get_abs(target_angle-now_angle);
	if(delta_angle<=90){
		*direction=1;
		return target_angle-now_angle;
	}
	else if (get_abs(target_angle - now_angle - 360) <= 90)
	{
		*direction = 1;
		return target_angle - now_angle - 360;
	}
	else if(get_abs(target_angle -now_angle +360) <= 90){
		*direction =1;
		return target_angle - now_angle + 360;
	}
	delta_angle=target_angle-now_angle+180;
	if(delta_angle>90) delta_angle-=360;
	*direction=-1;
	return delta_angle;
}


void control()
{
	now_angle_pos = (Get_6020_information(0).angle-675)/8192.0*360.0;
	int temp=now_angle_pos/360;
	now_angle_pos=now_angle_pos-360*temp;
	if(now_angle_pos>180){
		now_angle_pos-=360;
	}else if(now_angle_pos<=-180){
		now_angle_pos+=360;
	}
	final_delta_angle=get_delta_angle(target_angle_pos,now_angle_pos,&direction);
	set_blazer_speed(0x03,vel*direction);

//	Change_vesc_speed(1,vel*direction);
//	Com2vesc(1); 
}

