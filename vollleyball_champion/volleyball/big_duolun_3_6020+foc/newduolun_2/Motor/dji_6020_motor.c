#include "dji_6020_motor.h"
#include "fdcan.h"
#include "pid.h"
#include "basic.h"
#include "wheel_control.h"

/**************内部宏定义与重命名begin**************/

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
static int set_6020_spd_s[1]={0};
int init_flag=0;
static int set_6020_loc_s[1]={1749};
static int set_6020_mode_s[1]={LOC_MODE};
static motor_measure_t motor_6020_inf[1]={0};/*6020电机参数*/
static void Get_6020_total_angle(motor_measure_t *p);
static void Can_dji_6020_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor);
/**************内部变量与函数end**************/


/**************外部接口begin**************/
void Dji_6020_first_four_motor_control(int i,uint8_t rx_data[8]);
void Dji_6020_last_four_motor_control(int i,uint8_t rx_data[8]);
void Change_6020_speed(int motor_id,int target_spd);
void Change_6020_loc(int motor_id,int target_loc);
motor_measure_t Get_6020_information(int motor_id);

/**************外部接口end**************/

/*
1.函数功能：请求dji电机信息，包括速度、位置等
2.入参：电机ID
3.返回值：电机信息结构体
4.用法及调用要求：
5.其它：
*/
motor_measure_t Get_6020_information(int motor_id){
	return motor_6020_inf[motor_id];
}
/*
1.函数功能：设定dji电机的速度大小
2.入参：电机ID，speed（RPM）
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_6020_speed(int motor_id,int target_spd){
	set_6020_spd_s[motor_id]=target_spd;
}
/*
1.函数功能：设定dji电机的位置
2.入参：电机ID，整数值，编码器相关
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_6020_loc(int motor_id,int target_loc){
	set_6020_loc_s[motor_id]=target_loc;
}
/*
1.函数功能：得到dji电调的电机信息返回值
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：宏函数
*/
#define Get_6020_motor_measure(ptr, data)                                    \
    {                                                                  \
        (ptr)->last_angle = (ptr)->angle;                                   \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#ifdef ALL_Send_Flag
		int can1_send_flag[8]={0};//收到所有电机报文后发送一条报文的标志
#endif
/*
1.函数功能：得到dji电机的总转程
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：
*/
static void Get_6020_total_angle(motor_measure_t *p){
		int res1, res2, delta;
		if(p->angle < p->last_angle){			//可能的情况
			res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
			res2 = p->angle - p->last_angle;				//反转	delta=-
		}else{	//angle > last
			res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
			res2 = p->angle - p->last_angle;				//正转	delta +
		}
		//不管正反转，肯定是转的角度小的那个是真的
		if(Basic_int_abs(res1)<Basic_int_abs(res2))
			delta = res1;
		else
			delta = res2;

		p->total_angle += delta;
		p->last_angle = p->angle;
}
/*
1.函数功能：发送四个dji电机的电流值
2.入参：
3.返回值：无
4.用法及调用要求：注意修改发送通道为CAN2或CAN3
5.其它：通过FDCAN通信的6020电机
*/
static void Can_dji_6020_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor)
{
	static FDCAN_TxHeaderTypeDef  motor_tx_message;
	static uint8_t              motor_can_send_data[8];
    uint32_t send_mail_box;
	motor_tx_message.Identifier			  = all_response_id;
	motor_tx_message.IdType			  = FDCAN_STANDARD_ID ;
	motor_tx_message.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
motor_tx_message.DataLength 			= 0X08;			
    motor_can_send_data[0] = motor >> 8;
    motor_can_send_data[1] = motor;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_tx_message, motor_can_send_data);


}
/*
1.函数功能：pid计算出电机此次应当输出的电流值并发送
2.入参：报文的header，报文数据域内容
3.返回值：无
4.用法及调用要求：在dji电机的CAN的接收回调里调用此函数
5.其它：
*/

void Dji_6020_motor_control(FDCAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8]){        
				static uint8_t i = 0;           
				i = rx_header.Identifier	 - CAN_6020_M1_ID;
				Get_6020_motor_measure(&motor_6020_inf[i], rx_data);//得到电机信息
				if (motor_6020_inf[i].first == 0){
						motor_6020_inf[i].first = 1;
						motor_6020_inf[i].last_angle=motor_6020_inf[i].angle;
				}
				Get_6020_total_angle(&motor_6020_inf[i]);
				if(set_6020_mode_s[i]==LOC_MODE){

						Pid_incremental_cal(&motor_6020_pid_g[i].loc, -final_delta_angle*8192.0/360.0 ,0);//利用电机反馈的速度和位置信息做pid计算
						Pid_incremental_cal(&motor_6020_pid_g[i].spd,motor_6020_inf[i].speed_rpm,motor_6020_pid_g[i].loc.now_out);
					
				}
				else if(set_6020_mode_s[i]==SPEED_MODE){
					Pid_incremental_cal(&motor_6020_pid_g[i].spd,motor_6020_inf[i].speed_rpm,set_6020_spd_s[i]);
				}
					Can_dji_6020_motor_send(&hfdcan1,CAN_6020_MOTOR_ALL_ID,(int16_t)motor_6020_pid_g[0].spd.now_out);
						  //将PID的计算结果通过 CAN1 发到电机



}

