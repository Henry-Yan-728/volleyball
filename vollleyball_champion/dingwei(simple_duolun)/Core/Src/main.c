/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stm32g4xx_hal_fdcan.h"
#include "jiguang_relocate.h"
//#define floatingtest
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define dis1_0 1
#define dis2_0 1
#define dis3_0 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define jiaozhun

struct GyroResult gyroResult;
struct EncoderResult encoderRes;
struct LocatorResult lcResult;
struct GyroResult timerGyroResult;
struct EncoderResult timerEncResult;
struct LocatorResult timerlcResult;
struct EncoderResult lastEncResult;
struct GyroResult lastGyroResult;
int32_t QEI_pulse[2] = {0};
double vx_dis=0,vy_dis=0,vr_ang=0;
double total_x[5]={0},total_y[5]={0},total_r[5]={0};
int v_n=0;
int send_times=0;
uint8_t restart_flag=0;
int gyroloop=0;
int cnt1=0;
#ifdef jiaozhun
	double floating=0;
	double floating_total=0;
	int cnt=0;
	int cnt_p=0;  //正转计数
	int cnt_n=0;  //反转计数
	int pulse_out=0;
	int jiaozhun_flag=0;
	float ang_total=0;
	int start_flag_jiaozhun=1;
	float gyro_angle_jiaozhun=0;
	float last_angle_jiaozhun=0;
	float gyro_angle_jiaozhun_start=0;
	float del_angle_jiaozhun=0;
	float p_del_angle_jiaozhun_total=0;
	float n_del_angle_jiaozhun_total=0;
	int pulse1_jiaozhun=0;
	int pulse0_jiaozhun=0;
	int pulse1_jiaozhun_start=0;
	int pulse0_jiaozhun_start=0;
	int gyro_start_time_jiaozhun=0;
	int last_out_time=0;
	int now_out_time=0;
	int pulse0_jiaozhun_last=0;
	int pulse1_jiaozhun_last=0;
	#define floating_numbers 10000
#endif
#ifndef jiaozhun
	#define p_rotation_e0 (-11347.224609)
	#define p_rotation_e1 (4312.856445)
	#define n_rotation_e0 (-11465.922852)
	#define n_rotation_e1 (4142.797852)
//	#define p_rotation_e0 (3570.952148)
//	#define p_rotation_e1 (3370.002686)
//	#define n_rotation_e0 (3467.672852)
//	#define n_rotation_e1 (3509.345703)

/*
系数矩阵:
y     e0 e1    0.0139    0.0137
x			     		-0.0137    0.0138
*/
	#define I_mul_x0 (-0.01386101253)
	#define I_mul_x1 (-0.0135421252)
	#define I_mul_y0 (-0.01367515292)
	#define I_mul_y1 ( 0.0139137016)
//	#define I_mul_x0 (-0.01407752897)
//	#define I_mul_x1 ( -0.01417931264)
//	#define I_mul_y0 ( -0.014361242)
//	#define I_mul_y1 (0.01423372233)
#endif

#ifdef jiaozhun
	int p_rotation_e0_total=0;
	int p_rotation_e1_total=0;
	int n_rotation_e0_total=0;
	int n_rotation_e1_total=0;
	
	float p_rotation_e0=0;
	float p_rotation_e1=0;
	float n_rotation_e0=0;
	float n_rotation_e1=0;

	float I_mul_x0=0;
	float I_mul_x1=0;
	float I_mul_y0=0;
	float I_mul_y1=0;
#endif
//重定向输出   uart3做输入输出交互
void USART_printf(char *fmt, ...)
{
	va_list ap;
	char str[128];

	va_start(ap,fmt);
	vsprintf(str,fmt,ap);
	va_end(ap);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str),0xFFFF);
}



//耦合时用到的一些参数
int data_state=0;   	//获取数据状态  0表示挂起  1表示执行
int cal_state=0;		//计算状态		0表示挂起  1表示执行
int gyro_update=0;
int encoder_updata=0;
int gyro_start=0;
int if_gyro_err=0;

volatile float del_encoder0=0;
volatile float del_encoder1=0;


//Pc交互
uint8_t Pc_buffer[2]={0,0};

//陀螺仪数据获取以及姿态解算的一些函数和变量
/**********************************************************************************************************************************/
volatile uint8_t gyro_buffer[33];
uint8_t if_gyro_right=0;
//陀螺仪角度值；
float gyro_angle=0;
float q_angle[4]={1,0,0,0};     //旋转四元数
float Gyro_v[3]={0,0,0};        //分别对应绕x y z旋转的角速度
float Gyro_a[3]={0,0,0};        //分别对应沿x轴 y轴 z轴方向的加速度
float gyro_dealt_t=0;	        //用于存间隔时间
float gyro_start_time=0;		//用于记录初始时间，解决零漂
float gyrotime=0;	        	//记录陀螺仪时间戳
float gyro_lasttime=0;			//记录上次获取陀螺仪的时间戳
int if_gyro_start_first=1;		//初始时间用到的标志位
uint32_t gyro_vel[3]={0,0,0};   //用于接收角速度数据
uint32_t gyro_a[3]={0,0,0};     //用于接收轴向加速度数据
float accex=0;					//用于存储误差积分值 
float accey=0;
float accez=0;


//获取陀螺仪数据，三个角速度以及三个轴向加速度
void get_gyro_data()
{
	gyro_vel[0]=(gyro_buffer[5]<<24) | (gyro_buffer[4]<<16)|(gyro_buffer[3]<<8)|gyro_buffer[2];
	gyro_vel[1]=(gyro_buffer[9]<<24) | (gyro_buffer[8]<<16)|(gyro_buffer[7]<<8)|gyro_buffer[6];
	gyro_vel[2]=(gyro_buffer[13]<<24) | (gyro_buffer[12]<<16)|(gyro_buffer[11]<<8)|gyro_buffer[10];
	gyro_a[0]=(gyro_buffer[17]<<24) | (gyro_buffer[16]<<16)|(gyro_buffer[15]<<8)|gyro_buffer[14];
	gyro_a[1]=(gyro_buffer[21]<<24) | (gyro_buffer[20]<<16)|(gyro_buffer[19]<<8)|gyro_buffer[18];
	gyro_a[2]=(gyro_buffer[25]<<24) | (gyro_buffer[24]<<16)|(gyro_buffer[23]<<8)|gyro_buffer[22];
	for(int i=0;i<3;i++)
	{
		Gyro_v[i]=(*((float *)(&gyro_vel[i])))*0.01745329252;  //转化为弧度
		Gyro_a[i]=*((float *)(&gyro_a[i]));
	}
	Gyro_a[0]*=-1;
	Gyro_a[2]*=-1;
	Gyro_v[0]*=-1;
	Gyro_v[2]*=-1;
	 if(Gyro_v[2]>4.98||Gyro_v[2]<-4.98)
	{
		if_gyro_err=1;
		//#ifdef jiaozhun
		USART_printf("err  v=%.5f\n",Gyro_v[2]);
		//#endif		
	}
}


//求平方根倒数函数
static float invSqrt(float x)
{
	float halfx=0.5f*x;
	float y=x;
	long  i=*(long*)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-(halfx*y*y));
	return y;
}


//利用加速度进行陀螺仪误差修正    输入时间差；
void gyro_err_clear(float delat_t )
{
	//加速度归一化

	float xishu=invSqrt(Gyro_a[0]*Gyro_a[0]+Gyro_a[1]*Gyro_a[1]+Gyro_a[2]*Gyro_a[2]);
	Gyro_a[0]*=xishu;
	Gyro_a[1]*=xishu;
	Gyro_a[2]*=xishu;
	//提取姿态矩阵中的重力分量
	float Vx=2*(q_angle[1]*q_angle[3]-q_angle[0]*q_angle[2]);
	float Vy=2*(q_angle[1]*q_angle[0]+q_angle[3]*q_angle[2]);
	float Vz=1-2*(q_angle[1]*q_angle[1]+q_angle[2]*q_angle[2]);
	//USART_printf(" q0=%f\n",q_angle[0]);
	//求出姿态误差
	float ex=Gyro_a[1]*Vz-Gyro_a[2]*Vy;
	float ey=Gyro_a[2]*Vx-Gyro_a[0]*Vz;
	float ez=Gyro_a[0]*Vy-Gyro_a[1]*Vx;
	//USART_printf(" %f\n",ez);
	//误差积分
	float ki=0.001;
	float kp=0.01;

	accex+=ex*ki*delat_t;
	accey+=ey*ki*delat_t;
	accez+=ez*ki*delat_t;
	
	//角速度修正
	Gyro_v[0]+=kp*ex+accex;
	Gyro_v[1]+=kp*ey+accey;
	Gyro_v[2]+=kp*ez+accez;
	//USART_printf(" v2=%f\n",Gyro_v[2]);
	
}

//四元数数据更新，输入间隔时间；
void q_angle_update(float delat_t ) 
{
	q_angle[0]=q_angle[0]+0.5*delat_t*(-1*Gyro_v[0]*q_angle[1]-Gyro_v[1]*q_angle[2]-Gyro_v[2]*q_angle[3]);
	q_angle[1]=q_angle[1]+0.5*delat_t*( 1*Gyro_v[0]*q_angle[0]-Gyro_v[1]*q_angle[3]+Gyro_v[2]*q_angle[2]);
	q_angle[2]=q_angle[2]+0.5*delat_t*( 1*Gyro_v[0]*q_angle[3]+Gyro_v[1]*q_angle[0]-Gyro_v[2]*q_angle[1]);
	q_angle[3]=q_angle[3]+0.5*delat_t*(-1*Gyro_v[0]*q_angle[2]+Gyro_v[1]*q_angle[1]+Gyro_v[2]*q_angle[0]);
	
	//四元数归一化
	float xishu=invSqrt(q_angle[0]*q_angle[0]+q_angle[1]*q_angle[1]+q_angle[2]*q_angle[2]+q_angle[3]*q_angle[3]);
	for(int i=0;i<=3;i++)
	{
		q_angle[i]*=xishu;
	}
}

//陀螺仪角度的反解
float get_gyro_angle(int i)
{
	float g1=0,g2=0,g3=0,g4=0,g5=0;
  float angle[3]={0};
 
	g1=2*(q_angle[1]*q_angle[3]-q_angle[0]*q_angle[2]);
	g2=2*(q_angle[1]*q_angle[0]+q_angle[3]*q_angle[2]);
	g3=   q_angle[0]*q_angle[0]-q_angle[1]*q_angle[1]-q_angle[2]*q_angle[2]+q_angle[3]*q_angle[3];
	g4=2*(q_angle[1]*q_angle[2]+q_angle[3]*q_angle[0]);
	g5=   q_angle[0]*q_angle[0]+q_angle[1]*q_angle[1]-q_angle[2]*q_angle[2]-q_angle[3]*q_angle[3];
	//角度值分别对应俯仰角、翻滚角、偏航角；
	angle[0]=-1*asinf(g1);
	angle[1]=atanf(g2/g3);
	angle[2]=atan2f(g4,g5);
	//USART_printf("g5=%f\n",g4/g5);
	return angle[i];
}
//陀螺仪数据更新 ，包含角度值和时间戳的更新
void gyrodata_update()
{
	//USART_printf("get");
	get_gyro_data();
	gyrotime=HAL_GetTick();
	if(if_gyro_start_first)  //获取初始时刻
	{
		gyro_start_time=gyrotime;
		if_gyro_start_first=0;
	}
	gyro_dealt_t=(gyrotime-gyro_lasttime)*0.001;
	gyro_err_clear(gyro_dealt_t);
	q_angle_update(gyro_dealt_t);
	gyro_angle=get_gyro_angle(2);      
	//gyro_angle-=0.000617*(gyrotime-gyro_start_time)*0.001; //0.000832 //0.000154
	gyro_angle-=-0.000071*(gyrotime-gyro_start_time)*0.001; 
	//USART_printf("angz %f \n",gyro_angle);
	//Gyroz=Gyroz+0.2056-(gyrotime-gyro_lasttime)*0.001*0.0031;
 	if (gyro_angle >= pi)
	{
			gyroloop++;
			gyro_angle -= 2*pi;
	}
	else if (gyro_angle <= -pi)
	{
			gyroloop--;
			gyro_angle += 2*pi;
	}
	gyro_lasttime=gyrotime;
	if_gyro_right=0;	
	//y = -0.0434x + 0.0031  角度制下零漂	
}

/**********************************************************************************************************************************/

//自动装载次数，正转溢出+1  反转溢出-1；
volatile int32_t ENCODER_overflow_cnt[2] = {-1,-1};   
TIM_TypeDef* ENCODER_TIM[2]={TIM1, TIM2};


//计算总脉冲值
int32_t Get_Encoder_Pulse_Count(int i)
{
	return (ENCODER_overflow_cnt[i] * (1 + (ENCODER_TIM[i]->ARR)) + ENCODER_TIM[i]->CNT);    //得到总脉冲数
}	

float pos_x=0;
float pos_y=0;
float sum_del_encoder0=0;
float sum_del_encoder1=0;
float sum_del_angle=0;

//void process_locator_data()
//{
//	static double last_encoder_0=0;
//	static double last_encoder_1=0;
//	static float last_angle=0;
//	
//	float del_angle=gyro_angle-last_angle;
//	if(del_angle>pi||del_angle<-pi)
//	{
//		if(last_angle<0)            // del_angle > pi  朝着角度减小的方向旋转一个小于pi的角度
//		{
//			last_angle+=2*pi;
//		}
//		else                        //del_angle<-pi    朝着角度增加的方向旋转一个小于pi的角度
//		{
//			last_angle-=2*pi;
//		}
//	}
//	del_angle=gyro_angle-last_angle;     //得到一个正负号和规定角度正负号同相的    小于pi的角度
//	float now_encoder_0=Get_Encoder_Pulse_Count(0);
//	float now_encoder_1=Get_Encoder_Pulse_Count(1);
//	if(del_angle<0.000000001&&del_angle>-0.000000001) 
//	{
//		del_encoder0+=(now_encoder_0-last_encoder_0);
//		del_encoder1+=(now_encoder_1-last_encoder_1);
//		last_encoder_0=now_encoder_0;
//		last_encoder_1=now_encoder_1;
//		
//		data_state=0;  
//		return;
//	}
//	else
//	{
//		sum_del_angle+=del_angle;            //总的角度误差    以及   -pi到pi的归化    

//		if(sum_del_angle>pi)	
//		{
//			sum_del_angle-=2*pi;
//		}
//		if(sum_del_angle<-pi)
//		{
//			sum_del_angle+=2*pi;
//		}
//		if(del_angle>0)
//		{
//			 del_encoder0+=(now_encoder_0-last_encoder_0-p_rotation_e0*del_angle);
//			 del_encoder1+=(now_encoder_1-last_encoder_1-p_rotation_e1*del_angle);
//		}
//		else
//		{
//			del_encoder0+=(now_encoder_0-last_encoder_0-n_rotation_e0*del_angle);
//			del_encoder1+=(now_encoder_1-last_encoder_1-n_rotation_e1*del_angle);
//		}
//		//now_encoder_0、1  计算中不能被修改
//		last_encoder_0=now_encoder_0;
//		last_encoder_1=now_encoder_1;
//		last_angle=gyro_angle;
//	}
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
//	USART_printf("get");
	if(huart->Instance == USART1)   
	{
		//USART_printf("ok");
		if(!if_gyro_right)
		{
//			USART_printf("get3");
			if(gyro_buffer[0]==0xBD)
			{
				if_gyro_right=1;
				//USART_printf("get3");
			}
		}
		else
		{
	//		USART_printf("get2");
			if(!(gyro_buffer[0]==0xDB&&gyro_buffer[1]==0x0A))
			{
				//USART_printf("get2");
				if_gyro_right=0; 
				return;
			}
			else
			{
				//USART_printf("wait cal\n");
				data_state=1;
				if(cal_state==1)
				{
					//USART_printf("wait cal\n");
					data_state=0;
					return;
				}
				//USART_printf("gggg");
				gyro_update=1;
				gyrodata_update();
				gyroResult.rotation=gyro_angle;
				gyroResult.loop=gyroloop;
				gyroResult.timeStamp = GenerateTimeStamp();
				double pul0 = Get_Encoder_Pulse_Count(0);
				double pul1 = Get_Encoder_Pulse_Count(1);
				encoderRes.distance[0] += pul0-QEI_pulse[0];
				encoderRes.distance[1] += pul1-QEI_pulse[1];
				QEI_pulse[0] = pul0;
				QEI_pulse[1] = pul1;
				encoderRes.timeStamp = GenerateTimeStamp();
				cnt1++;				
				//process_locator_data();
			}
			data_state=0;
		}
		#ifndef jiaozhun
		return;
		#endif
	}
	if(huart->Instance == USART2)
	{

		#ifdef jiaozhun
		if(Pc_buffer[0]=='1'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=1;
			start_flag_jiaozhun=1;
			USART_printf("start floating jiaozhun!");
			
		}
		else if(Pc_buffer[0]=='2'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=2;
			start_flag_jiaozhun=1;
			USART_printf("start encoder jiaozhun!");
			pulse_out=1;
			
		}
		else if(Pc_buffer[0]=='2'&&Pc_buffer[1]=='e')
		{
			jiaozhun_flag=2;
			pulse_out=2;
			
		}
		else if(Pc_buffer[0]=='3'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=3;
			start_flag_jiaozhun=1;
			pulse_out=1;
			USART_printf("start locator jiaozhun!");
			
		}
		else if(Pc_buffer[0]=='3'&&Pc_buffer[1]=='e')
		{
			jiaozhun_flag=3;
			pulse_out=2;
			
		}
		else if(Pc_buffer[0]=='4'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=4;
			start_flag_jiaozhun=1;
			pulse_out=1;
			USART_printf("start locator jiaozhun!");
			
		}
		else if(Pc_buffer[0]=='4'&&Pc_buffer[1]=='e')
		{
			jiaozhun_flag=4;
			pulse_out=2;
			
		}			
		else{
			USART_printf("1err\n");
		}
		#endif
        HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
	}
}

//E0 TIM1 A8 A9
//E1 TIM2 A0 A1

void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//encoder  0   脉冲数
 	if(htim->Instance==TIM1)
	{
		
		//USART_printf("in\n");
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			--ENCODER_overflow_cnt[0];
			//USART_printf("11  %d\n",ENCODER_overflow_cnt[0]);
		}
		else
		{
			++ENCODER_overflow_cnt[0];
			//USART_printf("22  %d\n",ENCODER_overflow_cnt[0]);
		}
//		float pul0 = Get_Encoder_Pulse_Count(0);
//		float pul1 = Get_Encoder_Pulse_Count(1);
//		encoderRes.distance[0] += pul0-QEI_pulse[0];
//		encoderRes.distance[1] += pul1-QEI_pulse[1];
//    QEI_pulse[0] = pul0;
//		QEI_pulse[1] = pul1;
//		encoderRes.timeStamp = GenerateTimeStamp();
	}
	//encoder 1  脉冲数
	if(htim->Instance==TIM2)
	{
		//process_locator_data();
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
		{
			--ENCODER_overflow_cnt[1];
		}
		else
		{
			++ENCODER_overflow_cnt[1];
		}
//		float pul0 = Get_Encoder_Pulse_Count(0);
//		float pul1 = Get_Encoder_Pulse_Count(1);
//		encoderRes.distance[0] += pul0-QEI_pulse[0];
//		encoderRes.distance[1] += pul1-QEI_pulse[1];
//    QEI_pulse[0] = pul0;
//		QEI_pulse[1] = pul1;
//		encoderRes.timeStamp = GenerateTimeStamp();		
	}
}
/*************************************************************/
void LocatorUpdate()
{

	cal_state=1;   //计算坐标时  cal_state=1 now_state=0;
	if(data_state==1)
	{
		cal_state=0;
		//USART_printf("wait get\n");
		return;
	}
	
	if(gyro_update==0) //cal_state[1]=0;
	{         
		cal_state=0;
		//USART_printf("wait gyro\n");
		return;
	}
	else gyro_update=0; 
	
	lcResult.timeStamp = GenerateTimeStamp();
	double dx,dy,dr,sinr,cosr;
	double dz;
	struct GyroResult readGyro = gyroResult;
	struct EncoderResult readEnc = encoderRes;
  struct LocatorResult readlc = lcResult;
  //计算速度
	double deltaTime = TIMESTAMP2SECOND(readlc.timeStamp,timerlcResult.timeStamp);
	if(deltaTime>LOCATOR_MIN_TIMEGAP)
	{
		deltaTime*=5;
		total_x[v_n]=vx_dis;
		total_y[v_n]=vy_dis;
		total_r[v_n]=vr_ang;
		v_n=(v_n==4)?0:(v_n+1);
		for(int n=0;n<=4;n++)
		{
			lcResult.vx+=total_x[n];
			lcResult.vy+=total_y[n];
			lcResult.vAng+=total_r[n];
		}
		lcResult.vx=(total_x[0]+total_x[1]+total_x[2]+total_x[3]+total_x[4])/deltaTime;
		lcResult.vy=(total_y[0]+total_y[1]+total_y[2]+total_y[3]+total_y[4])/deltaTime;
		lcResult.vAng=(total_r[0]+total_r[1]+total_r[2]+total_r[3]+total_r[4])/deltaTime;
		vx_dis=0;
		vy_dis=0;
		vr_ang=0;
		timerlcResult = readlc;
	
	}	
	
	//dx dy dr
	//dr=readGyro.rotation-lastGyroResult.rotation;
	dr=(readGyro.loop*2*pi+readGyro.rotation)-(lastGyroResult.loop*2*pi+lastGyroResult.rotation);
	if (dr >= pi) dr -= 2*pi;
	else if (dr <= -pi) dr += 2*pi;

	//USART_printf("DR=%.6f",dr);
  lcResult.r += dr;	
	vr_ang+=dr;
	if (lcResult.r >= pi) lcResult.r -= 2*pi;
	else if (lcResult.r <= -pi) lcResult.r += 2*pi;
	
	dx = readEnc.distance[0]-lastEncResult.distance[0];
	dy = readEnc.distance[1]-lastEncResult.distance[1];
	
	if(dr>0)
	{
		 dx -= p_rotation_e0*dr;
		 dy -= p_rotation_e1*dr;
	}
	else
	{
		 dx -= n_rotation_e0*dr;
		 dy -= n_rotation_e1*dr;
	}

	dz = dx;//using as a buffer
	dx=I_mul_x0*dz+I_mul_x1*dy;
	dy=I_mul_y0*dz+I_mul_y1*dy;
	
	  //  USART_printf("dx %f   dy %f    r %f\n",dx,dy,lcResult.r);
	dz = dx;//using as a buffer
	sinr = sin(lcResult.r);
	cosr = cos(lcResult.r);
	dx = cosr*dz-sinr*dy;
	//r>0
	dy = sinr*dz+cosr*dy;
//	if(dx>0.1)
//	{
//	USART_printf("dx%f\n",dx);
//	}
	lcResult.x += dx;
	lcResult.y += dy;
	vx_dis+=dx;
	vy_dis+=dy;	
	lastEncResult = readEnc;
	lastGyroResult = readGyro;
	if(restart_flag==1)
	{
		//LocatorSystemInitialize();
		lcResult.x=0;
		lcResult.y=0;
		lcResult.r=0;
		restart_flag=0;
	}
	cal_state=0;
	
}
//float cal_time=0;
//float last_caltime=0;
////获取坐标  posx 和 posy
//void cal_loc()
//{
//	cal_state=1;   //计算坐标时  cal_state=1 now_state=0;
//	now_state=0;
//	if(data_state==1 &&now_state==0)
//	{
//		cal_state=0;
//		return;
//	}
//	
//	if(gyro_update==0) //cal_state[1]=0;
//	{         
//		cal_state=0;
//		return;
//	}
//	else gyro_update=0;           //update!=0 在此之后变为0
//	
//	cal_time=HAL_GetTick();       //获取当前计算时间
//	
//	//计算此时的dealt x、y
//	float delx=I_mul_x0*del_encoder0+I_mul_x1*del_encoder1;
//	float dely=I_mul_y0*del_encoder0+I_mul_y1*del_encoder1;
//	sum_del_encoder0+=del_encoder0;     
//	sum_del_encoder1+=del_encoder1;
//	del_encoder0=0;
//	del_encoder1=0;
//	
//	
//	
//	pos_x+=	cos(gyro_angle)*delx;   //把正交的两个码盘方向的dealt量 分解到定位坐标系
//	pos_x+=-sin(gyro_angle)*dely;
//	pos_y+=	cos(gyro_angle)*dely;
//	pos_y+=	sin(gyro_angle)*delx;
//	
//	last_caltime=cal_time;
//	cal_state=0;
//}

//计算此时角度到上次角度中较小的那个角
//float cal_sum_angle(void)
//{
//	static float last_angle=0;
//	static float sum_angle=0;
//	float angle=gyro_angle;
//	float delta=angle-last_angle;
//	if(delta>pi||delta<-pi)
//	{
//		if(last_angle<0)  last_angle+=2*pi;
//		else last_angle-=2*pi;
//		sum_angle+=(angle-last_angle);
//		last_angle=angle;

//	}
//	else
//	{
//		sum_angle+=delta;	
//		last_angle=angle;
//	}
//	return sum_angle;
//}
void LocatorSystemInitialize()
{
    gyroResult.timeStamp = 0; 
    gyroResult.rotation = 0;
	  encoderRes.distance[0]=0;
	  encoderRes.distance[1]=0;
	  lcResult.timeStamp=0;
    lcResult.x = 0;
    lcResult.y = 0;
    lcResult.r = 0;
	  lcResult.vx = 0;
    lcResult.vy = 0;
    lcResult.vAng = 0;
	  lastEncResult = encoderRes;
    lastGyroResult = gyroResult;
}
void send_lc(float x ,float y ,float r,float vx,float vy){
	static uint8_t data_byte[8];
	uint8_t datas='a';
//	if(if_gyro_err)
//	{
//		data_byte[0]=0xBD;
//		if_gyro_err=0;
//	}
//	else{

//	}
     int16_t tempx=(int16_t)x;
	uint8_t* tmp1 = (uint8_t*)&tempx;
	data_byte[0] = tmp1[0]; 
	data_byte[1] = tmp1[1];
    int16_t tempy=(int16_t)y;
	uint8_t* tmp2 = (uint8_t*)&tempy;
	data_byte[2] = tmp2[0]; 
	data_byte[3] = tmp2[1];                                                                 
	uint8_t* tmp3 = (uint8_t*)&r;
	data_byte[4] = tmp3[0]; 
	data_byte[5] = tmp3[1];
	data_byte[6] = tmp3[2];
	data_byte[7] = tmp3[3];

	
//创建FDCAN消息结构
    uint8_t data_byte1[8]={};
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0x123;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    // 发送数据
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data_byte) != HAL_OK)
    {
        // 错误处理
		USART_printf("NO, SB!\n");
        Error_Handler();
    }	
}
//	float x=0.01,y=0.01,r=0.01;
/*************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	  if(huart == &huart1)
    {
        __HAL_UNLOCK(huart);
        HAL_UART_Receive_IT(&huart1, (uint8_t*) gyro_buffer, 1);
    }
		else if(huart == &huart2)
		{
        __HAL_UNLOCK(huart);
        HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
		}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//可直插的串口为串口3   用于调试发送信息和对外发送定位信息    二者只能使用一个
//	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);	
//	HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) gyro_buffer, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
	
	//时间线定义  第一次获取时间
	int time0=HAL_GetTick();
	int time1=time0;
	int time2=time0;
	int time3=time0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//重新获取时间
		//gyro_cal();
		LocatorUpdate();
		time0=HAL_GetTick();
		//cal_loc();
		
		int32_t pulse0=Get_Encoder_Pulse_Count(0); 
		int32_t pulse1=Get_Encoder_Pulse_Count(1); 
		//不同处理时间间隔下进行操作
		if(time0-time1>=1000)
		{
			//对外发送信息时不能存在任何打印
	//USART_printf("position x%.6f y%.6f r%.6f \n",lcResult.x,lcResult.y,lcResult.r);
//			USART_printf("pulse0 %d\n",pulse0);
//			USART_printf("pulse1 %d\n",pulse1);
		//	USART_printf("angz %f\n",gyro_angle);
			
			
//			USART_printf("%d\n",pulse1);
//			USART_printf("%d\n",ENCODER_overflow_cnt[0]);
//			USART_printf("%d\n",__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1));
			//USART_printf("%d\n",ENCODER_TIM[0]->CNT);
//			if(flag)
//			{
//				USART_printf("angz %f    time  %d\n",Gyroz, time0);
//				USART_printf("angz %f    time  %d\n",Gyroz_a, time0);
//				USART_printf("angx %f    time  %d\n",Gyrox_a, time0);
//				USART_printf("angy %f    time  %d\n",Gyroy_a, time0);
//				USART_printf("buff10  %d\n",gyro_buffer[10]);
//				USART_printf("buff11  %d\n",gyro_buffer[11]);
//				USART_printf("buff12  %d\n",gyro_buffer[12]);
//				USART_printf("buff13  %d\n",gyro_buffer[13]);
//			}
			time1=time0;
		}
		if(time0-time2>=5)
		{
			#ifdef jiaozhun
				go_jiaozhun();
			#endif
			time2=time0;
		}
		if(time0-time3>=20)
		{
//			x+=0.01;
//			y+=0.02;
//			r-=0.01;
			//#ifndef jiaozhun
			//
//			USART_printf("position x%.6f y%.6f r%.6f \n",pos_x,pos_y,gyro_angle);
      //      relocate_cal();
			send_lc(lcResult.x,lcResult.y,lcResult.r,0,0);	  //输出串口  串口3   调试时不打开
			//#endif
			time3=time0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#ifdef jiaozhun
void go_jiaozhun(void)
{
	now_out_time=HAL_GetTick();
	//静止校准零漂
		if(jiaozhun_flag==1)
		{
			
			if(start_flag_jiaozhun)
			{
				gyro_angle_jiaozhun_start=lcResult.r;
				start_flag_jiaozhun=0;
				gyro_start_time_jiaozhun=HAL_GetTick();
			}
			gyro_angle_jiaozhun=lcResult.r-gyro_angle_jiaozhun_start;
			cnt++;
			if(now_out_time-last_out_time>400){
					USART_printf("gyro_angle=%f\n",gyro_angle_jiaozhun);
				last_out_time=now_out_time;
			}
			

			
			//floating_total+=gyro_angle_jiaozhun;
			//调整校准零漂时间
			if(cnt>floating_numbers&&jiaozhun_flag==1)
			{
				int gyro_end_time_jiaozhun=HAL_GetTick();
				floating=gyro_angle_jiaozhun/((gyro_end_time_jiaozhun-gyro_start_time_jiaozhun)*0.001);
				USART_printf("floating=%f\n",floating);
				USART_printf("time=%d\n",gyro_end_time_jiaozhun-gyro_start_time_jiaozhun);
				USART_printf("floating=%f\n",floating);
				USART_printf("floating=%f\n",floating);
				jiaozhun_flag=0;
				USART_printf("end floating jiaozhun!\n");
			}
		}
	//只平动   校准  脉冲和距离之间的换算关系   以及码盘坐标系和定位坐标系的换算关系  
		if(jiaozhun_flag==2)
		{
			//获取开始时刻的码盘脉冲值
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				start_flag_jiaozhun=0;
			}
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1)-pulse1_jiaozhun_start;
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0)-pulse0_jiaozhun_start;
			if(pulse_out)	
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				last_out_time=now_out_time;
			}

			}
			if(pulse_out==2)
			{
				USART_printf("endpulse0=%d  endpulse1=%d\n",pulse0_jiaozhun,pulse1_jiaozhun);
				USART_printf("end edncoder jiaozhun!\n");
				pulse_out=0;
			}
		}
		//只自转  校准  绕中心旋转的脉冲变化   分为正转和反转两部分 分别去校准两组系数
		//顺着陀螺仪角度值增大的方向旋转
		if(jiaozhun_flag==3)
		{
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				gyro_angle_jiaozhun_start=lcResult.r;
				last_angle_jiaozhun=gyro_angle_jiaozhun_start;
//				pulse0_jiaozhun_last=pulse0_jiaozhun_start;
//				pulse1_jiaozhun_last=pulse1_jiaozhun_start;
//				p_rotation_e0_total=0;
//				p_rotation_e1_total=0;
				p_del_angle_jiaozhun_total=0;
				start_flag_jiaozhun=0;
//				cnt_p=0;
				del_angle_jiaozhun=0;
			}
		

		
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0);
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1);
		
			del_angle_jiaozhun=lcResult.r-last_angle_jiaozhun;
//			int del_encoder0_jiaozhun=(pulse0_jiaozhun-pulse0_jiaozhun_last);
//			int del_encoder1_jiaozhun=(pulse1_jiaozhun-pulse1_jiaozhun_last);
			
			if(del_angle_jiaozhun>pi||del_angle_jiaozhun<-pi)
			{
				if(last_angle_jiaozhun<0)            // del_angle > pi  朝着角度减小的方向旋转一个小于pi的角度
				{
					last_angle_jiaozhun+=2*pi;
				}
				else                        //del_angle<-pi    朝着角度增加的方向旋转一个小于pi的角度
				{
					last_angle_jiaozhun-=2*pi;
				}
			}
			del_angle_jiaozhun=lcResult.r-last_angle_jiaozhun;     //得到一个正负号和规定角度正负号同相的    小于pi的角度

			if(del_angle_jiaozhun>0)
			{
//				 cnt_p++;
//				 p_rotation_e0_total+=(del_encoder0_jiaozhun/del_angle_jiaozhun);
//				 p_rotation_e1_total+=(del_encoder1_jiaozhun/del_angle_jiaozhun);
				  p_del_angle_jiaozhun_total+=del_angle_jiaozhun;
			}
			if(pulse_out)
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				USART_printf("gyro_angle=%f\n",lcResult.r);
					last_out_time=now_out_time;
				}
			}
			if(pulse_out==2)
			{
//				p_rotation_e0=p_rotation_e0_total/cnt_p;
//				p_rotation_e1=p_rotation_e1_total/cnt_p;
				p_rotation_e0=(pulse0_jiaozhun-pulse0_jiaozhun_start)/p_del_angle_jiaozhun_total;
				p_rotation_e1=(pulse1_jiaozhun-pulse1_jiaozhun_start)/p_del_angle_jiaozhun_total;
				USART_printf("p_rotation_e0=%f\n",p_rotation_e0);
				USART_printf("p_rotation_e1=%f\n",p_rotation_e1);
				USART_printf("p_total_ang=%f\n",p_del_angle_jiaozhun_total);
				USART_printf("go next locator jiaozhun!\n");
				pulse_out=0;
			}
			last_angle_jiaozhun=lcResult.r;
//			pulse0_jiaozhun_last=pulse0_jiaozhun;
//			pulse1_jiaozhun_last=pulse1_jiaozhun;
		}
		
		//顺着陀螺仪角度值减小的方向旋转
		if(jiaozhun_flag==4)
		{
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				gyro_angle_jiaozhun_start=lcResult.r;							
				last_angle_jiaozhun=gyro_angle_jiaozhun_start;
//				pulse0_jiaozhun_last=pulse0_jiaozhun_start;
//				pulse1_jiaozhun_last=pulse1_jiaozhun_start;
//				n_rotation_e0_total=0;
//				n_rotation_e1_total=0;
				n_del_angle_jiaozhun_total=0;
				start_flag_jiaozhun=0;
				del_angle_jiaozhun=0;
//				cnt_n=0;
			}

		
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0);
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1);
		
			del_angle_jiaozhun=lcResult.r-last_angle_jiaozhun;
//			int del_encoder0_jiaozhun=(pulse0_jiaozhun-pulse0_jiaozhun_last);
//			int del_encoder1_jiaozhun=(pulse1_jiaozhun-pulse1_jiaozhun_last);
			
			if(del_angle_jiaozhun>pi||del_angle_jiaozhun<-pi)
			{
				if(last_angle_jiaozhun<0)            // del_angle > pi  朝着角度减小的方向旋转一个小于pi的角度
				{
					last_angle_jiaozhun+=2*pi;
				}
				else                        //del_angle<-pi    朝着角度增加的方向旋转一个小于pi的角度
				{
					last_angle_jiaozhun-=2*pi;
				}
			}
			del_angle_jiaozhun=lcResult.r-last_angle_jiaozhun;     //得到一个正负号和规定角度正负号同相的    小于pi的角度

			if(del_angle_jiaozhun<0)
			{
//				 cnt_n++;
//				 n_rotation_e0_total+=(del_encoder0_jiaozhun/del_angle_jiaozhun);
//				 n_rotation_e1_total+=(del_encoder1_jiaozhun/del_angle_jiaozhun);
				 n_del_angle_jiaozhun_total+=del_angle_jiaozhun;
			}
			if(pulse_out)
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				USART_printf("gyro_angle=%f\n",lcResult.r);
					last_out_time=now_out_time;
				}
			}
			if(pulse_out==2)
			{
				n_rotation_e0=(pulse0_jiaozhun-pulse0_jiaozhun_start)/n_del_angle_jiaozhun_total;
				n_rotation_e1=(pulse1_jiaozhun-pulse1_jiaozhun_start)/n_del_angle_jiaozhun_total;
				USART_printf("n_rotation_e0=%f\n",n_rotation_e0);
				USART_printf("n_rotation_e1=%f\n",n_rotation_e1);
				USART_printf("n_total_ang=%f\n",n_del_angle_jiaozhun_total);
				USART_printf("end locator jiaozhun!\n");
				pulse_out=0;
			}
			last_angle_jiaozhun=lcResult.r;
//			pulse0_jiaozhun_last=pulse0_jiaozhun;
//			pulse1_jiaozhun_last=pulse1_jiaozhun;
		}
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
