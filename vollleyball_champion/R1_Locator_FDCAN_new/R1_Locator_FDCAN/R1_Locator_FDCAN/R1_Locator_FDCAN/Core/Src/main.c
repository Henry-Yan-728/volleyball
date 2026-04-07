/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "bsp_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef pi
#define pi 3.14159265358979323846f
#endif
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

#define jiaozhun

//r2 距离中心偏移
struct GyroResult gyroResult;
struct EncoderResult encoderRes;
volatile struct LocatorResult lcResult;
struct GyroResult timerGyroResult;
struct EncoderResult timerEncResult;
struct LocatorResult timerlcResult;
struct EncoderResult lastEncResult;
struct GyroResult lastGyroResult;
double QEI_pulse[2] = {0};
float encVel[2];
double vx_dis=0,vy_dis=0,vr_ang=0;
double total_x[5]={0},total_y[5]={0},total_r[5]={0};
int v_n=0;
float dis1=0,dis2=0,dis3=0;   //激光雷达信息
int if_relc=0;   //是否要重新定位
int send_times=0;
uint8_t restart_flag=0;

#ifdef jiaozhun
	double floating=0;
	double floating_total=0;
	int cnt=0;
	int cnt_p=0;  //正转圈数
	int cnt_n=0;  //反转圈数
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
	#define floating_numbers 5000
#endif
#ifndef jiaozhun
		#define p_rotation_e0 (-886.069336)
		#define p_rotation_e1 (-881.045837)
		#define n_rotation_e0 (-1401.902344)
		#define n_rotation_e1 (-657.607117)

	#define I_mul_x0 (0.01383840444)
	#define I_mul_x1 (0.01379097491)
	#define I_mul_y0 ( 0.01380621156)
	#define I_mul_y1 (-0.01376404342)
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

//定位输出   uart2发给上位机
void USART_printf(char *fmt, ...)
{
	va_list ap;
	char str[128];

	va_start(ap,fmt);
	vsprintf(str,fmt,ap);
	va_end(ap);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str),0xFFFF);
}

// 卡尔曼相关变量
int if_laser_ready=0;
State state ;
Covariance cov ; 
double process_noise[3][3] ;
double measurement_noise[3][3];

int data_state=0;   	//读取数据状态  0表示空闲  1表示执行
int cal_state=0;		//计算状态		0表示空闲  1表示执行
int now_state=0;		//目前状态     	1表示读取  0表示计算
int gyro_update=0;
int encoder_updata=0;
int gyro_start=0;
int if_gyro_err=0;
signed int gyro_vel0=0;
volatile double del_encoder0=0;
volatile double del_encoder1=0;
typedef   signed  int s32;
double floating0=0;
double convert[2]={20.0720416085,20.05672187};
int gyroloop=0;

//Pc缓存
uint8_t Pc_buffer[11];
int if_get_laser=0;

//卡尔曼状态更新用
float laser_x=0;
float laser_y=0;
float jiguang_x=0;
float jiguang_y=0;
float laser_data[3];

// 陀螺仪数据获取变量
volatile uint8_t gyro_buffer[2]; 
double gyro_angle=0;
int cnt1=0;
int cnt2=0;

//自动装载定时器 记录转数+1  反转数-1
volatile int32_t ENCODER_overflow_cnt[2] = {-1,-1};   
TIM_TypeDef* ENCODER_TIM[2]={TIM1, TIM2};

//获取编码器值
int32_t Get_Encoder_Pulse_Count(int i)
{
	return (ENCODER_overflow_cnt[i] * (1 + (ENCODER_TIM[i]->ARR)) + ENCODER_TIM[i]->CNT);    //得到总脉冲数
}	

float pos_x=0;
float pos_y=0;
float sum_del_encoder0=0;
float sum_del_encoder1=0;
float sum_del_angle=0;

/* ========================================================================= */
/* 维特智能 HWT906 解析状态机 */
/* ========================================================================= */
void Parse_WitMotion(uint8_t data) {
    static uint8_t rx_buf[11];
    static uint8_t rx_cnt = 0;

    rx_buf[rx_cnt++] = data;

    // 1. 检查帧头
    if (rx_buf[0] != 0x55) {
        rx_cnt = 0;
        return;
    }

    // 2. 接收满 11 个字节
    if (rx_cnt == 11) {
        // 3. 校验和
        uint8_t sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += rx_buf[i];
        }

        if (sum == rx_buf[10]) {
            // 4. 判断是否为角度数据包 (0x53)
            if (rx_buf[1] == 0x53) {
                // 解析 Yaw 角 (Z轴)
                short yaw_raw = (short)((rx_buf[7] << 8) | rx_buf[6]);
                
                // 维特协议换算公式：角度 = raw / 32768 * 180
                float yaw_degree = (float)yaw_raw / 32768.0f * 180.0f;

                // 转换为弧度
                gyro_angle = yaw_degree * (pi / 180.0f);
                
                // 更新传感器结构体
                gyroResult.rotation = gyro_angle;
                gyroResult.timeStamp = GenerateTimeStamp();
                
                // 同步读取编码器数据
                double pul0 = Get_Encoder_Pulse_Count(0);
                double pul1 = Get_Encoder_Pulse_Count(1);
                encoderRes.distance[0] += pul0 - QEI_pulse[0];
                encoderRes.distance[1] += pul1 - QEI_pulse[1];
                QEI_pulse[0] = pul0;
                QEI_pulse[1] = pul1;
                encoderRes.timeStamp = GenerateTimeStamp();
                cnt1++;
            }
        }
        rx_cnt = 0; 
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
	if(huart->Instance == USART1)   
	{
		// 维特陀螺仪数据解析
		Parse_WitMotion(gyro_buffer[0]);
		// 重新开启接收
		HAL_UART_Receive_IT(&huart1, (uint8_t*) gyro_buffer, 1);
		return;
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
			USART_printf("er111r\n");
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
		#else

		if(!if_get_laser)
		{
			if(Pc_buffer[0]==0xBE)
			{
				if_get_laser=1;
			}
		}
		else
		{
			static uint32_t data_base;
			if(!(Pc_buffer[0]==0xBF&&Pc_buffer[1]==0XCF))
			{
				if_get_laser=0;
			}
			else
			{
			    data_base=(uint32_t)Pc_buffer[5]<<24|(uint32_t)Pc_buffer[4]<<16|(uint32_t)Pc_buffer[3]<<8|(uint32_t)Pc_buffer[2];
				laser_x=(*((float *)(&data_base)));
				data_base=(uint32_t)Pc_buffer[9]<<24|(uint32_t)Pc_buffer[8]<<16|(uint32_t)Pc_buffer[7]<<8|(uint32_t)Pc_buffer[6];
				laser_y=*((float *)(&data_base));
                if_get_laser=0;
				laser_data[0]=laser_x;
				laser_data[1]=laser_y;
				laser_data[2]=lcResult.r;
				if_laser_ready=1;
			}
		}
		#endif  
	}
}

//E0 TIM1 A8 A9
//E1 TIM2 A0 A1
void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//encoder  0  
 	if(htim->Instance==TIM1)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			--ENCODER_overflow_cnt[0];
		}
		else
		{
			++ENCODER_overflow_cnt[0];
		}
	}
	//encoder 1  
	if(htim->Instance==TIM2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
		{
			--ENCODER_overflow_cnt[1];
		}
		else
		{
			++ENCODER_overflow_cnt[1];
		}	
	}
}
/*************************************************************/

void LocatorUpdate()
{
	lcResult.timeStamp = GenerateTimeStamp();
	double dx,dy,dr,sinr,cosr;
	double dz;
				
	struct GyroResult readGyro = gyroResult;
	struct EncoderResult readEnc = encoderRes;
    struct LocatorResult readlc = lcResult;
	
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
	dr=readGyro.rotation-lastGyroResult.rotation;

	//dr越界处理
	if (dr >= pi) dr -= 2*pi;
	else if (dr <= -pi) dr += 2*pi;
	
    lcResult.r += dr;
	vr_ang+=dr;
	if (lcResult.r >= pi) lcResult.r -= 2*pi;
	else if (lcResult.r <= -pi) lcResult.r += 2*pi;
	
	//实际为轮盘里程计变化
	dx = readEnc.distance[0]-lastEncResult.distance[0];
	dy = readEnc.distance[1]-lastEncResult.distance[1];
	
	//补偿旋转带来的位移量
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
	
	//转换为底盘坐标系下的dealt xy
	dz = dx;
	dx=I_mul_x0*dz+I_mul_x1*dy;
	dy=I_mul_y0*dz+I_mul_y1*dy;
	
	//转换为全局坐标系下的定位增量 dx dy
	dz = dx;
	sinr = sin(lcResult.r);
	cosr = cos(lcResult.r);
	dx = cosr*dz-sinr*dy;
	dy = sinr*dz+cosr*dy;
	
	lcResult.x += dx;
	lcResult.y += dy;
	vx_dis+=dx;
	vy_dis+=dy;

	lastEncResult = readEnc;
	lastGyroResult = readGyro;
	
	if(restart_flag==1)
	{
		lcResult.x=105;
		lcResult.y=105;
		lcResult.r=0;
		restart_flag=0;
	}
}

void LocatorSystemInitialize()
{
    gyroResult.timeStamp = 0; 
    gyroResult.rotation = 0;
	encoderRes.distance[0]=0;
	encoderRes.distance[1]=0;
	lcResult.timeStamp=0;
    lcResult.x = 105;
    lcResult.y = 105;
    lcResult.r = 0;
	lcResult.vx = 0;
    lcResult.vy = 0;
    lcResult.vAng = 0;
	lastEncResult = encoderRes;
    lastGyroResult = gyroResult;
}

void send_lc(float x, float y, float r, float vx, float vy, float vr) {
    static uint8_t data_byte[24];

    uint8_t* tmp1 = (uint8_t*)&x;
    data_byte[0] = tmp1[0]; data_byte[1] = tmp1[1]; data_byte[2] = tmp1[2]; data_byte[3] = tmp1[3];
    
    uint8_t* tmp2 = (uint8_t*)&y;
    data_byte[4] = tmp2[0]; data_byte[5] = tmp2[1]; data_byte[6] = tmp2[2]; data_byte[7] = tmp2[3];
    
    uint8_t* tmp3 = (uint8_t*)&r;
    data_byte[8] = tmp3[0]; data_byte[9] = tmp3[1]; data_byte[10] = tmp3[2]; data_byte[11] = tmp3[3];
    
    uint8_t* tmp4 = (uint8_t*)&vx;
    data_byte[12] = tmp4[0]; data_byte[13] = tmp4[1]; data_byte[14] = tmp4[2]; data_byte[15] = tmp4[3];
    
    uint8_t* tmp5 = (uint8_t*)&vy;
    data_byte[16] = tmp5[0]; data_byte[17] = tmp5[1]; data_byte[18] = tmp5[2]; data_byte[19] = tmp5[3];
    
    uint8_t* tmp6 = (uint8_t*)&vr;
    data_byte[20] = tmp6[0]; data_byte[21] = tmp6[1]; data_byte[22] = tmp6[2]; data_byte[23] = tmp6[3];

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0xAA;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; 
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; 
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; 
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_StatusTypeDef status;

    for (int i = 0; i < 3; i++) {
        TxHeader.Identifier = 0xAA + i; 
        uint8_t *current_packet_ptr = &data_byte[i * 8];

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
            status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, current_packet_ptr);
            if (status != HAL_OK) {
                printf("Classic CAN Send Packet %d Failed! Status: %d\r\n", i, status);
            }
        } else {
            printf("FDCAN Tx FIFO Full!\r\n");
            break; 
        }
    }
}

void laser_start(){
  FDCAN_TxHeaderTypeDef TxMessage;
    uint8_t txData[8];
    TxMessage.Identifier=0xAA;
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_8;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_ON;
    TxMessage.FDFormat = FDCAN_FD_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;
    txData[0]=txData[1]=txData[2]=txData[3]=txData[4]=txData[5]=txData[6]=txData[7]=0;
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&TxMessage,txData)!=HAL_OK)
    {
        printf("Send :Laser ERROR\n");
    }
}

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

uint32_t dis1_temp = 0;
uint32_t dis2_temp = 0;
uint32_t dis3_temp = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan==&hfdcan1){
			FDCAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];
			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            Error_Handler();
        }
			if(rx_header.Identifier==0xAE)
			{
				if(rx_data[0]=='r')
				{
					laser_start();
					restart_flag=1;
				}
			}
			if(rx_header.Identifier==0xBE)
			{
				if(rx_data[0]=='j')
				{
					if(dis1-5+23.06<2500  && dis1-5+23.06>200  &&  dis2-5-21.46>200 && dis2-5-21.46<2500)
					{
						lcResult.x=dis1-5+23.06;  
						lcResult.y=dis2-5-21.46;  
						lcResult.r=dis3;
					}
					else
					{
						lcResult.x=lcResult.x;
						lcResult.y=lcResult.y;
						lcResult.r=lcResult.r;
					}
					
					jiguang_restart_position( lcResult.x,lcResult.y,lcResult.r,lcResult.vx,lcResult.vy,lcResult.vAng);
				}
			}
    }
}

void restart_position();
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(hfdcan == &hfdcan2)
    {
        FDCAN_RxHeaderTypeDef rx_header;
				uint8_t rx_data[8];
				if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data)!=HAL_OK)
				{
					USART_printf("NO laser\n");
				}
				if(rx_header.Identifier==0x50)
				{
					dis1_temp=(uint32_t)((rx_data[3]<<24)|(rx_data[2]<<16)|(rx_data[1]<<8)|(rx_data[0]));
					dis1 = (float)dis1_temp/100.0f;
					dis2_temp=(uint32_t)((rx_data[7]<<24)|(rx_data[6]<<16)|(rx_data[5]<<8)|(rx_data[4]));
					dis2 = (float)dis2_temp/100.0f;
				}
    }
}

void jiguang_restart_position(float x ,float y ,float r,float vx,float vy,float vr)
{
	static uint8_t data_byte[24];
	uint8_t* tmp1 = (uint8_t*)&x;
	data_byte[0] = tmp1[0]; 
	data_byte[1] = tmp1[1];
	data_byte[2] = tmp1[2];
	data_byte[3] = tmp1[3];
	uint8_t* tmp2 = (uint8_t*)&y;
	data_byte[4] = tmp2[0]; 
	data_byte[5] = tmp2[1];
	data_byte[6] = tmp2[2];
	data_byte[7] = tmp2[3];                                                   
	uint8_t* tmp3 = (uint8_t*)&r;
	data_byte[8] = tmp3[0]; 
	data_byte[9] = tmp3[1];
	data_byte[10] = tmp3[2];
	data_byte[11] = tmp3[3];
	uint8_t* tmp4 = (uint8_t*)&vx;
	data_byte[12] = tmp4[0]; 
	data_byte[13] = tmp4[1];
	data_byte[14] = tmp4[2];
	data_byte[15] = tmp4[3];
	uint8_t* tmp5 = (uint8_t*)&vy;
	data_byte[16] = tmp5[0]; 
	data_byte[17] = tmp5[1];
	data_byte[18] = tmp5[2];
	data_byte[19] = tmp5[3];
	uint8_t* tmp6 = (uint8_t*)&vr;
	data_byte[20] = tmp6[0]; 
	data_byte[21] = tmp6[1];
	data_byte[22] = tmp6[2];
	data_byte[23] = tmp6[3];
	
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0xCE;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

	HAL_StatusTypeDef status;

    for (int i = 0; i < 3; i++) {
        TxHeader.Identifier = 0xCE + i;
        uint8_t *current_packet_ptr = &data_byte[i * 8];

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
            status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, current_packet_ptr);
            if (status != HAL_OK) {
                printf("FDCAN Send Packet %d Failed! Status: %d\r\n", i, status);
            }
        } else {
            printf("FDCAN Tx FIFO Full!\r\n");
            break; 
        }
	}
}

void restart_position()
{
	static uint8_t data_byte[1];
    data_byte[0] = (dis1_temp & 0XFF);

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0xAE;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data_byte) != HAL_OK)
    {
        printf("no sb!\n");
    }
}

// 归一化角度 [-PI, PI]
double normalize_angle(double angle) {
    while (angle > pi) angle -= 2 * pi;
    while (angle < -pi) angle += 2 * pi;
    return angle;
}

// 状态预测
void predict(State *state, Covariance *cov, double imu_x, double imu_y, double imu_theta, double Q[3][3]) {
    state->x = imu_x;
    state->y = imu_y;
    state->theta = normalize_angle(imu_theta);

    double F[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    double P_new[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                P_new[i][j] += F[i][k] * cov->P[k][j];
            }
            P_new[i][j] += Q[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov->P[i][j] = P_new[i][j];
        }
    }
}

// 状态更新
void update(State *state, Covariance *cov, float z[3], double R[3][3]) {
    double H[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    double y[3] = {
        z[0] - state->x,
        z[1] - state->y,
        z[2] - state->theta
    };
    y[2] = normalize_angle(y[2]);

    double S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                S[i][j] += H[i][k] * cov->P[k][j];
            }
            S[i][j] += R[i][j];
        }
    }

    double K[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = cov->P[i][j] / S[j][j]; 
        }
    }

    state->x += K[0][0] * y[0] + K[0][1] * y[1] + K[0][2] * y[2];
    state->y += K[1][0] * y[0] + K[1][1] * y[1] + K[1][2] * y[2];
    state->theta += K[2][0] * y[0] + K[2][1] * y[1] + K[2][2] * y[2];
    state->theta = normalize_angle(state->theta);

    double I_KH[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        I_KH[i][i] = 1.0;
        for (int j = 0; j < 3; j++) {
            I_KH[i][j] -= K[i][j] * H[j][j];
        }
    }

    double P_new[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                P_new[i][j] += I_KH[i][k] * cov->P[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov->P[i][j] = P_new[i][j];
        }
    }
}

void Merge_init()
{
	State state = {0, 0, 0};
    Covariance cov = { 
        {{0.1, 0, 0}, 
		 {0, 0.1, 0}, 
		 {0, 0, 0.1}} 
    };

    double process_noise[3][3] = {
        {0.01, 0, 0},
        {0, 0.01, 0},
        {0, 0, 0.01}
    };
    double measurement_noise[3][3] = {
        {2.974, 0, 0},
        {0, 4.974, 0},
        {0, 0, 0}
    };
}

void Merge_calculate()
{
	predict(&state, &cov, lcResult.x, lcResult.y, lcResult.r, process_noise);
	if(if_laser_ready)
	{
		update(&state, &cov, laser_data, measurement_noise);
		if_laser_ready=0;
		lcResult.x=laser_x;
		lcResult.y=laser_y;
		lcResult.r=state.theta;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, (uint8_t*) gyro_buffer, 1);
  HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
	
  // 【新增】启动 FDCAN1 和 FDCAN2
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
  
  // 【新增】开启 FDCAN 接收中断 (根据你的代码逻辑，1用FIFO0，2用FIFO1)
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  USART_printf("ENTER MAIN\n");
	
  int time0=HAL_GetTick();
  int time1=time0;
  int time2=time0;
  int time3=time0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LocatorSystemInitialize();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		LocatorUpdate();
		time0=HAL_GetTick();
		
		int32_t pulse0=Get_Encoder_Pulse_Count(0); 
		int32_t pulse1=Get_Encoder_Pulse_Count(1); 
		
		if(time0-time1>=400) 
		{
			//USART_printf("ang %f,cnt %d\r\n",gyro_angle,cnt1);
			time1=time0;
		}
		if(time0-time2>=2)
		{
			#ifdef jiaozhun
				go_jiaozhun();
			#endif
			time2=time0;
		}
		if(time0-time3>=20)
		{
			send_lc(lcResult.x, lcResult.y, lcResult.r, lcResult.vx, lcResult.vy, lcResult.vAng);
			time3=time0;
  		}
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

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

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
		if(jiaozhun_flag==1)
		{
			if(start_flag_jiaozhun)
			{
				gyro_angle_jiaozhun_start=gyro_angle;
				start_flag_jiaozhun=0;
				gyro_start_time_jiaozhun=HAL_GetTick();
			}
			gyro_angle_jiaozhun=gyro_angle-gyro_angle_jiaozhun_start;
			cnt++;
			if(now_out_time-last_out_time>400){
					USART_printf("gyro_angle=%f\n",gyro_angle_jiaozhun);
				last_out_time=now_out_time;
			}
			
			if(cnt>floating_numbers&&jiaozhun_flag==1)
			{
				int gyro_end_time_jiaozhun=HAL_GetTick();
				floating=gyro_angle_jiaozhun/((gyro_end_time_jiaozhun-gyro_start_time_jiaozhun)*0.001);
				USART_printf("floating=%f\n",floating);
				USART_printf("time=%d\n",gyro_end_time_jiaozhun-gyro_start_time_jiaozhun);
				jiaozhun_flag=0;
				USART_printf("end floating jiaozhun!\n");
			}
		}
		if(jiaozhun_flag==2)
		{
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
		if(jiaozhun_flag==3)
		{
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				gyro_angle_jiaozhun_start=gyro_angle;
				last_angle_jiaozhun=gyro_angle_jiaozhun_start;
				p_del_angle_jiaozhun_total=0;
				start_flag_jiaozhun=0;
				del_angle_jiaozhun=0;
			}
		
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0);
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1);
		
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;
			
			if(del_angle_jiaozhun>pi||del_angle_jiaozhun<-pi)
			{
				if(last_angle_jiaozhun<0)
				{
					last_angle_jiaozhun+=2*pi;
				}
				else
				{
					last_angle_jiaozhun-=2*pi;
				}
			}
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;

			if(del_angle_jiaozhun>0)
			{
				  p_del_angle_jiaozhun_total+=del_angle_jiaozhun;
			}
			if(pulse_out)
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				USART_printf("gyro_angle=%f\n",gyro_angle);
					last_out_time=now_out_time;
				}
			}
			if(pulse_out==2)
			{
				p_rotation_e0=(pulse0_jiaozhun-pulse0_jiaozhun_start)/p_del_angle_jiaozhun_total;
				p_rotation_e1=(pulse1_jiaozhun-pulse1_jiaozhun_start)/p_del_angle_jiaozhun_total;
				USART_printf("p_rotation_e0=%f\n",p_rotation_e0);
				USART_printf("p_rotation_e1=%f\n",p_rotation_e1);
				USART_printf("p_total_ang=%f\n",p_del_angle_jiaozhun_total);
				USART_printf("go next locator jiaozhun!\n");
				pulse_out=0;
			}
			last_angle_jiaozhun=gyro_angle;
		}
		
		if(jiaozhun_flag==4)
		{
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				gyro_angle_jiaozhun_start=gyro_angle;							
				last_angle_jiaozhun=gyro_angle_jiaozhun_start;
				n_del_angle_jiaozhun_total=0;
				start_flag_jiaozhun=0;
				del_angle_jiaozhun=0;
			}
		
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0);
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1);
		
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;
			
			if(del_angle_jiaozhun>pi||del_angle_jiaozhun<-pi)
			{
				if(last_angle_jiaozhun<0) 
				{
					last_angle_jiaozhun+=2*pi;
				}
				else
				{
					last_angle_jiaozhun-=2*pi;
				}
			}
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;

			if(del_angle_jiaozhun<0)
			{
				 n_del_angle_jiaozhun_total+=del_angle_jiaozhun;
			}
			if(pulse_out)
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				USART_printf("gyro_angle=%f\n",gyro_angle);
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
			last_angle_jiaozhun=gyro_angle;
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
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */