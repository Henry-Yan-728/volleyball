#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "jiguang_relocate.h"
#include "math.h"

float dis1=0,dis2=0,dis3=0;   //激光距离信息
int if_relc=0;   //是否要进行重定位
//存储激光重定位坐标
float reloc_x = 0;
float reloc_y = 0;
float dis1_0 = 0;
float dis2_0 = 0;

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(hfdcan == &hfdcan2)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        {
            USART_printf("NO, SB!");
            Error_Handler();
        }
        if(rx_header.Identifier == 0x012)
        {
            dis1=((rx_data[3]<<24)|(rx_data[2]<<16)|(rx_data[1]<<8)|(rx_data[0]));
			dis1=*((float*)&dis1);
        }
        else if(rx_header.Identifier == 0x023)
        {
			dis2=((rx_data[3]<<24)|(rx_data[2]<<16)|(rx_data[1]<<8)|(rx_data[0]));
			dis2=*((float*)&dis2);
        }
//		else if(rx_header.Identifier==0x034)
//		{
//			dis3=((rx_data[3]<<24)|(rx_data[2]<<16)|(rx_data[1]<<8)|(rx_data[0]));
//			dis3=*((float*)&dis3);
//		}
    }
}

/**
  * @brief  根据激光和陀螺仪数据更新车体位置
  * @param  theta 车体方向（弧度，0为Y轴正方向）
  * @param  dis1 激光1测量值（左侧）
  * @param  dis2 激光2测量值（后方）
  */

void relocate_cal(void)
{
    if(fabs(lcResult.r) < 0.3490658503988659)
    {
        float cos_theta = cosf(fabs(lcResult.r));
        float sin_theta = sinf(fabs(lcResult.r));
        reloc_x = dis1 * cos_theta - dis1_0;
        reloc_y = dis2 * cos_theta - dis2_0;
        lcResult.x = reloc_x;
        lcResult.y = reloc_y;
    }
    
    
}