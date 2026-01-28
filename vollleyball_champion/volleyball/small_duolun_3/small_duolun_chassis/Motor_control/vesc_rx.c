#include "vesc_rx.h"
#include "vesc.h"
#include "stdio.h"

int32_t vesc_motor_speed[8]={0};//?

void get_vesc_speed(int id,uint8_t rx_data[8])
{
    uint32_t temp_u32 = ((uint32_t)rx_data[0] << 24 | (uint32_t)rx_data[1] << 16 | (uint32_t)rx_data[2] << 8 | (uint32_t)rx_data[3]);
    vesc_motor_speed[id] = (int32_t)temp_u32;
    //USART_printf("vesc_speed[%d]=%d\n",id,vesc_motor_speed[id]);
}