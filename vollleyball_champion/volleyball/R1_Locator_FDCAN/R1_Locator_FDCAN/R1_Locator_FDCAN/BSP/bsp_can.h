#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "fdcan.h"
//#include "dji_3508_2006_motor.h"
//#include "vesc.h"
//#include "vesc_rx.h"
#include "stm32g4xx.h"

#ifndef NULL
	#define NULL ((void *)0)//此处的NULL指示candatabase中的NULL
#endif
#define READ_ONLY  0    //主控读，外设写
#define WRITE_ONLY 1    //主控写，外设读

typedef enum{
//	vesc_motor1=0x001+(CAN_PACKET_SET_RPM<<8),//新增vesc电机ID号
//	vesc_motor2=0x002+(CAN_PACKET_SET_RPM<<8),
//	vesc_motor3=0x003+(CAN_PACKET_SET_RPM<<8),
	odrive_motor1=0x01,//新增odrive电机号
} ID_NUMDEF;

typedef struct
{
    uint8_t  Data_type;
    ID_NUMDEF  Data_ID;
    uint8_t* Data_ptr;
    uint8_t  Data_length;   
    void (*MenuFunc)(void);//入口函数        
    uint8_t  Channel;
		uint32_t  Fifo_num;//在接收方将该ID配置的fifo号
} Can_Data;

void FDCAN1_RxFilter_Config(void);
void FDCAN2_RxFilter_Config(void);

void Hash_table_init(void);

extern uint8_t can_data_num_g;//结构体数组can_database_g的大小，在Hash_table_init(void)中更新
extern Can_Data can_database_g[];
extern uint16_t hash_table[1000];

#endif
