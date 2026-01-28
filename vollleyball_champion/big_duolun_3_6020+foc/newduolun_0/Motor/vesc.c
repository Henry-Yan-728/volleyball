#include "vesc.h"
#include "fdcan.h"
#include "can_database.h"

/**************内部宏定义与重命名begin**************/

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
union s32_to_u8 vesc_content_transform[vesc_motor_nums]={0};//存储并转换vesc命令的内容
static int vesc_motor_poles_s[vesc_motor_nums]={0,21};//电机的极对数,这里从下标为1开始对应1号电机，下标为0的数据无用
static int motor_speed_s[vesc_motor_nums]={0};//发送电机速度，下标为电机ID
static void Write_database_vesc_extid(uint32_t id_num);//扩展ID报文发送函数
/**************内部变量与函数end**************/


/**************外部接口begin**************/
void Change_vesc_speed(int motor_id,int target_spd);//更改对应id的vesc速度
void Com2vesc(uint32_t motor_id);//间隔1ms内被调用一次，固定发送报文
void Vesc_speed_contral_init(void);//所有vesc初始速度设为0
/**************外部接口end**************/

/*
1.函数功能：更改vesc电机速度
2.入参：电机ID+目标速度
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_vesc_speed(int motor_id,int target_spd){
	 motor_speed_s[motor_id]=target_spd;
}

/*
1.函数功能：填充并发送 vesc 速度 控制报文的id和content
2.入参：电机ID
3.返回值：无
4.用法及调用要求：间隔1ms以内调用一次
5.其它：
*/
void Com2vesc(uint32_t motor_id){
	uint32_t vesc_speed_id=motor_id+(CAN_PACKET_SET_RPM<<8);
	uint32_t Erpm= motor_speed_s[motor_id] * vesc_motor_poles_s[motor_id];//转换为电角度
	vesc_content_transform[motor_id].u8_data[0]=Erpm>>24;
	vesc_content_transform[motor_id].u8_data[1]=Erpm>>16;
	vesc_content_transform[motor_id].u8_data[2]=Erpm>>8;
	vesc_content_transform[motor_id].u8_data[3]=Erpm;
	Write_database_vesc_extid(vesc_speed_id);
}

/*
1.函数功能：所有电调初始速度设为0
2.入参：无
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Vesc_speed_contral_init(void){
	for (int i=0;i<vesc_motor_nums;i++){
		motor_speed_s[i]=0;
	}
}

/*
1.函数功能：填充vesc控制报文并发送
2.入参：电机ID
3.返回值：无
4.用法及调用要求：
5.其它：注意错误检查以及发送阻滞的处理
*/
static void Write_database_vesc_extid(uint32_t id_num){
	FDCAN_TxHeaderTypeDef TxMessage;
	int sendnum=0;//发送次数，防止阻滞

	if((hash_table[id_num] >= can_data_num_g_vesc)){
		return;
	}//错误检查
	TxMessage.Identifier			  = can_database_g_vesc[hash_table[id_num]].Data_ID;;
	TxMessage.IdType			  = FDCAN_EXTENDED_ID ;
	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= 0X04;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;			/* FDCAN格式 */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */
	
	if(can_database_g_vesc[hash_table[id_num]].Channel==1){
		while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, can_database_g_vesc[hash_table[id_num]].Data_ptr) != HAL_OK){
			sendnum++;
			if(sendnum==10){
				break;//发送阻滞
			}
    }	
	}

}

#if 0

void set_vesc_position(uint32_t ID,int32_t location){
	uint32_t vesc_position_id=ID+(CAN_PACKET_SET_POS<<8);
	location*=1000000;
	vesc_content_transform[ID].u8_data[0]=location>>24;
	vesc_content_transform[ID].u8_data[1]=location>>16;
	vesc_content_transform[ID].u8_data[2]=location>>8;
	vesc_content_transform[ID].u8_data[3]=location;
	Write_database_vesc_extid(vesc_position_id);
}

void set_vesc_current(uint32_t ID,int32_t current){
	uint32_t vesc_current_id=ID+(CAN_PACKET_SET_CURRENT<<8);
	current*=1000;
	vesc_content_transform[ID].u8_data[0]=current>>24;
	vesc_content_transform[ID].u8_data[1]=current>>16;
	vesc_content_transform[ID].u8_data[2]=current>>8;
	vesc_content_transform[ID].u8_data[3]=current;
	Write_database_vesc_extid(vesc_current_id);
}

#endif

