#ifndef vesc_h
#define vesc_h
#include "main.h" // 包含 main.h 以获取 STM32 类型定义

#define vesc_motor_nums 4 // 控制的电机总数

// VESC CAN commands (VESC 固件默认定义)
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
} CAN_PACKET_ID;

union s32_to_u8{     // 一个32位数据转为4个8位数据
	uint32_t s32_data;
	uint8_t u8_data[4];
};

extern union s32_to_u8 vesc_content_transform[vesc_motor_nums];

/**************外部接口begin**************/
// 1. 初始化 VESC 参数（如需配置 CAN 映射，可在此处扩展）
void Vesc_init(void);

// 2. 更改对应 ID 的 VESC 目标速度 (缓存，不发送)
void Change_vesc_speed(int motor_id, int target_spd);

// 3. 将缓存的速度指令发送给 VESC (间隔调用)
void Com2vesc(uint32_t motor_id);

// 4. 所有 VESC 初始速度设为 0
void Vesc_speed_control_init(void);
/**************外部接口end**************/

#endif