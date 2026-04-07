#ifndef VESC_H
#define VESC_H

#include "main.h"

#define vesc_motor_nums 4

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
} CAN_PACKET_ID;

union s32_to_u8 {
    uint32_t s32_data;
    uint8_t u8_data[4];
};

extern union s32_to_u8 vesc_content_transform[vesc_motor_nums];

void Vesc_init(void);
void Change_vesc_speed(int motor_id, int target_spd);
void Com2vesc(uint32_t motor_id);
void Vesc_speed_control_init(void);

#endif /* VESC_H */
