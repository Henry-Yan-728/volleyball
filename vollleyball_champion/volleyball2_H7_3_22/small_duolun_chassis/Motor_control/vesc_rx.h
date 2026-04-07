#ifndef vesc_rx_h
#define vesc_rx_h

#include <stdint.h>

#define CAN_5065_M1_ID 0x00000901
#define CAN_5065_M2_ID 0x00000902
#define CAN_5065_M3_ID 0x00000903

extern int32_t vesc_motor_speed[8];

void get_vesc_speed(int id,uint8_t rx_data[8]);

#endif
