#ifndef VESC_RX_H
#define VESC_RX_H

#include <stdint.h>

extern int32_t vesc_motor_speed[8];

void get_vesc_speed(int id, uint8_t rx_data[8]);

#endif /* VESC_RX_H */
