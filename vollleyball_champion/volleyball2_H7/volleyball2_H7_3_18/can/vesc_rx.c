#include "vesc_rx.h"

int32_t vesc_motor_speed[8] = {0};

void get_vesc_speed(int id, uint8_t rx_data[8])
{
    if ((id < 0) || (id >= 8)) return;

    uint32_t temp_u32 =
        ((uint32_t)rx_data[0] << 24) |
        ((uint32_t)rx_data[1] << 16) |
        ((uint32_t)rx_data[2] << 8)  |
        ((uint32_t)rx_data[3]);

    vesc_motor_speed[id] = (int32_t)temp_u32;
}
