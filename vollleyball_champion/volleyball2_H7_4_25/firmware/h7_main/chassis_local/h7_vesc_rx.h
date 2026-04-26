#ifndef H7_VESC_RX_H
#define H7_VESC_RX_H

#include "fdcan_bsp.h"

void H7Vesc_RxMessageHandler(void *instance_ptr,
                             FDCAN_RxHeaderTypeDef *rx_header,
                             uint8_t rx_data[64]);

#endif /* H7_VESC_RX_H */
