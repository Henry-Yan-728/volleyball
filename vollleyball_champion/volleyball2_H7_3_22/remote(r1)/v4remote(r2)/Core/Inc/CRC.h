
#ifndef R1_DIPAN_CRC_H
#define R1_DIPAN_CRC_H


typedef enum  {
    Success,
    Fail
}uart_status;



#include "usart.h"

uint16_t CRC_16 (const uint8_t *data ,uint8_t length);



#endif //