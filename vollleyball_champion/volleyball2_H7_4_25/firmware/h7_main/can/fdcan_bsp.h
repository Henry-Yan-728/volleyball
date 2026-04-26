#ifndef FDCAN_BSP_H
#define FDCAN_BSP_H

#include "main.h"
#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

typedef struct
{
    uint32_t id_type;
    uint32_t id;
    uint32_t mask;
    void* instance_ptr;
    void (*handler)(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64]);
} FDCAN_Dispatch_t;

void fdcan_bsp_init(void);
void fdcan_bsp_register_all_dispatches(void);
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan);
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan);
uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data);

extern volatile uint32_t g_fdcan1_rx_fifo0_irq_count;
extern volatile uint32_t g_fdcan1_rx_fifo0_ext_count;
extern volatile uint32_t g_fdcan1_rx_fifo0_last_id;
extern volatile uint32_t g_fdcan_tx_mutex_timeout_count;
extern volatile uint32_t g_fdcan_tx_fifo_full_count;
extern volatile uint32_t g_fdcan_tx_hal_fail_count;
extern volatile uint32_t g_fdcan_bus_off_count;
extern volatile uint32_t g_fdcan_bus_off_recover_count;
extern volatile uint32_t g_fdcan_bus_off_recover_fail_count;

#endif // FDCAN_BSP_H
