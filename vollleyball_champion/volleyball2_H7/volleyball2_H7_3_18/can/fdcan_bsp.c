#include "fdcan_bsp.h"
#include "vesc_rx.h"

#include <string.h>

#define MAX_STD_DISPATCH_ITEMS 32
#define HASH_TABLE_SIZE 2048
#define HASH_EMPTY_SLOT 0xFFFF

static FDCAN_Dispatch_t std_dispatch_table[MAX_STD_DISPATCH_ITEMS];
static uint16_t hash_table[HASH_TABLE_SIZE];
static uint8_t std_dispatch_count = 0;

static void fdcan_dispatch_router(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_location);

void fdcan_bsp_init(void)
{
    for (int i = 0; i < HASH_TABLE_SIZE; ++i) {
        hash_table[i] = HASH_EMPTY_SLOT;
    }
    std_dispatch_count = 0;
}

void fdcan_bsp_register(FDCAN_Dispatch_t *dispatch_item, FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;

    if (dispatch_item == NULL) return;

    if (dispatch_item->id_type == FDCAN_STANDARD_ID) {
        if (std_dispatch_count >= MAX_STD_DISPATCH_ITEMS || dispatch_item->id >= HASH_TABLE_SIZE) return;
        if (hash_table[dispatch_item->id] != HASH_EMPTY_SLOT) return;

        memcpy(&std_dispatch_table[std_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        hash_table[dispatch_item->id] = std_dispatch_count;
        std_dispatch_count++;
    }
}

void fdcan_bsp_start(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef s_filter_config;

    s_filter_config.IdType = FDCAN_STANDARD_ID;
    s_filter_config.FilterIndex = 0;
    s_filter_config.FilterType = FDCAN_FILTER_MASK;
    s_filter_config.FilterID1 = 0x00000000;
    s_filter_config.FilterID2 = 0x00000000;

    if (hfdcan->Instance == FDCAN1) s_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    else if (hfdcan->Instance == FDCAN2) s_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    else s_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    (void)HAL_FDCAN_ConfigFilter(hfdcan, &s_filter_config);

    uint32_t fifo_alloc = (hfdcan->Instance == FDCAN2) ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0;
    (void)HAL_FDCAN_ConfigGlobalFilter(hfdcan, fifo_alloc, fifo_alloc, FDCAN_FILTER_REJECT, FDCAN_FILTER_REJECT);

    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
    if (hfdcan->Instance == FDCAN1) (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    else if (hfdcan->Instance == FDCAN2) (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    else (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    (void)HAL_FDCAN_Start(hfdcan);
}

uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, tx_header, tx_data) != HAL_OK) return 1U;
    return 0U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO1);
    }
}

static void fdcan_dispatch_router(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_location)
{
    (void)hfdcan;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo_location, &rx_header, rx_data) != HAL_OK) return;

    if (rx_header.IdType == FDCAN_STANDARD_ID) {
        uint16_t id = (uint16_t)rx_header.Identifier;
        if (id < HASH_TABLE_SIZE && hash_table[id] != HASH_EMPTY_SLOT) {
            uint16_t index = hash_table[id];
            if (std_dispatch_table[index].handler != NULL) {
                std_dispatch_table[index].handler(std_dispatch_table[index].instance_ptr, &rx_header, rx_data);
            }
        }
    } else if (rx_header.IdType == FDCAN_EXTENDED_ID) {
        uint8_t motor_id = (uint8_t)(rx_header.Identifier & 0xFFU);
        if (motor_id < 8U) {
            get_vesc_speed(motor_id, rx_data);
        }
    }
}
