#include "fdcan_bsp.h"
#include <string.h>

/* ------------------------- Private Configuration ------------------------- */
#define MAX_STD_DISPATCH_ITEMS 32U
#define MAX_EXT_DISPATCH_ITEMS 16U
#define HASH_TABLE_SIZE        2048U
#define HASH_EMPTY_SLOT        0xFFFFU
#define STD_ID_MASK            0x7FFU
#define EXT_ID_MASK            0x1FFFFFFFU

/* ------------------------- Private State ------------------------- */
static FDCAN_Dispatch_t std_dispatch_table[MAX_STD_DISPATCH_ITEMS];
static uint16_t hash_table[HASH_TABLE_SIZE];
static uint8_t std_dispatch_count = 0U;

static FDCAN_Dispatch_t ext_dispatch_table[MAX_EXT_DISPATCH_ITEMS];
static uint8_t ext_dispatch_count = 0U;

/* ------------------------- Internal Prototypes ------------------------- */
static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location);
static uint8_t fdcan_register_std(const FDCAN_Dispatch_t* dispatch_item);
static uint8_t fdcan_register_ext(const FDCAN_Dispatch_t* dispatch_item);
static void fdcan_dispatch_std(FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64]);
static void fdcan_dispatch_ext(FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64]);

/* ------------------------- Public API ------------------------- */

void fdcan_bsp_init(void)
{
    uint32_t i;

    for (i = 0U; i < HASH_TABLE_SIZE; ++i) {
        hash_table[i] = HASH_EMPTY_SLOT;
    }

    memset(std_dispatch_table, 0, sizeof(std_dispatch_table));
    memset(ext_dispatch_table, 0, sizeof(ext_dispatch_table));

    std_dispatch_count = 0U;
    ext_dispatch_count = 0U;
}

void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan)
{
    if ((dispatch_item == NULL) || (dispatch_item->handler == NULL)) {
        return;
    }

    /* Keep API compatibility. Routing table is shared by all configured CAN buses. */
    (void)hfdcan;

    if (dispatch_item->id_type == FDCAN_STANDARD_ID) {
        (void)fdcan_register_std(dispatch_item);
    } else if (dispatch_item->id_type == FDCAN_EXTENDED_ID) {
        (void)fdcan_register_ext(dispatch_item);
    }
}

void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;
    uint32_t fifo_alloc;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterID1 = 0x00000000U;
    sFilterConfig.FilterID2 = 0x00000000U;

    if (hfdcan->Instance == FDCAN1) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    } else if (hfdcan->Instance == FDCAN2) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    } else {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    }

    (void)HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);

    fifo_alloc = (hfdcan->Instance == FDCAN2) ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0;
    (void)HAL_FDCAN_ConfigGlobalFilter(hfdcan, fifo_alloc, fifo_alloc, FDCAN_FILTER_REJECT, FDCAN_FILTER_REJECT);

    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
    if (hfdcan->Instance == FDCAN1) {
        (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    } else if (hfdcan->Instance == FDCAN2) {
        (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    } else {
        (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }

    (void)HAL_FDCAN_Start(hfdcan);
}

uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, tx_header, tx_data) != HAL_OK) {
        return 1U;
    }
    return 0U;
}

/* ------------------------- HAL Callbacks ------------------------- */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO1);
    }
}

/* ------------------------- Internal Implementation ------------------------- */

static uint8_t fdcan_register_std(const FDCAN_Dispatch_t* dispatch_item)
{
    uint32_t std_id;

    if (std_dispatch_count >= MAX_STD_DISPATCH_ITEMS) {
        return 1U;
    }

    std_id = dispatch_item->id & STD_ID_MASK;
    if (std_id >= HASH_TABLE_SIZE) {
        return 1U;
    }

    if (hash_table[std_id] != HASH_EMPTY_SLOT) {
        return 1U;
    }

    std_dispatch_table[std_dispatch_count] = *dispatch_item;
    std_dispatch_table[std_dispatch_count].id_type = FDCAN_STANDARD_ID;
    std_dispatch_table[std_dispatch_count].id = std_id;
    std_dispatch_table[std_dispatch_count].mask = STD_ID_MASK;

    hash_table[std_id] = std_dispatch_count;
    std_dispatch_count++;

    return 0U;
}

static uint8_t fdcan_register_ext(const FDCAN_Dispatch_t* dispatch_item)
{
    if (ext_dispatch_count >= MAX_EXT_DISPATCH_ITEMS) {
        return 1U;
    }

    ext_dispatch_table[ext_dispatch_count] = *dispatch_item;
    ext_dispatch_table[ext_dispatch_count].id_type = FDCAN_EXTENDED_ID;
    ext_dispatch_table[ext_dispatch_count].id &= EXT_ID_MASK;

    if (ext_dispatch_table[ext_dispatch_count].mask == 0U) {
        ext_dispatch_table[ext_dispatch_count].mask = EXT_ID_MASK;
    } else {
        ext_dispatch_table[ext_dispatch_count].mask &= EXT_ID_MASK;
    }

    ext_dispatch_count++;
    return 0U;
}

static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo_location, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if (rx_header.IdType == FDCAN_STANDARD_ID) {
        fdcan_dispatch_std(&rx_header, rx_data);
    } else if (rx_header.IdType == FDCAN_EXTENDED_ID) {
        fdcan_dispatch_ext(&rx_header, rx_data);
    }
}

static void fdcan_dispatch_std(FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    uint32_t std_id;
    uint16_t index;

    std_id = rx_header->Identifier & STD_ID_MASK;
    if (std_id >= HASH_TABLE_SIZE) {
        return;
    }

    index = hash_table[std_id];
    if (index == HASH_EMPTY_SLOT) {
        return;
    }

    if (std_dispatch_table[index].handler != NULL) {
        std_dispatch_table[index].handler(std_dispatch_table[index].instance_ptr, rx_header, rx_data);
    }
}

static void fdcan_dispatch_ext(FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64])
{
    uint32_t i;
    uint32_t rx_id;

    rx_id = rx_header->Identifier & EXT_ID_MASK;

    for (i = 0U; i < ext_dispatch_count; ++i) {
        uint32_t mask;
        uint32_t item_id;

        mask = ext_dispatch_table[i].mask;
        item_id = ext_dispatch_table[i].id;

        if (((rx_id ^ item_id) & mask) == 0U) {
            if (ext_dispatch_table[i].handler != NULL) {
                ext_dispatch_table[i].handler(ext_dispatch_table[i].instance_ptr, rx_header, rx_data);
            }
        }
    }
}
