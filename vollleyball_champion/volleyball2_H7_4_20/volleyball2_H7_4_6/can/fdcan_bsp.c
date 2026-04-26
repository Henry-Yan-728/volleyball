#include "fdcan_bsp.h"

#include <string.h>

#include "dji_motor.h"
#include "robot_data.h"
#include "cybergear_motor.h"

#ifdef USE_FREERTOS
    #include "FreeRTOS.h"
    #include "semphr.h"
#endif

#define MAX_STD_DISPATCH_ITEMS 32U
#define MAX_EXT_DISPATCH_ITEMS 16U
#define HASH_TABLE_SIZE 2048U
#define HASH_EMPTY_SLOT 0xFFFFU

static FDCAN_Dispatch_t std_dispatch_table[MAX_STD_DISPATCH_ITEMS];
static uint16_t hash_table[HASH_TABLE_SIZE];
static uint8_t std_dispatch_count = 0U;
static FDCAN_HandleTypeDef* std_dispatch_bus[MAX_STD_DISPATCH_ITEMS];

static FDCAN_Dispatch_t ext_dispatch_table[MAX_EXT_DISPATCH_ITEMS];
static uint8_t ext_dispatch_count = 0U;
static FDCAN_HandleTypeDef* ext_dispatch_bus[MAX_EXT_DISPATCH_ITEMS];
static uint8_t s_dispatches_registered = 0U;

#ifdef USE_FREERTOS
static SemaphoreHandle_t s_fdcan_tx_mutex[3] = {NULL, NULL, NULL};
#endif

volatile uint32_t g_fdcan1_rx_fifo0_irq_count = 0U;
volatile uint32_t g_fdcan1_rx_fifo0_ext_count = 0U;
volatile uint32_t g_fdcan1_rx_fifo0_last_id = 0U;
volatile uint32_t g_fdcan_tx_mutex_timeout_count = 0U;
volatile uint32_t g_fdcan_tx_fifo_full_count = 0U;
volatile uint32_t g_fdcan_tx_hal_fail_count = 0U;
volatile uint32_t g_fdcan_bus_off_count = 0U;
volatile uint32_t g_fdcan_bus_off_recover_count = 0U;
volatile uint32_t g_fdcan_bus_off_recover_fail_count = 0U;

static uint8_t fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location);
static uint8_t fdcan_get_bus_index(FDCAN_HandleTypeDef* hfdcan, uint8_t* bus_index);
static HAL_StatusTypeDef fdcan_activate_notifications(FDCAN_HandleTypeDef* hfdcan);

void fdcan_bsp_init(void)
{
    for (uint16_t i = 0U; i < HASH_TABLE_SIZE; ++i)
    {
        hash_table[i] = HASH_EMPTY_SLOT;
    }

    for (uint8_t i = 0U; i < MAX_STD_DISPATCH_ITEMS; ++i)
    {
        std_dispatch_bus[i] = NULL;
    }

    for (uint8_t i = 0U; i < MAX_EXT_DISPATCH_ITEMS; ++i)
    {
        ext_dispatch_bus[i] = NULL;
    }

    std_dispatch_count = 0U;
    ext_dispatch_count = 0U;
    s_dispatches_registered = 0U;

#ifdef USE_FREERTOS
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (s_fdcan_tx_mutex[i] == NULL)
        {
            s_fdcan_tx_mutex[i] = xSemaphoreCreateMutex();
        }
    }
#endif
}

void fdcan_bsp_register_all_dispatches(void)
{
    if (s_dispatches_registered != 0U)
    {
        return;
    }

    Robot_Data_Register_Dispatches();
    dji_motors_register_dispatches();
    cybergear_motors_register_dispatches();

    s_dispatches_registered = 1U;
}

void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan)
{
    if ((dispatch_item == NULL) || (hfdcan == NULL) || (dispatch_item->handler == NULL))
    {
        return;
    }

    if (dispatch_item->id_type == FDCAN_STANDARD_ID)
    {
        if ((std_dispatch_count >= MAX_STD_DISPATCH_ITEMS) || (dispatch_item->id >= HASH_TABLE_SIZE))
        {
            return;
        }

        if (hash_table[dispatch_item->id] != HASH_EMPTY_SLOT)
        {
            return;
        }

        memcpy(&std_dispatch_table[std_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        std_dispatch_bus[std_dispatch_count] = hfdcan;
        hash_table[dispatch_item->id] = std_dispatch_count;
        std_dispatch_count++;
        return;
    }

    if (dispatch_item->id_type == FDCAN_EXTENDED_ID)
    {
        if (ext_dispatch_count >= MAX_EXT_DISPATCH_ITEMS)
        {
            return;
        }

        memcpy(&ext_dispatch_table[ext_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        ext_dispatch_bus[ext_dispatch_count] = hfdcan;
        ext_dispatch_count++;
    }
}

void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    if (hfdcan == NULL)
    {
        Error_Handler();
        return;
    }

    if (hfdcan->Instance == FDCAN2)
    {
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = 0U;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        sFilterConfig.FilterID1 = 0x200U;
        sFilterConfig.FilterID2 = 0x7F0U;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        sFilterConfig.IdType = FDCAN_EXTENDED_ID;
        sFilterConfig.FilterIndex = 0U;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterID1 = 0U;
        sFilterConfig.FilterID2 = 0U;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

    }
    else if (hfdcan->Instance == FDCAN3)
    {
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

        sFilterConfig.FilterIndex = 0U;
        sFilterConfig.FilterID1 = CAN_ID_POSE_PART1;
        sFilterConfig.FilterID2 = CAN_ID_POSE_PART2;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        sFilterConfig.FilterIndex = 1U;
        sFilterConfig.FilterID1 = CAN_ID_POSE_PART3;
        sFilterConfig.FilterID2 = CAN_ID_PC_SET_TARGET;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        sFilterConfig.FilterIndex = 2U;
        sFilterConfig.FilterID1 = CAN_ID_PC_PAN_TILT;
        sFilterConfig.FilterID2 = 0x7FFU;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        sFilterConfig.IdType = FDCAN_EXTENDED_ID;
        sFilterConfig.FilterIndex = 0U;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterID1 = 0U;
        sFilterConfig.FilterID2 = 0U;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else if (hfdcan->Instance == FDCAN1)
    {
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = 0U;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterID1 = 0U;
        sFilterConfig.FilterID2 = 0U;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        sFilterConfig.IdType = FDCAN_EXTENDED_ID;
        sFilterConfig.FilterIndex = 0U;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterID1 = 0U;
        sFilterConfig.FilterID2 = 0U;

        for (uint8_t i = 0U; i < ext_dispatch_count; i++)
        {
            if (ext_dispatch_bus[i] == hfdcan)
            {
                sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
                sFilterConfig.FilterID1 = ext_dispatch_table[i].id;
                sFilterConfig.FilterID2 = ext_dispatch_table[i].mask;
                break;
            }
        }

        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            hfdcan,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (fdcan_activate_notifications(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;

    if (hfdcan == NULL) {
        return;
    }

    if (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status) != HAL_OK) {
        g_fdcan_bus_off_recover_fail_count++;
        return;
    }

    if (protocol_status.BusOff == 0U) {
        return;
    }

    g_fdcan_bus_off_count++;
    (void)HAL_FDCAN_Stop(hfdcan);

    if ((fdcan_activate_notifications(hfdcan) != HAL_OK) ||
        (HAL_FDCAN_Start(hfdcan) != HAL_OK)) {
        g_fdcan_bus_off_recover_fail_count++;
        return;
    }

    g_fdcan_bus_off_recover_count++;
}

uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data)
{
    uint8_t bus_index = 0U;
    uint8_t lock_taken = 0U;

    if ((hfdcan == NULL) || (tx_header == NULL) || (tx_data == NULL))
    {
        return 1U;
    }

#ifdef USE_FREERTOS
    if ((__get_IPSR() == 0U) &&
        (fdcan_get_bus_index(hfdcan, &bus_index) != 0U) &&
        (s_fdcan_tx_mutex[bus_index] != NULL))
    {
        if (xSemaphoreTake(s_fdcan_tx_mutex[bus_index], pdMS_TO_TICKS(1)) != pdTRUE)
        {
            g_fdcan_tx_mutex_timeout_count++;
            return 1U;
        }

        lock_taken = 1U;
    }
#endif

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        g_fdcan_tx_fifo_full_count++;
#ifdef USE_FREERTOS
        if (lock_taken != 0U)
        {
            xSemaphoreGive(s_fdcan_tx_mutex[bus_index]);
        }
#endif
        return 1U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, tx_header, tx_data) != HAL_OK)
    {
        g_fdcan_tx_hal_fail_count++;
#ifdef USE_FREERTOS
        if (lock_taken != 0U)
        {
            xSemaphoreGive(s_fdcan_tx_mutex[bus_index]);
        }
#endif
        return 1U;
    }

#ifdef USE_FREERTOS
    if (lock_taken != 0U)
    {
        xSemaphoreGive(s_fdcan_tx_mutex[bus_index]);
    }
#endif

    return 0U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            g_fdcan1_rx_fifo0_irq_count++;
        }

        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
        {
            if (fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO0) == 0U)
            {
                break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
        {
            if (fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO1) == 0U)
            {
                break;
            }
        }
    }
}

static uint8_t fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo_location, &rx_header, rx_data) != HAL_OK)
    {
        return 0U;
    }

    if (rx_header.IdType == FDCAN_STANDARD_ID)
    {
        uint16_t id = (uint16_t)rx_header.Identifier;

        if ((id < HASH_TABLE_SIZE) && (hash_table[id] != HASH_EMPTY_SLOT))
        {
            uint16_t index = hash_table[id];

            if ((std_dispatch_bus[index] == hfdcan) && (std_dispatch_table[index].handler != NULL))
            {
                std_dispatch_table[index].handler(std_dispatch_table[index].instance_ptr, &rx_header, rx_data);
            }
        }

        return 1U;
    }

    if (rx_header.IdType == FDCAN_EXTENDED_ID)
    {
        uint32_t id = rx_header.Identifier;

        if (hfdcan->Instance == FDCAN1)
        {
            g_fdcan1_rx_fifo0_ext_count++;
            g_fdcan1_rx_fifo0_last_id = id;
        }

        for (uint8_t i = 0U; i < ext_dispatch_count; i++)
        {
            uint32_t mask = ext_dispatch_table[i].mask;

            if ((ext_dispatch_bus[i] == hfdcan) &&
                ((id & mask) == (ext_dispatch_table[i].id & mask)) &&
                (ext_dispatch_table[i].handler != NULL))
            {
                ext_dispatch_table[i].handler(ext_dispatch_table[i].instance_ptr, &rx_header, rx_data);
            }
        }
    }

    return 1U;
}

static uint8_t fdcan_get_bus_index(FDCAN_HandleTypeDef* hfdcan, uint8_t* bus_index)
{
    if ((hfdcan == NULL) || (bus_index == NULL))
    {
        return 0U;
    }

    if (hfdcan->Instance == FDCAN1)
    {
        *bus_index = 0U;
        return 1U;
    }

    if (hfdcan->Instance == FDCAN2)
    {
        *bus_index = 1U;
        return 1U;
    }

    if (hfdcan->Instance == FDCAN3)
    {
        *bus_index = 2U;
        return 1U;
    }

    return 0U;
}

static HAL_StatusTypeDef fdcan_activate_notifications(FDCAN_HandleTypeDef* hfdcan)
{
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (hfdcan->Instance == FDCAN2)
    {
        return HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U);
    }

    return HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}
