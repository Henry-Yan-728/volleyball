#include "fdcan_bsp.h"
#include <string.h> 

/* ------------------------- 绉佹湁瀹?------------------------- */
#define MAX_STD_DISPATCH_ITEMS 32     // 鏍囧噯甯ф敞鍐岄」鏈€澶ф暟閲?
#define MAX_EXT_DISPATCH_ITEMS 16     // 鎵╁睍甯ф敞鍐岄」鏈€澶ф暟閲?
#define HASH_TABLE_SIZE 2048          // 鏍囧噯甯?ID 鍝堝笇琛ㄥぇ灏?(瑕嗙洊 ID 0~0x7FF)
#define HASH_EMPTY_SLOT 0xFFFF        // 绌烘Ы浣嶆爣璁?

/* ------------------------- 绉佹湁鍙橀噺 ------------------------- */
static FDCAN_Dispatch_t std_dispatch_table[MAX_STD_DISPATCH_ITEMS]; // 鏍囧噯甯у垎鍙戣〃
static uint16_t hash_table[HASH_TABLE_SIZE]; // 蹇€熸煡鎵捐〃
static uint8_t std_dispatch_count = 0;       // 褰撳墠娉ㄥ唽鏁伴噺
static FDCAN_HandleTypeDef* std_dispatch_bus[MAX_STD_DISPATCH_ITEMS];

static FDCAN_Dispatch_t ext_dispatch_table[MAX_EXT_DISPATCH_ITEMS]; // 鎵╁睍甯у垎鍙戣〃锛堟帺鐮佸尮閰嶏級
static uint8_t ext_dispatch_count = 0;       // 鎵╁睍甯ф敞鍐屾暟閲?
static FDCAN_HandleTypeDef* ext_dispatch_bus[MAX_EXT_DISPATCH_ITEMS];

/* ------------------------- 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷 ------------------------- */
static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location);

/* ------------------------- 锟斤拷锟斤拷锟斤拷锟斤拷实锟斤拷 ------------------------- */

// 1. 鍒濆鍖?
void fdcan_bsp_init(void)
{
    for (int i = 0; i < HASH_TABLE_SIZE; ++i) {
        hash_table[i] = HASH_EMPTY_SLOT;
    }
    for (int i = 0; i < MAX_STD_DISPATCH_ITEMS; ++i) {
        std_dispatch_bus[i] = NULL;
    }
    for (int i = 0; i < MAX_EXT_DISPATCH_ITEMS; ++i) {
        ext_dispatch_bus[i] = NULL;
    }
    std_dispatch_count = 0;
    ext_dispatch_count = 0;
}

// 2. 娉ㄥ唽鍒嗗彂椤?
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan)
{
    if (dispatch_item->id_type == FDCAN_STANDARD_ID)
    {
        if (std_dispatch_count >= MAX_STD_DISPATCH_ITEMS || dispatch_item->id >= HASH_TABLE_SIZE) return;
        if (hash_table[dispatch_item->id] != HASH_EMPTY_SLOT) return;

        memcpy(&std_dispatch_table[std_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        std_dispatch_bus[std_dispatch_count] = hfdcan;
        hash_table[dispatch_item->id] = std_dispatch_count;
        std_dispatch_count++;
    }
    else if (dispatch_item->id_type == FDCAN_EXTENDED_ID)
    {
        if (ext_dispatch_count >= MAX_EXT_DISPATCH_ITEMS) return;
        memcpy(&ext_dispatch_table[ext_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        ext_dispatch_bus[ext_dispatch_count] = hfdcan;
        ext_dispatch_count++;
    }
}

// 3. 锟斤拷锟斤拷 CAN 锟斤拷锟斤拷
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 锟斤拷锟矫癸拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为全通 (锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫分凤拷)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000; // 锟斤拷锟斤拷为0锟斤拷锟斤拷示锟斤拷锟斤拷锟斤拷锟侥ｏ拷锟斤拷全通
    
    // 锟斤拷锟捷撅拷锟斤拷锟斤拷锟绞癸拷锟斤拷母锟?FIFO
    // 默锟较诧拷锟皆ｏ拷FDCAN1->FIFO0, FDCAN2->FIFO1, FDCAN3->FIFO0 (锟缴革拷锟斤拷Cubemx锟斤拷锟矫碉拷锟斤拷)
    if (hfdcan->Instance == FDCAN1) sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    else if (hfdcan->Instance == FDCAN2) sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    else sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN3 默锟斤拷 FIFO0

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);

    // 锟斤拷锟斤拷全锟街癸拷锟斤拷锟斤拷锟斤拷锟杰撅拷远锟斤拷帧
    uint32_t fifo_alloc = (hfdcan->Instance == FDCAN2) ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0;
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, fifo_alloc, fifo_alloc, FDCAN_FILTER_REJECT, FDCAN_FILTER_REJECT);

    // 锟斤拷锟斤拷锟叫讹拷
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
    if (hfdcan->Instance == FDCAN1) HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    else if (hfdcan->Instance == FDCAN2) HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    else HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // FDCAN3

    // 锟斤拷锟斤拷
    HAL_FDCAN_Start(hfdcan);
}

// 4. 通锟矫凤拷锟酵凤拷装
uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, tx_header, tx_data) != HAL_OK) return 1;
    return 0;
}

/* ------------------------- 锟叫断回碉拷锟斤拷锟斤拷 ------------------------- */

// FIFO0 锟截碉拷 (FDCAN1 锟斤拷 FDCAN3 默锟斤拷锟斤拷锟斤拷锟?
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO0);
    }
}

// FIFO1 锟截碉拷 (FDCAN2 默锟斤拷锟斤拷锟斤拷锟?
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO1);
    }
}

// 锟节诧拷路锟缴猴拷锟斤拷
static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64]; // [锟截碉拷] 锟斤拷锟斤拷锟斤拷锟斤拷锟?64 锟街节ｏ拷锟斤拷锟斤拷佣锟轿伙拷锟斤拷莼锟斤拷锟斤拷

    // 锟斤拷取锟斤拷锟斤拷
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo_location, &rx_header, rx_data) != HAL_OK) return;

    if (rx_header.IdType == FDCAN_STANDARD_ID)
    {
        uint16_t id = rx_header.Identifier;
        if (id < HASH_TABLE_SIZE && hash_table[id] != HASH_EMPTY_SLOT)
        {
            uint16_t index = hash_table[id];
            if (std_dispatch_bus[index] == hfdcan && std_dispatch_table[index].handler != NULL)
            {
                std_dispatch_table[index].handler(std_dispatch_table[index].instance_ptr, &rx_header, rx_data);
            }
        }
    }
    else if (rx_header.IdType == FDCAN_EXTENDED_ID)
    {
        uint32_t id = rx_header.Identifier;
        for (uint8_t i = 0; i < ext_dispatch_count; i++)
        {
            uint32_t mask = ext_dispatch_table[i].mask;
            if (ext_dispatch_bus[i] == hfdcan && (id & mask) == (ext_dispatch_table[i].id & mask))
            {
                if (ext_dispatch_table[i].handler != NULL)
                {
                    ext_dispatch_table[i].handler(ext_dispatch_table[i].instance_ptr, &rx_header, rx_data);
                }
            }
        }
    }
}
