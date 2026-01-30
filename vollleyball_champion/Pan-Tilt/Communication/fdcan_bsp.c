#include "fdcan_bsp.h"
#include <string.h> 

/* ------------------------- 私有配置 ------------------------- */
#define MAX_STD_DISPATCH_ITEMS 32     // 标准帧注册项最大数量
#define HASH_TABLE_SIZE 2048          // 标准帧 ID 哈希表大小 (覆盖 ID 0~0x7FF)
#define HASH_EMPTY_SLOT 0xFFFF        // 空槽位标记

/* ------------------------- 私有变量 ------------------------- */
static FDCAN_Dispatch_t std_dispatch_table[MAX_STD_DISPATCH_ITEMS]; // 分发表
static uint16_t hash_table[HASH_TABLE_SIZE]; // 快速查找表
static uint8_t std_dispatch_count = 0;       // 当前注册数量

/* ------------------------- 内部函数声明 ------------------------- */
static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location);

/* ------------------------- 公开函数实现 ------------------------- */

// 1. 初始化
void fdcan_bsp_init(void)
{
    // 清空哈希表
    for (int i = 0; i < HASH_TABLE_SIZE; ++i) {
        hash_table[i] = HASH_EMPTY_SLOT;
    }
    std_dispatch_count = 0;
}

// 2. 注册监听
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan)
{
    // 目前仅简易实现了标准帧 (Standard ID) 的支持
    if (dispatch_item->id_type == FDCAN_STANDARD_ID)
    {
        if (std_dispatch_count >= MAX_STD_DISPATCH_ITEMS || dispatch_item->id >= HASH_TABLE_SIZE) return;
        if (hash_table[dispatch_item->id] != HASH_EMPTY_SLOT) return; // 防止重复注册

        // 复制到分发表
        memcpy(&std_dispatch_table[std_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        
        // 建立映射 ID -> Table Index
        hash_table[dispatch_item->id] = std_dispatch_count;
        
        std_dispatch_count++;
    }
}

// 3. 启动 CAN 总线
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 配置过滤器：设置为全通 (接收所有数据，由软件进行分发)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000; // 掩码为0，表示都不关心，即全通
    
    // 根据句柄决定使用哪个 FIFO
    // 默认策略：FDCAN1->FIFO0, FDCAN2->FIFO1, FDCAN3->FIFO0 (可根据Cubemx配置调整)
    if (hfdcan->Instance == FDCAN1) sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    else if (hfdcan->Instance == FDCAN2) sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    else sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN3 默认 FIFO0

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);

    // 配置全局过滤器：拒绝远程帧
    uint32_t fifo_alloc = (hfdcan->Instance == FDCAN2) ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0;
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, fifo_alloc, fifo_alloc, FDCAN_FILTER_REJECT, FDCAN_FILTER_REJECT);

    // 开启中断
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
    if (hfdcan->Instance == FDCAN1) HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    else if (hfdcan->Instance == FDCAN2) HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    else HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // FDCAN3

    // 启动
    HAL_FDCAN_Start(hfdcan);
}

// 4. 通用发送封装
uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, tx_header, tx_data) != HAL_OK) return 1;
    return 0;
}

/* ------------------------- 中断回调处理 ------------------------- */

// FIFO0 回调 (FDCAN1 和 FDCAN3 默认用这个)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO0);
    }
}

// FIFO1 回调 (FDCAN2 默认用这个)
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO1);
    }
}

// 内部路由函数
static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    // 读取数据
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo_location, &rx_header, rx_data) != HAL_OK) return;

    // 1. 处理标准帧 (原有逻辑，用于 DJI 电机和定位板)
    if (rx_header.IdType == FDCAN_STANDARD_ID)
    {
        uint16_t id = rx_header.Identifier;
        if (id < HASH_TABLE_SIZE && hash_table[id] != HASH_EMPTY_SLOT)
        {
            uint16_t index = hash_table[id];
            if (std_dispatch_table[index].handler != NULL)
            {
                std_dispatch_table[index].handler(std_dispatch_table[index].instance_ptr, &rx_header, rx_data);
            }
        }
    }
}