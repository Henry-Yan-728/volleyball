#ifndef FDCAN_BSP_H
#define FDCAN_BSP_H

#include "main.h"
#include "fdcan.h" // 确保包含 cubemx 生成的 fdcan 头文件

// 引入 HAL 库 fdcan 句柄 (CubeMX 生成的)
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief FDCAN分发条目的结构体定义
 */
typedef struct{
    // ID类型: FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
    uint32_t id_type;

    // 监听的 CAN ID
    uint32_t id;

    // 指向与此ID关联的设备实例的指针 (例如电机结构体指针)
    void* instance_ptr;

    /**
     * @brief 回调函数指针
     * @note  注意：rx_data 数组大小已改为 64，以支持定位板的长帧数据
     */
    void (*handler)(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64]);

} FDCAN_Dispatch_t;


/*************************** 公开函数 ***************************/

// 初始化哈希表 (在 main 开始时调用)
void fdcan_bsp_init(void);

// 启动指定的 CAN 总线 (如 &hfdcan1)
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan);

// 注册接收回调 (告诉底层你想听哪个ID)
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan);

// 通用发送函数 (可选封装，也可以直接用 HAL 库)
uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data);

#endif // FDCAN_BSP_H