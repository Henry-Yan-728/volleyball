#ifndef FDCAN_BSP_H
#define FDCAN_BSP_H

#include "main.h"
#include "fdcan.h" // ȷ������ cubemx ���ɵ� fdcan ͷ�ļ�

// ���� HAL �� fdcan ��� (CubeMX ���ɵ�)
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief FDCAN�ַ���Ŀ�Ľṹ�嶨��
 */
typedef struct{
    // ID类型: FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
    uint32_t id_type;

    // 要匹配的 CAN ID
    uint32_t id;

    // 掩码：仅对扩展帧有效，路由时执行 (rx_id & mask) == (id & mask)
    // 标准帧不使用此字段（精确匹配）
    uint32_t mask;

    // 指向该ID对应的设备实例的指针 (通常是结构体指针)
    void* instance_ptr;

    /**
     * @brief 回调函数指针
     * @note  注意：rx_data 缓冲区大小已改为 64，支持多位数据长度的长帧格式
     */
    void (*handler)(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[64]);

} FDCAN_Dispatch_t;


/*************************** �������� ***************************/

// ��ʼ����ϣ�� (�� main ��ʼʱ����)
void fdcan_bsp_init(void);

// ����ָ���� CAN ���� (�� &hfdcan1)
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan);

// ע����ջص� (���ߵײ��������ĸ�ID)
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan);

// ͨ�÷��ͺ��� (��ѡ��װ��Ҳ����ֱ���� HAL ��)
uint8_t fdcan_bsp_send(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* tx_header, uint8_t* tx_data);

#endif // FDCAN_BSP_H