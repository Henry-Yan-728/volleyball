#include "h7_vesc_rx.h"

#include "h7_vesc.h"

void H7Vesc_RxMessageHandler(void *instance_ptr,
                             FDCAN_RxHeaderTypeDef *rx_header,
                             uint8_t rx_data[64])
{
    uint32_t ext_id;
    uint8_t packet_id;
    uint8_t controller_id;
    uint32_t raw_feedback;
    H7VescMotor_t *motor;

    (void)instance_ptr;

    if ((rx_header == NULL) || (rx_data == NULL)) {
        return;
    }
    if ((rx_header->IdType != FDCAN_EXTENDED_ID) || (rx_header->DataLength < FDCAN_DLC_BYTES_4)) {
        return;
    }

    ext_id = rx_header->Identifier;
    packet_id = (uint8_t)((ext_id >> 8) & 0xFFU);
    controller_id = (uint8_t)(ext_id & 0xFFU);

    if (packet_id != H7_VESC_STATUS_PACKET_ID) {
        return;
    }

    motor = H7Vesc_GetInstanceByCan(&hfdcan1, controller_id);
    if (motor == NULL) {
        motor = H7Vesc_GetInstanceByCan(&hfdcan2, controller_id);
    }
    if (motor == NULL) {
        motor = H7Vesc_GetInstanceByCan(&hfdcan3, controller_id);
    }
    if (motor == NULL) {
        return;
    }

    raw_feedback = ((uint32_t)rx_data[0] << 24) |
                   ((uint32_t)rx_data[1] << 16) |
                   ((uint32_t)rx_data[2] << 8) |
                   (uint32_t)rx_data[3];
    H7Vesc_UpdateFeedback(motor, (int32_t)raw_feedback);
}
