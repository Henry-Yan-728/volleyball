#ifndef H7_VESC_H
#define H7_VESC_H

#include "main.h"

#define H7_VESC_MOTOR_COUNT       4U
#define H7_VESC_STATUS_PACKET_ID  9U
#define H7_VESC_STARTUP_DELAY_MS  5000U

typedef enum {
    H7_CAN_PACKET_SET_DUTY = 0,
    H7_CAN_PACKET_SET_CURRENT,
    H7_CAN_PACKET_SET_CURRENT_BRAKE,
    H7_CAN_PACKET_SET_RPM,
    H7_CAN_PACKET_SET_POS,
} H7VescCanPacketId_t;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t controller_id;
    uint8_t motor_poles;
    int32_t target_rpm;
    int32_t last_feedback_erpm;
    int32_t last_feedback_rpm;
    uint32_t last_feedback_time;
    uint8_t tx_data[4];
} H7VescMotor_t;

void H7Vesc_Init(void);
H7VescMotor_t *H7Vesc_GetInstance(uint8_t motor_index);
H7VescMotor_t *H7Vesc_GetInstanceByCan(FDCAN_HandleTypeDef *hfdcan, uint8_t controller_id);
void H7Vesc_SetTargetRpm(H7VescMotor_t *motor, int32_t target_rpm);
uint8_t H7Vesc_SendTarget(H7VescMotor_t *motor);
uint8_t H7Vesc_IsTxReady(void);
void H7Vesc_UpdateFeedback(H7VescMotor_t *motor, int32_t feedback_erpm);
int32_t H7Vesc_GetFeedbackRpm(const H7VescMotor_t *motor);

#endif /* H7_VESC_H */
