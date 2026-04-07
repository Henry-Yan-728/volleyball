#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "main.h"
#include "pid.h"

#define DJI_MOTOR_COUNT 8U

#define RATIO_M3508 19.2032f
#define RATIO_M2006 36.0f
#define RATIO_GM6020 1.0f

#define DJI_ENCODER_RESOLUTION 8192.0f

/* User-required mapping:
 * ID1,2 -> 3508
 * ID3,4,5 -> 2006 steering motors
 * ID6 -> 6020
 */
#define GIMBAL_MOTOR_YAW_ID   5U   /* physical ID6 */
#define GIMBAL_MOTOR_PITCH_ID 0xFFU /* no dedicated DJI pitch in this mapping */

typedef enum {
    CAN_3508_2006_M1_ID = 0x201,
    CAN_3508_2006_M2_ID = 0x202,
    CAN_3508_2006_M3_ID = 0x203,
    CAN_3508_2006_M4_ID = 0x204,
    CAN_3508_2006_M5_ID = 0x205,
    CAN_3508_2006_M6_ID = 0x206,
    CAN_3508_2006_M7_ID = 0x207,
    CAN_3508_2006_M8_ID = 0x208,
} can_msg_id_e;

typedef enum {
    MOTOR_MODE_SPEED = 0,
    MOTOR_MODE_POSITION = 1,
} DJI_Motor_Mode_e;

typedef struct {
    uint16_t angle;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_angle;
    int32_t total_angle;
} motor_measure_t;

typedef struct DJI_Motor_Instance_t {
    motor_measure_t measure;

    int32_t target_loc;
    int16_t target_speed;
    DJI_Motor_Mode_e control_mode;

    pid_incremental_struct loc_pid;
    pid_incremental_struct spd_pid;

    uint16_t can_id;
    uint8_t is_online;
    uint32_t last_msg_time;

    FDCAN_HandleTypeDef *hfdcan;
} DJI_Motor_Instance;

int32_t dji_angle_to_encoder(float angle_deg, float reduction_ratio);
float dji_encoder_to_angle(int32_t encoder_val, float reduction_ratio);
int32_t dji_degree2encoder(float degree);

void dji_motors_init(void);
void dji_motor_set_location(DJI_Motor_Instance *motor, int32_t location);
void dji_motor_set_speed(DJI_Motor_Instance *motor, int16_t speed);
DJI_Motor_Instance *dji_motor_get_instance(uint8_t motor_index);
void dji_motor_stop_all(void);
void dji_motor_resume_all(void);

void dji_motor_message_handler(void *instance, FDCAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8]);

#endif /* DJI_MOTOR_H */
