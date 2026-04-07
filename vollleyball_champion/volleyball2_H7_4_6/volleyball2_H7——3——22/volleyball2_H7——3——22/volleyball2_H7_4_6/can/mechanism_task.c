#include "mechanism_task.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include <math.h>
#include "Pan_Tilt_control.h"
#include "unitree_motor.h"

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define DEG_TO_RAD_FACTOR (M_PI / 180.0f)
#define UNITREE_CUSHION_UART (&huart3)
#define UNITREE_SERVE_UART   (&huart2)

Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.1f, 0};

float current_yaw_speed = 0;
float current_pitch_speed = 0;

static float convert_deg_to_unitree_rad(float deg)
{
    return deg * DEG_TO_RAD_FACTOR * UNITREE_REDUCTION_RATIO;
}

void Mechanism_Init(void)
{
    dji_motors_init();
    gimbal_control_init();
    HAL_Delay(100);
}

void Mechanism_Cushion_SetAngle(float angle_deg, float kp)
{
    float target_rad = convert_deg_to_unitree_rad(angle_deg);
    float kd_default = 0.2f;

    UnitreeMotor_SetPosition(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_1, target_rad, kp, kd_default);
}

void Mechanism_Zero()
{
    MotorData_t rx_data;

    UnitreeMotor_SetPosition(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_1, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(UNITREE_CUSHION_UART, &rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_2, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(UNITREE_CUSHION_UART, &rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_3, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(UNITREE_CUSHION_UART, &rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_SERVE_UART, UNITREE_ID_SERVE, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(UNITREE_SERVE_UART, &rx_data, 2);
}

void Mechanism_Serve_SetAngle(float angle_deg)
{
    float target_val = convert_deg_to_unitree_rad(angle_deg);
    float kp = 2.06f;
    float kd = 0.1f;

    UnitreeMotor_SetPosition(UNITREE_SERVE_UART, UNITREE_ID_SERVE, target_val, kp, kd);
}

void Mechanism_Serve_SetAngle_back(float angle_deg)
{
    float target_val = convert_deg_to_unitree_rad(angle_deg);
    float kp = 0.9f;
    float kd = 0.3f;

    UnitreeMotor_SetPosition(UNITREE_SERVE_UART, UNITREE_ID_SERVE, target_val, kp, kd);
}

void Mechanism_Dian_Pitch_SetAngle(float angle_deg)
{
    v_axis.target_angle = angle_deg;
}

void Update_Virtual_Axis(void)
{
    uint32_t now = HAL_GetTick();
    float error;

    if (now - v_axis.last_tick < 1) {
        return;
    }
    v_axis.last_tick = now;

    error = v_axis.target_angle - v_axis.current_angle;

    if (fabs(error) <= v_axis.velocity_deg_ms) {
        v_axis.current_angle = v_axis.target_angle;
    } else {
        if (error > 0) {
            v_axis.current_angle += v_axis.velocity_deg_ms;
        } else {
            v_axis.current_angle -= v_axis.velocity_deg_ms;
        }
    }

    DJI_Motor_Instance* m_right = dji_motor_get_instance(0);
    DJI_Motor_Instance* m_left  = dji_motor_get_instance(1);

    if (m_right && m_left) {
        int32_t encoder_pos = dji_angle_to_encoder(v_axis.current_angle, EXTERNAL_MECHANISM_RATIO);

        dji_motor_set_location(m_left, encoder_pos);
        dji_motor_set_location(m_right, -encoder_pos);
    }
}

void Mechanism_Loop_1ms(void)
{
    gimbal_set_speed(current_yaw_speed, current_pitch_speed);
}
