#include "mechanism_task.h"

#include "Pan_Tilt_control.h"
#include "cmsis_os.h"
#include "dji_motor.h"
#include "unitree_motor.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define DEG_TO_RAD_FACTOR (M_PI / 180.0f)

typedef struct
{
    UnitreeBusId_t bus_id;
    uint8_t motor_id;
} UnitreeMotorRoute_t;

static const UnitreeMotorRoute_t kCushionMotors[] = {
    {UNITREE_BUS_USART2, UNITREE_ID_CUSHION_1},
    {UNITREE_BUS_USART2, UNITREE_ID_CUSHION_2},
    {UNITREE_BUS_USART3, UNITREE_ID_CUSHION_3},
};

static const UnitreeMotorRoute_t kServeMotor = {UNITREE_BUS_USART3, UNITREE_ID_SERVE};

Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.1f, 0};

float current_yaw_speed = 0.0f;
float current_pitch_speed = 0.0f;

static float convert_deg_to_unitree_rad(float deg)
{
    return deg * DEG_TO_RAD_FACTOR * UNITREE_REDUCTION_RATIO;
}

static void mechanism_unitree_set_position(const UnitreeMotorRoute_t *motor, float position, float kp, float kd)
{
    UnitreeMotor_SetPositionOnBus(motor->bus_id, motor->motor_id, position, kp, kd);
}

static int mechanism_unitree_receive(const UnitreeMotorRoute_t *motor, MotorData_t *rx_data, uint32_t timeout_ms)
{
    return UnitreeMotor_ReceiveDataOnBus(motor->bus_id, rx_data, timeout_ms);
}

void Mechanism_Init(void)
{
    UnitreeMotor_Init();
    dji_motors_init();
    gimbal_control_init();

    HAL_Delay(100);
    Mechanism_Serve_SetAngle(0.0f);
    HAL_Delay(10);
    Mechanism_Serve_SetAngle(0.0f);
}

void Mechanism_Cushion_SetAngle(float angle_deg, float kp)
{
    float target_rad = convert_deg_to_unitree_rad(angle_deg);
    float kd_default = 0.2f;
    uint32_t i;

    for (i = 0; i < (sizeof(kCushionMotors) / sizeof(kCushionMotors[0])); ++i)
    {
        mechanism_unitree_set_position(&kCushionMotors[i], target_rad, kp, kd_default);
        osDelay(1);
    }
}

void Mechanism_Zero(void)
{
    MotorData_t rx_data;
    uint32_t i;

    for (i = 0; i < (sizeof(kCushionMotors) / sizeof(kCushionMotors[0])); ++i)
    {
        mechanism_unitree_set_position(&kCushionMotors[i], 0.0f, 0.0f, 0.1f);
        (void)mechanism_unitree_receive(&kCushionMotors[i], &rx_data, 2);
    }

    mechanism_unitree_set_position(&kServeMotor, 0.0f, 0.0f, 0.1f);
    (void)mechanism_unitree_receive(&kServeMotor, &rx_data, 2);
}

void Mechanism_Serve_SetAngle(float angle_deg)
{
    float target_val = convert_deg_to_unitree_rad(angle_deg);
    float kp = 1.0f;
    float kd = 0.1f;

    mechanism_unitree_set_position(&kServeMotor, target_val, kp, kd);
}

void Mechanism_Dian_Pitch_SetAngle(float angle_deg)
{
    v_axis.target_angle = angle_deg;
}

void Update_Virtual_Axis(void)
{
    uint32_t now = HAL_GetTick();
    float error;
    DJI_Motor_Instance *m_right;
    DJI_Motor_Instance *m_left;

    if (now - v_axis.last_tick < 1U)
    {
        return;
    }

    v_axis.last_tick = now;
    error = v_axis.target_angle - v_axis.current_angle;

    if (fabsf(error) <= v_axis.velocity_deg_ms)
    {
        v_axis.current_angle = v_axis.target_angle;
    }
    else if (error > 0.0f)
    {
        v_axis.current_angle += v_axis.velocity_deg_ms;
    }
    else
    {
        v_axis.current_angle -= v_axis.velocity_deg_ms;
    }

    m_right = dji_motor_get_instance(0);
    m_left = dji_motor_get_instance(1);

    if ((m_right != NULL) && (m_left != NULL))
    {
        int32_t encoder_pos = dji_angle_to_encoder(v_axis.current_angle, EXTERNAL_MECHANISM_RATIO);

        dji_motor_set_location(m_left, encoder_pos);
        dji_motor_set_location(m_right, -encoder_pos);
    }
}

void Mechanism_Loop_1ms(void)
{
    gimbal_set_speed(current_yaw_speed, current_pitch_speed);
}
