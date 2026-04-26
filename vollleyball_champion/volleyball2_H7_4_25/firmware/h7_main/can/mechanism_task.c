#include "mechanism_task.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdio.h>
#include "Pan_Tilt_control.h"
#include "unitree_motor.h"
#include "chassis_task.h"

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define DEG_TO_RAD_FACTOR (M_PI / 180.0f)
#define UNITREE_CUSHION_UART (&huart3)
#define UNITREE_SERVE_UART   (&huart2)
#define MECH_CUSHION_READY_DEG   60.0f
#define MECH_CUSHION_ACTION_DEG 120.0f
#define MECH_SERVE_READY_DEG     30.0f
#define MECH_SERVE_ACTION_DEG   210.0f
#define MECH_SERVE_INIT_DELAY_MS 500U
#define MECH_SERVE_INIT_RETRY_MS 200U
#define MECH_SERVE_STEP1_MS      100U
#define MECH_SERVE_FIRE_MS       600U
#define MECH_SERVE_STOP_MS       750U
#define MECH_SERVE_RESET_MS     1350U
#define MECH_SERVE_CHASSIS_FEED_MM_S 8500.0f
#define MECH_BALL_HOLD_MS        850U
#define MECH_BALL_DEBOUNCE_MS      5U
#define MECH_BALL_RELEASE_MS     200U
#define MECH_BALL_RELEASE_POLL_MS   2U
#define MECH_BALL_RELEASE_TIMEOUT_MS 3000U
#define MECH_CUSHION_RESET_KP    0.3f
#define MECH_MOTOR_MAX_RETRY_COUNT 3U

typedef enum {
    SERVE_CTRL_IDLE = 0U,
    SERVE_CTRL_STEP1,
    SERVE_CTRL_STEP2,
    SERVE_CTRL_STEP3
} ServeControllerState_e;

Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.1f, 0};

volatile float current_yaw_speed = 0;
volatile float current_pitch_target_deg = 0;
static volatile PC_state s_system_doit = START;

static uint8_t mechanism_is_error_state(PC_state state)
{
    return (uint8_t)((state == SYS_ERROR_SENSOR_JAM) || (state == SYS_ERROR_MOTOR_COMMS));
}

static void mechanism_serve_hard_stop_outputs(void)
{
    Chassis_Stop();
    (void)UnitreeMotor_StopNoWait(UNITREE_SERVE_UART, UNITREE_ID_SERVE);
    (void)UnitreeMotor_StopNoWait(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_1);
}

static void mechanism_serve_reset_controller(MechanismServeController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->request_pending = 0U;
    controller->action_state = SERVE_CTRL_IDLE;
    controller->retry_count = 0U;
    controller->action_tick = 0U;
}

static void mechanism_record_motor_failure(MechanismServeController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->retry_count++;
    if (controller->retry_count >= MECH_MOTOR_MAX_RETRY_COUNT) {
        mechanism_serve_reset_controller(controller);
        mechanism_serve_hard_stop_outputs();
        Set_System_Doit_State(SYS_ERROR_MOTOR_COMMS);
    }
}

void Set_System_Doit_State(PC_state new_state)
{
    taskENTER_CRITICAL();
    s_system_doit = new_state;
    taskEXIT_CRITICAL();
}

PC_state Get_System_Doit_State(void)
{
    PC_state current_state;

    taskENTER_CRITICAL();
    current_state = s_system_doit;
    taskEXIT_CRITICAL();

    return current_state;
}

static float convert_deg_to_unitree_rad(float deg)
{
    return deg * DEG_TO_RAD_FACTOR * UNITREE_REDUCTION_RATIO;
}

void Mechanism_Init(void)
{
    dji_motors_init();
    gimbal_control_init();
    UnitreeMotor_Init();
}

UnitreeMotorLinkResult_t Mechanism_Cushion_SetAngle(float angle_deg, float kp)
{
    float target_rad = convert_deg_to_unitree_rad(angle_deg);
    float kd_default = 0.17f;

    return UnitreeMotor_SetPosition(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_1, target_rad, kp, kd_default);
}

void Mechanism_Zero()
{
    MotorData_t rx_data;

    UnitreeMotor_SetPosition(UNITREE_CUSHION_UART, UNITREE_ID_CUSHION_1, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(UNITREE_CUSHION_UART, &rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_SERVE_UART, UNITREE_ID_SERVE, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(UNITREE_SERVE_UART, &rx_data, 2);
}

UnitreeMotorLinkResult_t Mechanism_Serve_SetAngle(float angle_deg)
{
    float target_val = convert_deg_to_unitree_rad(angle_deg);
    float kp = 3.57f;
    float kd = 0.12f;

    return UnitreeMotor_SetPosition(UNITREE_SERVE_UART, UNITREE_ID_SERVE, target_val, kp, kd);
}

UnitreeMotorLinkResult_t Mechanism_Serve_SetAngle_back(float angle_deg)
{
    float target_val = convert_deg_to_unitree_rad(angle_deg);
    float kp = 0.42f;
    float kd = 0.3f;

    return UnitreeMotor_SetPosition(UNITREE_SERVE_UART, UNITREE_ID_SERVE, target_val, kp, kd);
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

    DJI_Motor_Instance* m_right = dji_motor_get_instance(DJI_INDEX_AXIS_RIGHT);
    DJI_Motor_Instance* m_left  = dji_motor_get_instance(DJI_INDEX_AXIS_LEFT);

    if (m_right && m_left) {
        int32_t encoder_pos = dji_angle_to_encoder(v_axis.current_angle, EXTERNAL_MECHANISM_RATIO);

        dji_motor_set_location(m_left, encoder_pos+45.0f);
        dji_motor_set_location(m_right, -encoder_pos);
    }
}

void Mechanism_Loop_1ms(void)
{
    gimbal_set_speed(current_yaw_speed, current_pitch_target_deg);
    DJI_Motor_Control_Loop_1ms();
}

void Mechanism_ServeController_Init(MechanismServeController_t *controller, uint32_t now_tick)
{
    if (controller == NULL) {
        return;
    }

    controller->init_state = 0U;
    controller->request_pending = 0U;
    controller->action_state = SERVE_CTRL_IDLE;
    controller->retry_count = 0U;
    controller->init_tick = now_tick;
    controller->action_tick = 0U;
}

void Mechanism_ServeController_Request(MechanismServeController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    if ((Get_System_Doit_State() != START) ||
        (controller->action_state != SERVE_CTRL_IDLE) ||
        (controller->init_state < 2U)) {
        return;
    }

    controller->request_pending = 1U;
}

void Mechanism_ServeController_Cancel(MechanismServeController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    mechanism_serve_reset_controller(controller);
    mechanism_serve_hard_stop_outputs();
}

uint8_t Mechanism_ServeController_IsReady(const MechanismServeController_t *controller)
{
    return (uint8_t)((controller != NULL) && (controller->init_state >= 2U));
}

uint8_t Mechanism_ServeController_IsIdle(const MechanismServeController_t *controller)
{
    return (uint8_t)((controller != NULL) && (controller->action_state == SERVE_CTRL_IDLE));
}

void Mechanism_ServeController_Process(MechanismServeController_t *controller, uint32_t now_tick,
                                       float cushion_kp)
{
    PC_state current_state;

    if (controller == NULL) {
        return;
    }

    current_state = Get_System_Doit_State();
    if (mechanism_is_error_state(current_state) != 0U) {
        Mechanism_ServeController_Cancel(controller);
        return;
    }

    if (controller->action_state > SERVE_CTRL_STEP3) {
        Mechanism_ServeController_Cancel(controller);
        return;
    }

    if (controller->init_state == 0U) {
        if (now_tick - controller->init_tick >= MECH_SERVE_INIT_DELAY_MS) {
            if (Mechanism_Serve_SetAngle(MECH_SERVE_READY_DEG) == UNITREE_MOTOR_LINK_OK) {
                controller->init_tick = now_tick;
                controller->init_state = 1U;
                controller->retry_count = 0U;
            } else {
                mechanism_record_motor_failure(controller);
            }
        }
    } else if (controller->init_state == 1U) {
        if (now_tick - controller->init_tick >= MECH_SERVE_INIT_RETRY_MS) {
            if (Mechanism_Serve_SetAngle(MECH_SERVE_READY_DEG) == UNITREE_MOTOR_LINK_OK) {
                controller->init_state = 2U;
                controller->retry_count = 0U;
            } else {
                mechanism_record_motor_failure(controller);
            }
        }
    }

    if ((controller->action_state == SERVE_CTRL_IDLE) &&
        (controller->request_pending != 0U) &&
        (current_state != START)) {
        controller->request_pending = 0U;
    }

    if ((controller->action_state == SERVE_CTRL_IDLE) &&
        (controller->request_pending != 0U) &&
        (current_state == START) &&
        (controller->init_state >= 2U)) {
        if (Mechanism_Cushion_SetAngle(MECH_CUSHION_ACTION_DEG, cushion_kp) == UNITREE_MOTOR_LINK_OK) {
            controller->request_pending = 0U;
            controller->action_state = SERVE_CTRL_STEP1;
            controller->action_tick = now_tick;
            controller->retry_count = 0U;
            Set_System_Doit_State(START_BUSY);
        } else {
            mechanism_record_motor_failure(controller);
        }
    }

    switch ((ServeControllerState_e)controller->action_state) {
        case SERVE_CTRL_STEP1:
            if (now_tick - controller->action_tick > MECH_SERVE_STEP1_MS) {
                controller->action_state = SERVE_CTRL_STEP2;
                controller->retry_count = 0U;
            }
            break;

        case SERVE_CTRL_STEP2:
            /* Feed the chassis slowly during serve; linear command uses mm/s at this boundary. */
            Chassis_Update(0.0f, MECH_SERVE_CHASSIS_FEED_MM_S, 0.0f);
            if (now_tick - controller->action_tick > MECH_SERVE_FIRE_MS) {
                if (Mechanism_Serve_SetAngle(MECH_SERVE_ACTION_DEG) == UNITREE_MOTOR_LINK_OK) {
                    controller->action_state = SERVE_CTRL_STEP3;
                    controller->retry_count = 0U;
                } else {
                    mechanism_record_motor_failure(controller);
                }
            }
            break;

        case SERVE_CTRL_STEP3:
            if (now_tick - controller->action_tick > MECH_SERVE_STOP_MS) {
                Chassis_Update(0.0f, 0.0f, 0.0f);
            }
            (void)Mechanism_Cushion_SetAngle(MECH_CUSHION_READY_DEG, MECH_CUSHION_RESET_KP);
            if (now_tick - controller->action_tick > MECH_SERVE_RESET_MS) {
                if (Mechanism_Serve_SetAngle_back(MECH_SERVE_READY_DEG) == UNITREE_MOTOR_LINK_OK) {
                    controller->action_state = SERVE_CTRL_IDLE;
                    controller->retry_count = 0U;
                    if (Get_System_Doit_State() == START_BUSY) {
                        Set_System_Doit_State(START);
                    }
                } else {
                    mechanism_record_motor_failure(controller);
                }
            }
            break;

        case SERVE_CTRL_IDLE:
        default:
            break;
    }
}

void Mechanism_BallDetect_Process(float cushion_kp)
{
    static uint8_t s_ball_retry_count = 0U;
    static uint8_t s_ball_reset_sent = 0U;
    static uint32_t s_ball_trigger_tick = 0U;
    static uint32_t s_ball_busy_tick = 0U;
    static uint32_t s_ball_release_tick = 0U;
    static uint32_t s_ball_release_wait_tick = 0U;
    PC_state current_state = Get_System_Doit_State();
    GPIO_PinState sensor_state = HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin);
    uint32_t now_tick = HAL_GetTick();

    if ((current_state != OVER) && (current_state != OVER_BUSY)) {
        s_ball_retry_count = 0U;
        s_ball_reset_sent = 0U;
        s_ball_trigger_tick = 0U;
        s_ball_busy_tick = 0U;
        s_ball_release_tick = 0U;
        s_ball_release_wait_tick = 0U;
        return;
    }

    if (current_state == OVER) {
        s_ball_reset_sent = 0U;
        s_ball_busy_tick = 0U;
        s_ball_release_tick = 0U;
        s_ball_release_wait_tick = 0U;

        if (sensor_state != GPIO_PIN_RESET) {
            s_ball_trigger_tick = 0U;
            return;
        }

        if (s_ball_trigger_tick == 0U) {
            s_ball_trigger_tick = now_tick;
            return;
        }

        if ((now_tick - s_ball_trigger_tick) < MECH_BALL_DEBOUNCE_MS) {
            return;
        }

        if (Mechanism_Cushion_SetAngle(MECH_CUSHION_ACTION_DEG, cushion_kp) == UNITREE_MOTOR_LINK_OK) {
            s_ball_retry_count = 0U;
            s_ball_reset_sent = 0U;
            s_ball_busy_tick = now_tick;
            s_ball_trigger_tick = 0U;
            printf("Ball detected! Action!\r\n");
            Set_System_Doit_State(OVER_BUSY);
        } else {
            s_ball_retry_count++;
            if (s_ball_retry_count >= MECH_MOTOR_MAX_RETRY_COUNT) {
                s_ball_retry_count = 0U;
                s_ball_trigger_tick = 0U;
                Set_System_Doit_State(SYS_ERROR_MOTOR_COMMS);
            }
        }
        return;
    }

    if ((s_ball_busy_tick == 0U) || (s_ball_busy_tick > now_tick)) {
        s_ball_busy_tick = now_tick;
    }

    if (((now_tick - s_ball_busy_tick) >= MECH_BALL_HOLD_MS) && (s_ball_reset_sent == 0U)) {
        if (Mechanism_Cushion_SetAngle(MECH_CUSHION_READY_DEG, MECH_CUSHION_RESET_KP) ==
            UNITREE_MOTOR_LINK_OK) {
            s_ball_reset_sent = 1U;
            s_ball_retry_count = 0U;
            s_ball_release_wait_tick = now_tick;
        } else {
            s_ball_retry_count++;
            if (s_ball_retry_count >= MECH_MOTOR_MAX_RETRY_COUNT) {
                s_ball_retry_count = 0U;
                Set_System_Doit_State(SYS_ERROR_MOTOR_COMMS);
            }
        }
        return;
    }

    if (s_ball_reset_sent == 0U) {
        return;
    }

    if (sensor_state == GPIO_PIN_RESET) {
        s_ball_release_tick = 0U;
        if ((now_tick - s_ball_release_wait_tick) > MECH_BALL_RELEASE_TIMEOUT_MS) {
            printf("Ball detect timeout!\r\n");
            Set_System_Doit_State(SYS_ERROR_SENSOR_JAM);
        } else {
            osDelay(MECH_BALL_RELEASE_POLL_MS);
        }
        return;
    }

    if (s_ball_release_tick == 0U) {
        s_ball_release_tick = now_tick;
        return;
    }

    if ((now_tick - s_ball_release_tick) < MECH_BALL_RELEASE_MS) {
        return;
    }

    s_ball_retry_count = 0U;
    s_ball_reset_sent = 0U;
    s_ball_busy_tick = 0U;
    s_ball_release_tick = 0U;
    s_ball_release_wait_tick = 0U;
    Set_System_Doit_State(OVER);
    printf("Ready.\r\n");
}
