#include "Pan_Tilt_control.h"

#include "chassis_cybergear.h"
#include "cybergear_speed_planner.h"
#include "dji_motor.h"

#define GIMBAL_YAW_HOME_ENABLE         1U
#define GIMBAL_YAW_HOME_TARGET_DEG     -113.0f
#define GIMBAL_YAW_HOME_TOLERANCE_DEG  1.0f
#define GIMBAL_YAW_HOME_TIMEOUT_MS     3000U

typedef enum
{
    GIMBAL_YAW_HOME_WAIT_ONLINE = 0,
    GIMBAL_YAW_HOME_RUNNING,
    GIMBAL_YAW_HOME_DONE
} GimbalYawHomeState_e;

static GimbalYawHomeState_e s_gimbal_yaw_home_state = GIMBAL_YAW_HOME_WAIT_ONLINE;
static uint32_t s_gimbal_yaw_home_start_tick = 0U;

volatile uint32_t g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_WAIT_ONLINE;
volatile int32_t g_gimbal_yaw_home_error = 0;

static uint8_t prv_gimbal_yaw_home_update(DJI_Motor_Instance* yaw_motor)
{
#if (GIMBAL_YAW_HOME_ENABLE == 1U)
    int32_t target_encoder;
    int32_t tolerance_encoder;
    int32_t error_encoder;

    if (yaw_motor == NULL)
    {
        return 1U;
    }

    if (yaw_motor->is_online == 0U)
    {
        s_gimbal_yaw_home_state = GIMBAL_YAW_HOME_WAIT_ONLINE;
        g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_WAIT_ONLINE;
        g_gimbal_yaw_home_error = 0;
        dji_motor_set_speed(yaw_motor, 0);
        return 1U;
    }

    if (s_gimbal_yaw_home_state == GIMBAL_YAW_HOME_WAIT_ONLINE)
    {
        s_gimbal_yaw_home_state = GIMBAL_YAW_HOME_RUNNING;
        s_gimbal_yaw_home_start_tick = HAL_GetTick();
    }

    target_encoder = dji_angle_to_encoder(GIMBAL_YAW_HOME_TARGET_DEG, RATIO_GM6020);
    tolerance_encoder = dji_angle_to_encoder(GIMBAL_YAW_HOME_TOLERANCE_DEG, RATIO_GM6020);
    if (tolerance_encoder < 10)
    {
        tolerance_encoder = 10;
    }

    error_encoder = target_encoder - yaw_motor->measure.total_angle;
    g_gimbal_yaw_home_error = error_encoder;

    if (s_gimbal_yaw_home_state == GIMBAL_YAW_HOME_RUNNING)
    {
        g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_RUNNING;
        dji_motor_set_location(yaw_motor, target_encoder);

        if (((error_encoder <= tolerance_encoder) && (error_encoder >= -tolerance_encoder)) ||
            ((HAL_GetTick() - s_gimbal_yaw_home_start_tick) >= GIMBAL_YAW_HOME_TIMEOUT_MS))
        {
            s_gimbal_yaw_home_state = GIMBAL_YAW_HOME_DONE;
            g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_DONE;
            dji_motor_set_speed(yaw_motor, 0);
            return 0U;
        }

        return 1U;
    }

    g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_DONE;
    return 0U;
#else
    (void)yaw_motor;
    g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_DONE;
    g_gimbal_yaw_home_error = 0;
    return 0U;
#endif
}

void gimbal_control_init(void)
{
    s_gimbal_yaw_home_state = GIMBAL_YAW_HOME_WAIT_ONLINE;
    s_gimbal_yaw_home_start_tick = 0U;
    g_gimbal_yaw_home_state = GIMBAL_YAW_HOME_WAIT_ONLINE;
    g_gimbal_yaw_home_error = 0;

    gimbal_set_speed(0, 0);
}

void gimbal_set_speed(float yaw_speed_cmd, float pitch_target_deg)
{
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);

    if (yaw_motor != NULL)
    {
        if (prv_gimbal_yaw_home_update(yaw_motor) == 0U)
        {
            dji_motor_set_speed(yaw_motor, (int16_t)yaw_speed_cmd);
        }
    }

    chassis_control(pitch_target_deg);
}

void gimbal_get_angles(float* yaw_angle_deg, float* pitch_angle_deg)
{
    DJI_Motor_Instance* yaw_motor = dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID);

    if ((yaw_angle_deg != NULL) && (yaw_motor != NULL))
    {
        *yaw_angle_deg = dji_encoder_to_angle(yaw_motor->measure.total_angle, RATIO_GM6020);
    }

    if (pitch_angle_deg != NULL)
    {
        *pitch_angle_deg = chassis_get_angle_rad() * (180.0f / (float)M_PI);
    }
}
