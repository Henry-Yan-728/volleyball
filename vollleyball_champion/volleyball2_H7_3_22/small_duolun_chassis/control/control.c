#include "control.h"
#include "dji_motor.h"
#include "motor_ramp.h"
#include "vesc.h"

#include <math.h>

#define PI 3.1415926535f
#define SPEED_RAMP_SCALE 1000.0f

static RampController_t drive_ramps[STEERING_WHEEL_COUNT];
static uint8_t ramps_inited = 0;

static void control_ensure_init(void)
{
    if (ramps_inited) return;

    for (uint8_t i = 0; i < STEERING_WHEEL_COUNT; ++i) {
        Motor_Ramp_Init(&drive_ramps[i], DRIVE_RAMP_ACCEL_STEP, DRIVE_RAMP_DECEL_STEP);
    }

    ramps_inited = 1;
}

void control_init(void)
{
    control_ensure_init();
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int32_t Speed_to_MotorRPM(float speed_ms)
{
    float wheel_rpm = (speed_ms / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;
    return (int32_t)motor_rpm;
}

static float Ramp_Speed_ms(uint8_t wheel_index, float target_speed_ms)
{
    int32_t target_scaled = (int32_t)(target_speed_ms * SPEED_RAMP_SCALE);

    Motor_Ramp_SetTarget(&drive_ramps[wheel_index], target_scaled);
    int32_t smoothed_scaled = Motor_Ramp_Calc(&drive_ramps[wheel_index]);

    return ((float)smoothed_scaled / SPEED_RAMP_SCALE);
}

void Steering_Wheel_Control(uint8_t wheel_index, float target_spd, float target_ang)
{
    if (wheel_index >= STEERING_WHEEL_COUNT) return;
    control_ensure_init();

    DJI_Motor_Instance *steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;

    uint8_t drive_enable = (fabsf(target_spd) >= SPEED_DEADBAND);

    double current_angle_deg =
        (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    float current_heading = fmodf((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(target_ang - current_heading);
    int direction = 1;

    if (fabsf(delta_angle) > 95.0f) {
        if (delta_angle > 0) delta_angle -= 180.0f;
        else                 delta_angle += 180.0f;
        direction = -1;
    }

    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);
    dji_motor_set_location(steering_motor, target_encoder);

    float drive_target_spd = drive_enable ? (target_spd * (float)direction) : 0.0f;
    float smoothed_spd = Ramp_Speed_ms(wheel_index, drive_target_spd);
    int32_t target_cmd = Speed_to_MotorRPM(smoothed_spd);

    Change_vesc_speed(wheel_index, target_cmd);
    Com2vesc(wheel_index);
}

void Steering_Wheel_AutoCenter(uint8_t wheel_index, float center_angle_deg)
{
    if (wheel_index >= STEERING_WHEEL_COUNT) return;
    control_ensure_init();

    DJI_Motor_Instance *steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;

    double current_angle_deg =
        (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    float current_heading = fmodf((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(center_angle_deg - current_heading);
    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);
    dji_motor_set_location(steering_motor, target_encoder);

    float smoothed_spd = Ramp_Speed_ms(wheel_index, 0.0f);
    int32_t target_cmd = Speed_to_MotorRPM(smoothed_spd);

    Change_vesc_speed(wheel_index, target_cmd);
    Com2vesc(wheel_index);
}
