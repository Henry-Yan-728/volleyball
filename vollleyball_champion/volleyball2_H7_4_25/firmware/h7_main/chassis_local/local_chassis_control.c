#include "local_chassis_control.h"

#include "dji_motor.h"
#include "h7_vesc.h"

#include <math.h>

#define PI_F                              3.1415926535f
#define DEG_TO_RAD_F                      (PI_F / 180.0f)
#define SPEED_RAMP_SCALE                  1000.0f

#define LOCAL_STEERING_GEAR_RATIO         108.0f
#define LOCAL_WHEEL_RADIUS_M              0.023f
#define LOCAL_DRIVE_GEAR_RATIO            1.0f
#define LOCAL_SPEED_DEADBAND_MS           0.05f

#define LOCAL_DRIVE_RAMP_ACCEL_STEP       70
#define LOCAL_DRIVE_RAMP_DECEL_STEP       45
#define LOCAL_DRIVE_RAMP_JERK_STEP        2

#define LOCAL_AUTO_CENTER_CMD_STOP_MS     0.02f
#define LOCAL_AUTO_CENTER_FB_STOP_RPM     80
#define LOCAL_AUTO_CENTER_FB_TIMEOUT_MS   100U

typedef struct {
    int32_t target_val;
    int32_t current_val;
    int32_t accel_step;
    int32_t decel_step;
    int32_t jerk_step;
    int32_t current_step;
} LocalRamp_t;

typedef struct {
    DJI_Motor_Instance *steering_motor;
    H7VescMotor_t *drive_motor;
    LocalRamp_t drive_ramp;
    int32_t ramp_target_scaled;
    int32_t steering_zero_encoder;
    uint8_t steering_zero_valid;
} LocalWheelModule_t;

static LocalWheelModule_t s_wheel_modules[LOCAL_STEERING_WHEEL_COUNT];
static uint8_t s_modules_inited = 0U;

static const uint8_t s_steering_dji_indices[LOCAL_STEERING_WHEEL_COUNT] = {
    DJI_INDEX_CHASSIS_STEER_0,
    DJI_INDEX_CHASSIS_STEER_1,
    DJI_INDEX_CHASSIS_STEER_2,
};

static int32_t LocalRamp_Abs(int32_t value);
static int32_t LocalRamp_PositiveOrOne(int32_t value);
static int32_t LocalRamp_SelectMaxStep(const LocalRamp_t *ramp, int32_t current, int32_t target);
static void LocalRamp_Init(LocalRamp_t *ramp, int32_t accel, int32_t decel, int32_t jerk);
static void LocalRamp_SetTarget(LocalRamp_t *ramp, int32_t target);
static int32_t LocalRamp_Calc(LocalRamp_t *ramp);
static void LocalRamp_EStop(LocalRamp_t *ramp);
static void LocalControl_EnsureInit(void);
static LocalWheelModule_t *LocalControl_GetModule(uint8_t wheel_index);
static void LocalControl_EnsureSteeringZero(LocalWheelModule_t *module);
static float LocalControl_NormalizeAngle(float angle);
static int32_t LocalControl_SpeedToMotorRpm(float speed_ms);
static float LocalControl_RampSpeedMs(LocalWheelModule_t *module, float target_speed_ms);
static float LocalControl_GetCurrentAngleDeg(LocalWheelModule_t *module);
static int32_t LocalControl_AngleDegToEncoder(LocalWheelModule_t *module, float target_angle_deg);
static float LocalControl_CalcDriveScale(float steering_error_deg);
static uint8_t LocalControl_DriveIsNearStop(const LocalWheelModule_t *module, float commanded_speed_ms);

void LocalChassisControl_Init(void)
{
    LocalControl_EnsureInit();
}

static void LocalControl_EnsureInit(void)
{
    if (s_modules_inited != 0U) {
        return;
    }

    for (uint8_t i = 0U; i < LOCAL_STEERING_WHEEL_COUNT; ++i) {
        s_wheel_modules[i].steering_motor = dji_motor_get_instance(s_steering_dji_indices[i]);
        s_wheel_modules[i].drive_motor = H7Vesc_GetInstance(i);
        s_wheel_modules[i].ramp_target_scaled = 0;
        s_wheel_modules[i].steering_zero_encoder = 0;
        s_wheel_modules[i].steering_zero_valid = 0U;
        LocalRamp_Init(&s_wheel_modules[i].drive_ramp,
                       LOCAL_DRIVE_RAMP_ACCEL_STEP,
                       LOCAL_DRIVE_RAMP_DECEL_STEP,
                       LOCAL_DRIVE_RAMP_JERK_STEP);
    }

    s_modules_inited = 1U;
}

static LocalWheelModule_t *LocalControl_GetModule(uint8_t wheel_index)
{
    if (wheel_index >= LOCAL_STEERING_WHEEL_COUNT) {
        return NULL;
    }

    LocalControl_EnsureInit();
    if ((s_wheel_modules[wheel_index].steering_motor == NULL) ||
        (s_wheel_modules[wheel_index].drive_motor == NULL) ||
        (s_wheel_modules[wheel_index].steering_motor->hfdcan == NULL) ||
        (s_wheel_modules[wheel_index].drive_motor->hfdcan == NULL)) {
        return NULL;
    }

    return &s_wheel_modules[wheel_index];
}

static void LocalControl_EnsureSteeringZero(LocalWheelModule_t *module)
{
    if ((module == NULL) || (module->steering_motor == NULL)) {
        return;
    }

    if (module->steering_zero_valid != 0U) {
        return;
    }

    if (module->steering_motor->is_online == 0U) {
        return;
    }

    module->steering_zero_encoder = module->steering_motor->measure.total_angle;
    module->steering_zero_valid = 1U;
}

static float LocalControl_NormalizeAngle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static int32_t LocalControl_SpeedToMotorRpm(float speed_ms)
{
    float wheel_rpm = (speed_ms / (2.0f * PI_F * LOCAL_WHEEL_RADIUS_M)) * 60.0f;
    float motor_rpm = wheel_rpm * LOCAL_DRIVE_GEAR_RATIO;

    return (int32_t)lroundf(motor_rpm);
}

static float LocalControl_RampSpeedMs(LocalWheelModule_t *module, float target_speed_ms)
{
    int32_t smoothed_scaled;

    if (module == NULL) {
        return 0.0f;
    }

    module->ramp_target_scaled = (int32_t)lroundf(target_speed_ms * SPEED_RAMP_SCALE);
    LocalRamp_SetTarget(&module->drive_ramp, module->ramp_target_scaled);
    smoothed_scaled = LocalRamp_Calc(&module->drive_ramp);

    return (float)smoothed_scaled / SPEED_RAMP_SCALE;
}

static float LocalControl_GetCurrentAngleDeg(LocalWheelModule_t *module)
{
    float motor_angle_deg;

    if ((module == NULL) || (module->steering_motor == NULL)) {
        return 0.0f;
    }

    LocalControl_EnsureSteeringZero(module);
    motor_angle_deg = ((float)(module->steering_motor->measure.total_angle -
                               module->steering_zero_encoder) / 8192.0f) * 360.0f;
    return motor_angle_deg / LOCAL_STEERING_GEAR_RATIO;
}

static int32_t LocalControl_AngleDegToEncoder(LocalWheelModule_t *module, float target_angle_deg)
{
    float motor_angle_deg = target_angle_deg * LOCAL_STEERING_GEAR_RATIO;
    int32_t relative_encoder = dji_angle_to_encoder(motor_angle_deg, RATIO_DEFAULT);

    LocalControl_EnsureSteeringZero(module);
    return module->steering_zero_encoder + relative_encoder;
}

static float LocalControl_CalcDriveScale(float steering_error_deg)
{
    float scale = cosf(steering_error_deg * DEG_TO_RAD_F);

    return (scale > 0.0f) ? scale : 0.0f;
}

static uint8_t LocalControl_DriveIsNearStop(const LocalWheelModule_t *module, float commanded_speed_ms)
{
    int32_t feedback_rpm;
    uint32_t feedback_age;

    if ((module == NULL) || (module->drive_motor == NULL)) {
        return 1U;
    }
    if (fabsf(commanded_speed_ms) > LOCAL_AUTO_CENTER_CMD_STOP_MS) {
        return 0U;
    }
    if (module->drive_motor->last_feedback_time == 0U) {
        return 1U;
    }

    feedback_age = HAL_GetTick() - module->drive_motor->last_feedback_time;
    if (feedback_age > LOCAL_AUTO_CENTER_FB_TIMEOUT_MS) {
        return 1U;
    }

    feedback_rpm = H7Vesc_GetFeedbackRpm(module->drive_motor);
    if (feedback_rpm < 0) {
        feedback_rpm = -feedback_rpm;
    }

    return (feedback_rpm <= LOCAL_AUTO_CENTER_FB_STOP_RPM) ? 1U : 0U;
}

void LocalChassisControl_SetWheel(uint8_t wheel_index, float target_speed_ms, float target_angle_deg)
{
    LocalWheelModule_t *module = LocalControl_GetModule(wheel_index);
    float current_angle_deg;
    float current_heading;
    float delta_angle;
    float final_target_deg;
    float drive_target_speed;
    float drive_scale;
    float smoothed_speed;
    int32_t target_encoder;
    int32_t target_rpm;
    int32_t direction = 1;

    if (module == NULL) {
        return;
    }

    current_angle_deg = LocalControl_GetCurrentAngleDeg(module);
    current_heading = LocalControl_NormalizeAngle(fmodf(current_angle_deg, 360.0f));

    delta_angle = LocalControl_NormalizeAngle(target_angle_deg - current_heading);
    if (fabsf(delta_angle) > 95.0f) {
        if (delta_angle > 0.0f) {
            delta_angle -= 180.0f;
        } else {
            delta_angle += 180.0f;
        }
        direction = -1;
    }

    final_target_deg = current_angle_deg + delta_angle;
    target_encoder = LocalControl_AngleDegToEncoder(module, final_target_deg);
    dji_motor_set_location(module->steering_motor, target_encoder);

    if (H7Vesc_IsTxReady() == 0U) {
        LocalRamp_EStop(&module->drive_ramp);
        H7Vesc_SetTargetRpm(module->drive_motor, 0);
        return;
    }

    drive_scale = LocalControl_CalcDriveScale(delta_angle);
    drive_target_speed = target_speed_ms * (float)direction * drive_scale;
    if (fabsf(drive_target_speed) < LOCAL_SPEED_DEADBAND_MS) {
        drive_target_speed = 0.0f;
    }

    smoothed_speed = LocalControl_RampSpeedMs(module, drive_target_speed);
    target_rpm = LocalControl_SpeedToMotorRpm(smoothed_speed);

    H7Vesc_SetTargetRpm(module->drive_motor, target_rpm);
    (void)H7Vesc_SendTarget(module->drive_motor);
}

void LocalChassisControl_AutoCenterWheel(uint8_t wheel_index, float center_angle_deg)
{
    LocalWheelModule_t *module = LocalControl_GetModule(wheel_index);
    float current_angle_deg;
    float current_heading;
    float delta_angle;
    float final_target_deg;
    float smoothed_speed;
    int32_t target_encoder;
    int32_t target_rpm;

    if (module == NULL) {
        return;
    }

    current_angle_deg = LocalControl_GetCurrentAngleDeg(module);
    current_heading = LocalControl_NormalizeAngle(fmodf(current_angle_deg, 360.0f));

    if (H7Vesc_IsTxReady() == 0U) {
        LocalRamp_EStop(&module->drive_ramp);
        H7Vesc_SetTargetRpm(module->drive_motor, 0);
        return;
    }

    smoothed_speed = LocalControl_RampSpeedMs(module, 0.0f);
    target_rpm = LocalControl_SpeedToMotorRpm(smoothed_speed);

    if (LocalControl_DriveIsNearStop(module, smoothed_speed) != 0U) {
        delta_angle = LocalControl_NormalizeAngle(center_angle_deg - current_heading);
        final_target_deg = current_angle_deg + delta_angle;
    } else {
        final_target_deg = current_angle_deg;
    }

    target_encoder = LocalControl_AngleDegToEncoder(module, final_target_deg);
    dji_motor_set_location(module->steering_motor, target_encoder);

    H7Vesc_SetTargetRpm(module->drive_motor, target_rpm);
    (void)H7Vesc_SendTarget(module->drive_motor);
}

static int32_t LocalRamp_Abs(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t LocalRamp_PositiveOrOne(int32_t value)
{
    value = (value >= 0) ? value : -value;
    return (value > 0) ? value : 1;
}

static int32_t LocalRamp_SelectMaxStep(const LocalRamp_t *ramp, int32_t current, int32_t target)
{
    if (current < target) {
        return (current >= 0) ? ramp->accel_step : ramp->decel_step;
    }

    if (current > target) {
        return (current <= 0) ? ramp->accel_step : ramp->decel_step;
    }

    return 0;
}

static void LocalRamp_Init(LocalRamp_t *ramp, int32_t accel, int32_t decel, int32_t jerk)
{
    if (ramp == NULL) {
        return;
    }

    ramp->target_val = 0;
    ramp->current_val = 0;
    ramp->accel_step = LocalRamp_PositiveOrOne(accel);
    ramp->decel_step = LocalRamp_PositiveOrOne(decel);
    ramp->jerk_step = LocalRamp_PositiveOrOne(jerk);
    ramp->current_step = 0;
}

static void LocalRamp_SetTarget(LocalRamp_t *ramp, int32_t target)
{
    if (ramp == NULL) {
        return;
    }

    ramp->target_val = target;
}

static int32_t LocalRamp_Calc(LocalRamp_t *ramp)
{
    int32_t target;
    int32_t current;
    int32_t error;
    int32_t max_step;
    int32_t desired_step;
    int32_t step;
    int32_t jerk;
    uint8_t reversing;

    if (ramp == NULL) {
        return 0;
    }

    target = ramp->target_val;
    current = ramp->current_val;
    step = ramp->current_step;
    error = target - current;
    if (error == 0) {
        ramp->current_step = 0;
        return current;
    }

    max_step = LocalRamp_PositiveOrOne(LocalRamp_SelectMaxStep(ramp, current, target));
    desired_step = (error > 0) ? max_step : -max_step;
    jerk = LocalRamp_PositiveOrOne(ramp->jerk_step);

    reversing = (((step > 0) && (desired_step < 0)) || ((step < 0) && (desired_step > 0))) ? 1U : 0U;
    if (reversing != 0U) {
        step = 0;
    } else if (step < desired_step) {
        step += jerk;
        if (step > desired_step) {
            step = desired_step;
        }
    } else if (step > desired_step) {
        step -= jerk;
        if (step < desired_step) {
            step = desired_step;
        }
    }

    if ((reversing == 0U) && (error > 0) && (step <= 0)) {
        step = 1;
    } else if ((reversing == 0U) && (error < 0) && (step >= 0)) {
        step = -1;
    }

    if (LocalRamp_Abs(error) < LocalRamp_Abs(step)) {
        step = error;
    }

    current += step;
    if (current == target) {
        step = 0;
    }

    ramp->current_step = step;
    ramp->current_val = current;
    return current;
}

static void LocalRamp_EStop(LocalRamp_t *ramp)
{
    if (ramp == NULL) {
        return;
    }

    ramp->target_val = 0;
    ramp->current_val = 0;
    ramp->current_step = 0;
}
