#include "local_kinematics.h"

#include <math.h>

#define PI_F             3.1415926f
#define SIN_60_DEG_F     0.8660254f

static float LocalKinematics_NormalizeAngle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float LocalKinematics_CalcSteeringAngle(float x, float y)
{
    float theta;
    float angle_deg;

    if ((fabsf(x) < 0.001f) && (fabsf(y) < 0.001f)) {
        return 0.0f;
    }

    theta = atan2f(y, x);
    angle_deg = 90.0f - theta * 180.0f / PI_F;

    return LocalKinematics_NormalizeAngle(angle_deg);
}

void LocalChassisKinematics_Solve(const LocalChassisConfig_t *config,
                                  float vx,
                                  float vy,
                                  float wz,
                                  LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT])
{
    float wz_r;
    float wheel_vx[LOCAL_CHASSIS_WHEEL_COUNT];
    float wheel_vy[LOCAL_CHASSIS_WHEEL_COUNT];

    if ((config == NULL) || (wheel_commands == NULL)) {
        return;
    }

    wz_r = wz * config->radius;

    wheel_vx[0] = vx - wz_r;
    wheel_vy[0] = vy;

    wheel_vx[1] = vx + 0.5f * wz_r;
    wheel_vy[1] = vy + SIN_60_DEG_F * wz_r;

    wheel_vx[2] = vx + 0.5f * wz_r;
    wheel_vy[2] = vy - SIN_60_DEG_F * wz_r;

    for (uint32_t i = 0U; i < LOCAL_CHASSIS_WHEEL_COUNT; ++i) {
        wheel_commands[i].drive_speed_ms =
            sqrtf(wheel_vx[i] * wheel_vx[i] + wheel_vy[i] * wheel_vy[i]);
        wheel_commands[i].steering_angle_deg =
            LocalKinematics_NormalizeAngle(-LocalKinematics_CalcSteeringAngle(wheel_vx[i], wheel_vy[i]) +
                                           config->steering_offset_deg[i]);
    }
}

void LocalChassisKinematics_DesaturateWheelSpeeds(
    LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT],
    float max_wheel_speed_ms)
{
    float max_abs_speed = 0.0f;
    float scale;

    if ((wheel_commands == NULL) || (max_wheel_speed_ms <= 0.0f)) {
        return;
    }

    for (uint32_t i = 0U; i < LOCAL_CHASSIS_WHEEL_COUNT; ++i) {
        float abs_speed = fabsf(wheel_commands[i].drive_speed_ms);
        if (abs_speed > max_abs_speed) {
            max_abs_speed = abs_speed;
        }
    }

    if ((max_abs_speed <= max_wheel_speed_ms) || (max_abs_speed < 0.001f)) {
        return;
    }

    scale = max_wheel_speed_ms / max_abs_speed;
    for (uint32_t i = 0U; i < LOCAL_CHASSIS_WHEEL_COUNT; ++i) {
        wheel_commands[i].drive_speed_ms *= scale;
    }
}
