#ifndef LOCAL_KINEMATICS_H
#define LOCAL_KINEMATICS_H

#include "main.h"

#define LOCAL_CHASSIS_WHEEL_COUNT 3U

typedef struct {
    float radius;
    float steering_offset_deg[LOCAL_CHASSIS_WHEEL_COUNT];
} LocalChassisConfig_t;

typedef struct {
    float drive_speed_ms;
    float steering_angle_deg;
} LocalChassisWheelCommand_t;

void LocalChassisKinematics_Solve(const LocalChassisConfig_t *config,
                                  float vx,
                                  float vy,
                                  float wz,
                                  LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT]);

void LocalChassisKinematics_DesaturateWheelSpeeds(
    LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT],
    float max_wheel_speed_ms);

#endif /* LOCAL_KINEMATICS_H */
