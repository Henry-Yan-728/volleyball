#ifndef LOCAL_SAFETY_GUARDIAN_H
#define LOCAL_SAFETY_GUARDIAN_H

#include "main.h"

typedef struct {
    uint32_t command_timeout_ms;
    float auto_center_linear_threshold;
    float auto_center_angular_threshold;
    float auto_center_angle_deg;
} LocalSafetyGuardianConfig_t;

typedef struct {
    LocalSafetyGuardianConfig_t config;
    uint32_t last_cmd_time;
} LocalSafetyGuardian_t;

typedef struct {
    float vx;
    float vy;
    float wz;
    uint8_t request_auto_center;
    uint8_t timed_out;
} LocalSafetyGuardianOutput_t;

void LocalSafetyGuardian_Init(LocalSafetyGuardian_t *guardian,
                              const LocalSafetyGuardianConfig_t *config,
                              uint32_t current_time);
void LocalSafetyGuardian_UpdateHeartbeat(LocalSafetyGuardian_t *guardian, uint32_t current_time);
void LocalSafetyGuardian_Evaluate(const LocalSafetyGuardian_t *guardian,
                                  uint32_t current_time,
                                  float vx,
                                  float vy,
                                  float wz,
                                  LocalSafetyGuardianOutput_t *output);

#endif /* LOCAL_SAFETY_GUARDIAN_H */
