#include "local_safety_guardian.h"

#include <math.h>
#include <string.h>

static uint8_t LocalSafetyGuardian_ShouldAutoCenter(const LocalSafetyGuardian_t *guardian,
                                                    float vx,
                                                    float vy,
                                                    float wz)
{
    return ((fabsf(vx) < guardian->config.auto_center_linear_threshold) &&
            (fabsf(vy) < guardian->config.auto_center_linear_threshold) &&
            (fabsf(wz) < guardian->config.auto_center_angular_threshold)) ? 1U : 0U;
}

void LocalSafetyGuardian_Init(LocalSafetyGuardian_t *guardian,
                              const LocalSafetyGuardianConfig_t *config,
                              uint32_t current_time)
{
    if ((guardian == NULL) || (config == NULL)) {
        return;
    }

    memset(guardian, 0, sizeof(*guardian));
    guardian->config = *config;
    guardian->last_cmd_time = current_time;
}

void LocalSafetyGuardian_UpdateHeartbeat(LocalSafetyGuardian_t *guardian, uint32_t current_time)
{
    if (guardian == NULL) {
        return;
    }

    guardian->last_cmd_time = current_time;
}

void LocalSafetyGuardian_Evaluate(const LocalSafetyGuardian_t *guardian,
                                  uint32_t current_time,
                                  float vx,
                                  float vy,
                                  float wz,
                                  LocalSafetyGuardianOutput_t *output)
{
    if ((guardian == NULL) || (output == NULL)) {
        return;
    }

    output->timed_out =
        ((current_time - guardian->last_cmd_time) > guardian->config.command_timeout_ms) ? 1U : 0U;
    output->vx = (output->timed_out != 0U) ? 0.0f : vx;
    output->vy = (output->timed_out != 0U) ? 0.0f : vy;
    output->wz = (output->timed_out != 0U) ? 0.0f : wz;
    output->request_auto_center =
        ((output->timed_out != 0U) ||
         (LocalSafetyGuardian_ShouldAutoCenter(guardian, output->vx, output->vy, output->wz) != 0U)) ? 1U : 0U;

    if (output->request_auto_center != 0U) {
        output->vx = 0.0f;
        output->vy = 0.0f;
        output->wz = 0.0f;
    }
}
