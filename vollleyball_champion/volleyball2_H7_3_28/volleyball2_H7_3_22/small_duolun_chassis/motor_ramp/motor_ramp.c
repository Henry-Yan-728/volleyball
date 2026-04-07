#include "motor_ramp.h"

static int32_t Ramp_Abs(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t Ramp_PositiveOrOne(int32_t value)
{
    value = (value >= 0) ? value : -value;
    return (value > 0) ? value : 1;
}

static int32_t Ramp_SelectMaxStep(const RampController_t *ramp, int32_t current, int32_t target)
{
    if (current < target) {
        return (current >= 0) ? ramp->accel_step : ramp->decel_step;
    }

    if (current > target) {
        return (current <= 0) ? ramp->accel_step : ramp->decel_step;
    }

    return 0;
}

void Motor_Ramp_Init(RampController_t *ramp, int32_t accel, int32_t decel)
{
    if (ramp == NULL) return;

    ramp->target_val = 0;
    ramp->current_val = 0;

    ramp->accel_step = Ramp_PositiveOrOne(accel);
    ramp->decel_step = Ramp_PositiveOrOne(decel);

    {
        int32_t base_step = (ramp->accel_step < ramp->decel_step) ? ramp->accel_step : ramp->decel_step;
        ramp->jerk_step = Ramp_PositiveOrOne(base_step / 2);
    }

    ramp->current_step = 0;
}

void Motor_Ramp_SetJerk(RampController_t *ramp, int32_t jerk)
{
    if (ramp == NULL) return;
    ramp->jerk_step = Ramp_PositiveOrOne(jerk);
}

void Motor_Ramp_SetTarget(RampController_t *ramp, int32_t target)
{
    if (ramp == NULL) return;
    ramp->target_val = target;
}

int32_t Motor_Ramp_Calc(RampController_t *ramp)
{
    int32_t target;
    int32_t current;
    int32_t error;
    int32_t max_step;
    int32_t desired_step;
    int32_t step;
    int32_t jerk;
    uint8_t reversing;

    if (ramp == NULL) return 0;

    target = ramp->target_val;
    current = ramp->current_val;
    step = ramp->current_step;

    error = target - current;
    if (error == 0) {
        ramp->current_step = 0;
        return current;
    }

    max_step = Ramp_PositiveOrOne(Ramp_SelectMaxStep(ramp, current, target));
    desired_step = (error > 0) ? max_step : -max_step;
    jerk = Ramp_PositiveOrOne(ramp->jerk_step);

    reversing = ((step > 0 && desired_step < 0) || (step < 0 && desired_step > 0));
    if (reversing) {
        step = 0;
    } else if (step < desired_step) {
        step += jerk;
        if (step > desired_step) step = desired_step;
    } else if (step > desired_step) {
        step -= jerk;
        if (step < desired_step) step = desired_step;
    }

    if (!reversing && error > 0 && step <= 0) {
        step = 1;
    } else if (!reversing && error < 0 && step >= 0) {
        step = -1;
    }

    if (Ramp_Abs(error) < Ramp_Abs(step)) {
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

void Motor_Ramp_EStop(RampController_t *ramp)
{
    if (ramp == NULL) return;
    ramp->target_val = 0;
    ramp->current_val = 0;
    ramp->current_step = 0;
}
