#ifndef UNITREE_PLANNER_H
#define UNITREE_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "unitree_motor.h"

#define UNITREE_GEAR_RATIO      6.33f
#define UNITREE_GEAR_RATIO_SQ   (UNITREE_GEAR_RATIO * UNITREE_GEAR_RATIO)
#define PI                      3.1415926535f

typedef struct {
    uint32_t motor_id;
    UnitreeBusId_t bus_id;

    float cmd_kp_rotor;
    float cmd_kd_rotor;

    float start_pos_rad;
    float end_pos_rad;
    float total_dist_rad;

    float current_time;
    float total_duration;
    bool is_moving;

    float last_pos_target;
} UnitreePlanner_t;

void Unitree_Init(UnitreePlanner_t *planner, uint32_t motor_id);
void Unitree_InitOnBus(UnitreePlanner_t *planner, UnitreeBusId_t bus_id, uint32_t motor_id);

void Unitree_SetTrajectory(UnitreePlanner_t *planner,
                           float start_deg_out,
                           float end_deg_out,
                           float speed_deg_s,
                           float kp_out,
                           float kd_out);

void Unitree_UpdateAndSend(UnitreePlanner_t *planner, float dt);

#endif
