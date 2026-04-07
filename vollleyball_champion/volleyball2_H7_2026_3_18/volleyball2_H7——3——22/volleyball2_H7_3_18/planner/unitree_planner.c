#include "unitree_planner.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD(x) ((x) * (PI / 180.0f))
#define VEL_FF_GAIN 0.8f

void Unitree_Init(UnitreePlanner_t *planner, uint32_t motor_id)
{
    Unitree_InitOnBus(planner, UNITREE_BUS_USART3, motor_id);
}

void Unitree_InitOnBus(UnitreePlanner_t *planner, UnitreeBusId_t bus_id, uint32_t motor_id)
{
    memset(planner, 0, sizeof(UnitreePlanner_t));
    planner->motor_id = motor_id;
    planner->bus_id = bus_id;
    planner->is_moving = false;
    planner->last_pos_target = 0.0f;
}

void Unitree_SetTrajectory(UnitreePlanner_t *planner,
                           float start_deg_out,
                           float end_deg_out,
                           float speed_deg_s,
                           float kp_out,
                           float kd_out)
{
    if (speed_deg_s <= 0.0f)
    {
        return;
    }

    planner->cmd_kp_rotor = kp_out / UNITREE_GEAR_RATIO_SQ;
    planner->cmd_kd_rotor = kd_out / UNITREE_GEAR_RATIO_SQ;

    planner->start_pos_rad = DEG_TO_RAD(start_deg_out) * UNITREE_GEAR_RATIO;
    planner->end_pos_rad = DEG_TO_RAD(end_deg_out) * UNITREE_GEAR_RATIO;
    planner->total_dist_rad = planner->end_pos_rad - planner->start_pos_rad;

    {
        float speed_rad_s = DEG_TO_RAD(speed_deg_s) * UNITREE_GEAR_RATIO;
        planner->total_duration = fabsf(planner->total_dist_rad) / speed_rad_s;
    }

    planner->current_time = 0.0f;
    planner->last_pos_target = planner->start_pos_rad;
    planner->is_moving = true;
}

void Unitree_UpdateAndSend(UnitreePlanner_t *planner, float dt)
{
    float pos_cmd = 0.0f;
    float vel_cmd = 0.0f;
    float torque_ff = 0.0f;

    if (planner->is_moving)
    {
        planner->current_time += dt;

        if (planner->current_time >= planner->total_duration)
        {
            pos_cmd = planner->end_pos_rad;
            vel_cmd = 0.0f;

            planner->last_pos_target = pos_cmd;
            planner->is_moving = false;
        }
        else
        {
            float t = planner->current_time / planner->total_duration;
            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            float t5 = t4 * t;
            float pos_scale = (10.0f * t3) - (15.0f * t4) + (6.0f * t5);
            float vel_scale = (30.0f * t2) - (60.0f * t3) + (30.0f * t4);
            float raw_vel = (planner->total_dist_rad * vel_scale) / planner->total_duration;

            pos_cmd = planner->start_pos_rad + (planner->total_dist_rad * pos_scale);
            vel_cmd = raw_vel * VEL_FF_GAIN;
            planner->last_pos_target = pos_cmd;
        }
    }
    else
    {
        pos_cmd = planner->last_pos_target;
        vel_cmd = 0.0f;
    }

    Unitree_Send_CmdOnBus(planner->bus_id,
                          planner->motor_id,
                          torque_ff,
                          vel_cmd,
                          pos_cmd,
                          planner->cmd_kp_rotor,
                          planner->cmd_kd_rotor);
}
