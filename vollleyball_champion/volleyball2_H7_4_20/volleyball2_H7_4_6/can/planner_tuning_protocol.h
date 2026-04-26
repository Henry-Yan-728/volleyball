#ifndef PLANNER_TUNING_PROTOCOL_H
#define PLANNER_TUNING_PROTOCOL_H

#include <stdint.h>

#define PLANNER_TUNING_FRAME_LENGTH 8U
#define PLANNER_TUNING_CMD_WRITE    0x01U
#define PLANNER_TUNING_CMD_READ     0x02U
#define PLANNER_TUNING_FRAME_HEADER 0xAAU

typedef enum {
    PID_MAX_SPD        = 0x10,
    PID_START_SPD      = 0x11,
    PID_STOP_SPD       = 0x12,
    PID_UP_DIST        = 0x13,
    PID_DOWN_DIST      = 0x14,

    PID_ANGLE_KP       = 0x20,
    PID_MAX_VR         = 0x21,
    PID_VR_SLEW_STEP   = 0x22,
    PID_YAW_DEADZONE   = 0x23,

    PID_FAR_NEAR_DIST  = 0x30,
    PID_FAR_WEIGHT_MIN = 0x31,
    PID_FAR_MAX_VR_SCALE = 0x32,
    PID_FAR_VR_SLEW_SCALE = 0x33,

    PID_POS_TOLERANCE  = 0x40,
    PID_YAW_TOLERANCE  = 0x41,

    SYS_SAVE_TO_FLASH  = 0xF0,
    SYS_RESET_DEFAULT  = 0xFF
} Param_ID_e;

typedef struct {
    uint8_t cmd_type;
    uint8_t param_id;
    float value;
} PlannerTuningFrame_t;

void PlannerTuning_Reset(void);
uint8_t PlannerTuning_Write(const uint8_t *data, uint8_t length);
uint8_t PlannerTuning_GetFrame(PlannerTuningFrame_t *frame);
uint8_t PlannerTuning_BuildFrame(uint8_t cmd_type, uint8_t param_id, float value,
                                 uint8_t out_frame[PLANNER_TUNING_FRAME_LENGTH]);

#endif
