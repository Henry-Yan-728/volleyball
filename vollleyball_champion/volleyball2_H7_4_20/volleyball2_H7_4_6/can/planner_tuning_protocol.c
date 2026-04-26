#include "planner_tuning_protocol.h"
#include <string.h>

#define PLANNER_TUNING_BUFFER_SIZE 128U

static uint8_t s_buffer[PLANNER_TUNING_BUFFER_SIZE];
static uint16_t s_read_index = 0U;
static uint16_t s_write_index = 0U;

static uint8_t planner_tuning_checksum(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < length; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

static void planner_tuning_add_read_index(uint16_t length)
{
    s_read_index = (uint16_t)((s_read_index + length) % PLANNER_TUNING_BUFFER_SIZE);
}

static uint8_t planner_tuning_read(uint16_t index)
{
    return s_buffer[index % PLANNER_TUNING_BUFFER_SIZE];
}

static uint16_t planner_tuning_get_length(void)
{
    return (uint16_t)((s_write_index + PLANNER_TUNING_BUFFER_SIZE - s_read_index) % PLANNER_TUNING_BUFFER_SIZE);
}

static uint16_t planner_tuning_get_remain(void)
{
    return (uint16_t)((PLANNER_TUNING_BUFFER_SIZE - 1U) - planner_tuning_get_length());
}

void PlannerTuning_Reset(void)
{
    s_read_index = 0U;
    s_write_index = 0U;
    memset(s_buffer, 0, sizeof(s_buffer));
}

uint8_t PlannerTuning_Write(const uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0U) || (length > planner_tuning_get_remain())) {
        return 0U;
    }

    for (uint8_t i = 0U; i < length; ++i) {
        s_buffer[s_write_index] = data[i];
        s_write_index = (uint16_t)((s_write_index + 1U) % PLANNER_TUNING_BUFFER_SIZE);
    }

    return length;
}

uint8_t PlannerTuning_GetFrame(PlannerTuningFrame_t *frame)
{
    uint8_t raw_frame[PLANNER_TUNING_FRAME_LENGTH];

    if (frame == NULL) {
        return 0U;
    }

    while (1) {
        if (planner_tuning_get_length() < PLANNER_TUNING_FRAME_LENGTH) {
            return 0U;
        }

        if (planner_tuning_read(s_read_index) != PLANNER_TUNING_FRAME_HEADER) {
            planner_tuning_add_read_index(1U);
            continue;
        }

        for (uint8_t i = 0U; i < PLANNER_TUNING_FRAME_LENGTH; ++i) {
            raw_frame[i] = planner_tuning_read((uint16_t)(s_read_index + i));
        }

        if (planner_tuning_checksum(raw_frame, PLANNER_TUNING_FRAME_LENGTH - 1U) != raw_frame[PLANNER_TUNING_FRAME_LENGTH - 1U]) {
            planner_tuning_add_read_index(1U);
            continue;
        }

        frame->cmd_type = raw_frame[1];
        frame->param_id = raw_frame[2];
        memcpy(&frame->value, &raw_frame[3], sizeof(float));
        planner_tuning_add_read_index(PLANNER_TUNING_FRAME_LENGTH);
        return 1U;
    }
}

uint8_t PlannerTuning_BuildFrame(uint8_t cmd_type, uint8_t param_id, float value,
                                 uint8_t out_frame[PLANNER_TUNING_FRAME_LENGTH])
{
    if (out_frame == NULL) {
        return 0U;
    }

    out_frame[0] = PLANNER_TUNING_FRAME_HEADER;
    out_frame[1] = cmd_type;
    out_frame[2] = param_id;
    memcpy(&out_frame[3], &value, sizeof(float));
    out_frame[7] = planner_tuning_checksum(out_frame, PLANNER_TUNING_FRAME_LENGTH - 1U);
    return 1U;
}
