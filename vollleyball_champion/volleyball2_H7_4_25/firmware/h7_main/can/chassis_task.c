#include "chassis_task.h"
#include "fdcan_bsp.h"

#include <math.h>
#include <stdint.h>

#if CHASSIS_LOCAL_CONTROL_ENABLE
    #include "h7_vesc.h"
    #include "local_chassis_control.h"
    #include "local_kinematics.h"
    #include "local_safety_guardian.h"
#endif

#define CHASSIS_CMD_DLC FDCAN_DLC_BYTES_8

volatile uint32_t g_chassis_fdcan1_tx_ok_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_wait_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_drop_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_drop_streak_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_fault_count = 0U;
volatile uint32_t g_chassis_cmd_clip_count = 0U;
volatile uint32_t g_chassis_local_update_count = 0U;
volatile uint32_t g_chassis_local_timeout_count = 0U;

#if CHASSIS_LOCAL_CONTROL_ENABLE
static volatile float s_chassis_target_vx_ms = 0.0f;
static volatile float s_chassis_target_vy_ms = 0.0f;
static volatile float s_chassis_target_wz = 0.0f;

static LocalSafetyGuardian_t s_chassis_guardian;

static const LocalChassisConfig_t s_chassis_config = {
    .radius = CHASSIS_RADIUS,
    .steering_offset_deg = {
        CHASSIS_STEERING_OFFSET_WHEEL_0_DEG,
        CHASSIS_STEERING_OFFSET_WHEEL_1_DEG,
        CHASSIS_STEERING_OFFSET_WHEEL_2_DEG,
    },
};

static const LocalSafetyGuardianConfig_t s_chassis_safety_config = {
    .command_timeout_ms = CHASSIS_CMD_TIMEOUT_MS,
    .auto_center_linear_threshold = CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD,
    .auto_center_angular_threshold = CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD,
    .auto_center_angle_deg = CHASSIS_AUTO_CENTER_ANGLE_DEG,
};

static float Chassis_ApplyMinAbs(float value, float min_abs);
static void Chassis_SetLocalCommand(float vx_ms, float vy_ms, float wz);
static void Chassis_GetLocalCommand(float *vx_ms, float *vy_ms, float *wz);
static uint8_t Chassis_ShouldApplySpinCompensation(const LocalSafetyGuardianOutput_t *guarded_cmd);
static void Chassis_ApplySpinCompensation(const LocalSafetyGuardianOutput_t *guarded_cmd,
                                          LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT]);
static void Chassis_ApplyWheelCommands(const LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT]);
static void Chassis_AutoCenter(void);
static void Chassis_RunLocalMotionPipeline(float vx_ms, float vy_ms, float wz);
#endif

static float Chassis_HostLinearMmToWireMs(float host_linear_mm_s)
{
    return host_linear_mm_s / CHASSIS_HOST_LINEAR_MM_PER_M;
}

static uint8_t Chassis_CommandWouldClip(const ChassisProtocolCommand_t *cmd)
{
    const float linear_limit = 32767.0f / CHASSIS_PROTOCOL_LINEAR_SCALE;
    const float angular_limit = 32767.0f / CHASSIS_PROTOCOL_ANGULAR_SCALE;

    if (cmd == 0) {
        return 0U;
    }

    return (fabsf(cmd->vx) > linear_limit) ||
           (fabsf(cmd->vy) > linear_limit) ||
           (fabsf(cmd->wz) > angular_limit);
}

static uint8_t FDCAN1_Transmit(const uint8_t *tx_data, uint32_t id)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = CHASSIS_CMD_DLC;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    for (uint8_t retry = 0U; retry < 3U; retry++)
    {
        if (fdcan_bsp_send(&hfdcan1, &tx_header, (uint8_t *)tx_data) == 0U)
        {
            if (retry != 0U)
            {
                g_chassis_fdcan1_tx_wait_count++;
            }
            g_chassis_fdcan1_tx_ok_count++;
            return 0U;
        }
    }

    g_chassis_fdcan1_tx_drop_count++;
    return 1U;
}

void Chassis_Init(void)
{
#if CHASSIS_LOCAL_CONTROL_ENABLE
    H7Vesc_Init();
    LocalChassisControl_Init();
    LocalSafetyGuardian_Init(&s_chassis_guardian, &s_chassis_safety_config, HAL_GetTick());
    Chassis_SetLocalCommand(0.0f, 0.0f, 0.0f);
#else
    Chassis_Stop();
#endif
}

void Chassis_Update(float vx, float vy, float vr)
{
#if CHASSIS_LOCAL_CONTROL_ENABLE
    float vx_ms = Chassis_HostLinearMmToWireMs(vx);
    float vy_ms = Chassis_HostLinearMmToWireMs(vy);

    Chassis_SetLocalCommand(vx_ms, vy_ms, vr);
    LocalSafetyGuardian_UpdateHeartbeat(&s_chassis_guardian, HAL_GetTick());
    Chassis_RunLocalMotionPipeline(vx_ms, vy_ms, vr);
    g_chassis_local_update_count++;
#else
    uint8_t tx_message[8];
    ChassisProtocolCommand_t cmd;
    const uint8_t link_degraded =
        (g_chassis_fdcan1_tx_drop_streak_count >= CHASSIS_TX_DROP_STREAK_LIMIT) ? 1U : 0U;

    cmd.vx = link_degraded ? 0.0f : Chassis_HostLinearMmToWireMs(vx);
    cmd.vy = link_degraded ? 0.0f : Chassis_HostLinearMmToWireMs(vy);
    cmd.wz = link_degraded ? 0.0f : vr;
    cmd.reserved = CHASSIS_PROTOCOL_RESERVED_BYTE;

    if (Chassis_CommandWouldClip(&cmd) != 0U) {
        g_chassis_cmd_clip_count++;
    }

    ChassisProtocol_PackCanonical(&cmd, tx_message);

    if (FDCAN1_Transmit(tx_message, CHASSIS_CMD_CAN_ID) == 0U) {
        g_chassis_fdcan1_tx_drop_streak_count = 0U;
        return;
    }

    g_chassis_fdcan1_tx_drop_streak_count++;
    if (g_chassis_fdcan1_tx_drop_streak_count == CHASSIS_TX_DROP_STREAK_LIMIT) {
        g_chassis_fdcan1_tx_fault_count++;
    }
#endif
}

void Chassis_Stop(void)
{
#if CHASSIS_LOCAL_CONTROL_ENABLE
    Chassis_SetLocalCommand(0.0f, 0.0f, 0.0f);
    LocalSafetyGuardian_UpdateHeartbeat(&s_chassis_guardian, HAL_GetTick());
    Chassis_RunLocalMotionPipeline(0.0f, 0.0f, 0.0f);
#else
    Chassis_Update(0.0f, 0.0f, 0.0f);
#endif
}

void Chassis_Task_Loop(void)
{
#if CHASSIS_LOCAL_CONTROL_ENABLE
    float vx_ms;
    float vy_ms;
    float wz;

    Chassis_GetLocalCommand(&vx_ms, &vy_ms, &wz);
    Chassis_RunLocalMotionPipeline(vx_ms, vy_ms, wz);
#endif
}

void Chassis_SafetyTick(void)
{
#if CHASSIS_LOCAL_CONTROL_ENABLE
    float vx_ms;
    float vy_ms;
    float wz;
    LocalSafetyGuardianOutput_t guarded_cmd;

    Chassis_GetLocalCommand(&vx_ms, &vy_ms, &wz);
    LocalSafetyGuardian_Evaluate(&s_chassis_guardian, HAL_GetTick(), vx_ms, vy_ms, wz, &guarded_cmd);
    if (guarded_cmd.timed_out != 0U) {
        Chassis_SetLocalCommand(0.0f, 0.0f, 0.0f);
        g_chassis_local_timeout_count++;
    }
#endif
}

#if CHASSIS_LOCAL_CONTROL_ENABLE
static float Chassis_ApplyMinAbs(float value, float min_abs)
{
    if (fabsf(value) >= min_abs) {
        return value;
    }

    return (value >= 0.0f) ? min_abs : -min_abs;
}

static void Chassis_SetLocalCommand(float vx_ms, float vy_ms, float wz)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_chassis_target_vx_ms = vx_ms;
    s_chassis_target_vy_ms = vy_ms;
    s_chassis_target_wz = wz;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void Chassis_GetLocalCommand(float *vx_ms, float *vy_ms, float *wz)
{
    uint32_t primask;

    if ((vx_ms == NULL) || (vy_ms == NULL) || (wz == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *vx_ms = s_chassis_target_vx_ms;
    *vy_ms = s_chassis_target_vy_ms;
    *wz = s_chassis_target_wz;
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint8_t Chassis_ShouldApplySpinCompensation(const LocalSafetyGuardianOutput_t *guarded_cmd)
{
    if (guarded_cmd == NULL) {
        return 0U;
    }

    return ((fabsf(guarded_cmd->vx) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
            (fabsf(guarded_cmd->vy) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
            (fabsf(guarded_cmd->wz) >= CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD)) ? 1U : 0U;
}

static void Chassis_ApplySpinCompensation(const LocalSafetyGuardianOutput_t *guarded_cmd,
                                          LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT])
{
    if (Chassis_ShouldApplySpinCompensation(guarded_cmd) == 0U) {
        return;
    }

    for (uint32_t i = 0U; i < LOCAL_CHASSIS_WHEEL_COUNT; ++i) {
        wheel_commands[i].drive_speed_ms =
            Chassis_ApplyMinAbs(wheel_commands[i].drive_speed_ms, CHASSIS_SPIN_MIN_WHEEL_SPEED);
    }
}

static void Chassis_ApplyWheelCommands(const LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT])
{
    if (wheel_commands == NULL) {
        return;
    }

    for (uint32_t i = 0U; i < LOCAL_CHASSIS_WHEEL_COUNT; ++i) {
        LocalChassisControl_SetWheel((uint8_t)i,
                                     wheel_commands[i].drive_speed_ms,
                                     wheel_commands[i].steering_angle_deg);
    }
}

static void Chassis_AutoCenter(void)
{
    for (uint32_t i = 0U; i < LOCAL_CHASSIS_WHEEL_COUNT; ++i) {
        LocalChassisControl_AutoCenterWheel((uint8_t)i, s_chassis_safety_config.auto_center_angle_deg);
    }
}

static void Chassis_RunLocalMotionPipeline(float vx_ms, float vy_ms, float wz)
{
    LocalSafetyGuardianOutput_t guarded_cmd;
    LocalChassisWheelCommand_t wheel_commands[LOCAL_CHASSIS_WHEEL_COUNT];

    LocalSafetyGuardian_Evaluate(&s_chassis_guardian, HAL_GetTick(), vx_ms, vy_ms, wz, &guarded_cmd);
    if (guarded_cmd.request_auto_center != 0U) {
        Chassis_AutoCenter();
        return;
    }

    LocalChassisKinematics_Solve(&s_chassis_config,
                                 guarded_cmd.vx,
                                 guarded_cmd.vy,
                                 guarded_cmd.wz,
                                 wheel_commands);
    Chassis_ApplySpinCompensation(&guarded_cmd, wheel_commands);
    LocalChassisKinematics_DesaturateWheelSpeeds(wheel_commands, CHASSIS_MAX_WHEEL_SPEED_MS);
    Chassis_ApplyWheelCommands(wheel_commands);
}
#endif
