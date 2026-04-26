# 2026-04-12 CAN realtime refactor memory

## Goal

- Reduce `hfdcan2` interrupt load for DJI motors.
- Move DJI PID and CAN transmit out of RX ISR into the 1 ms task loop.
- Add a shared safe transmit wrapper for task-context CAN sends.

## Changes made

### `can/fdcan_bsp.c` / `can/fdcan_bsp.h`

- Added a per-bus FreeRTOS mutex array for task-context CAN TX serialization.
- Extended `fdcan_bsp_send()` with:
  - mutex take/release
  - TX FIFO free-level check
  - error counters for timeout / fifo full / HAL send failure
- Tightened `hfdcan2` RX filtering:
  - standard ID mask filter limited to `0x200` - `0x20F`
  - extended filter disabled
  - non-matching standard / extended / remote frames rejected

### `can/dji_motor.c` / `can/dji_motor.h`

- `dji_motor_message_handler()` now only updates feedback state.
- Added `DJI_Motor_Control_Loop_1ms()`:
  - runs DJI position / speed PID in task context
  - sends grouped `0x200` and `0x1FF` control frames through `fdcan_bsp_send()`
- Kept existing target-setting API unchanged to minimize call-site churn.

### `can/mechanism_task.c` / `can/mechanism_task.h`

- Marked `current_yaw_speed` and `current_pitch_target_deg` as `volatile`.
- Added `DJI_Motor_Control_Loop_1ms()` call inside `Mechanism_Loop_1ms()`.

### `Core/Src/freertos.c`

- `StartTask06()` changed from `osDelay(1)` to `vTaskDelayUntil()` for stable 1 ms cadence.
- `StartTask03()` and `StartTask07()` switched from direct `HAL_FDCAN_AddMessageToTxFifoQ()` to `fdcan_bsp_send()`.

## Why this layout

- Reused the existing 1 ms mechanism task instead of creating a new motor-control task.
- This keeps the refactor narrow while still removing the highest-risk ISR behavior.
- The new TX wrapper is a stop-gap that can later be evolved into a dedicated CAN TX task.

## Validation to run

- Keil full-project rebuild.
- Runtime check that DJI feedback still arrives on `hfdcan2`.
- Runtime check that CAN3 periodic feedback still transmits normally through the shared wrapper.
- Bus-noise test on `hfdcan2` to confirm reduced interrupt pressure after filter tightening.
