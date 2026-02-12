#include "tcs_safety_supervisor.h"

void SafetySupervisor_Check(TCS_Handle_t *tcs, uint32_t current_tick) {
  if (!tcs)
    return;

  tcs->fault_flags = TCS_FAULT_NONE;

  // 1. Check Timeouts
  uint32_t timeout = tcs->params.timeout_ms;

  if ((current_tick - tcs->inputs.wheel_speed_update_tick) > timeout) {
    // We could be specific about which wheel if we tracked them separately,
    // but for now, any wheel timeout is a critical system fault.
    tcs->fault_flags |=
        TCS_FAULT_TIMEOUT_WHEEL_FL; // Setting FL as general flag for now
  }

  if ((current_tick - tcs->inputs.imu_update_tick) > timeout) {
    tcs->fault_flags |= TCS_FAULT_TIMEOUT_IMU;
  }

  if ((current_tick - tcs->inputs.torque_update_tick) > timeout) {
    tcs->fault_flags |= TCS_FAULT_TIMEOUT_TORQUE;
  }

  // 2. Check Hard Slip (Redundant Safety Layer)
  // If controller fails to clamp, or physics goes wild
  if (tcs->state.slip_ratio_max > tcs->params.hard_cut_threshold) {
    tcs->fault_flags |= TCS_FAULT_HARD_SLIP;
  }

  // 3. Degrade / Fail-safe Logic
  if (tcs->fault_flags != TCS_FAULT_NONE) {
    // Critical Fault -> Disable Output
    tcs->is_active = false; // Disable TCS?

    // If it's a hard slip fault, we specifically want to cut torque to 0
    if (tcs->fault_flags & TCS_FAULT_HARD_SLIP) {
      tcs->state.limited_torque_nm = 0.0f;
    } else {
      // For timeouts, we might want to pass through T_req (no TCS) or cut
      // (unsafe vehicle) Prompt says: "Timeout -> TCS devre dışı" (TCS OFF) If
      // TCS is off, output = input? Or output = 0? "Degrade davranışı: TCS
      // devre dışı" usually means raw torque request passes through BUT "Hard
      // slip -> torque cut".

      // NOTE: Ideally, if WS sensors fail, we CANNOT calculate slip, so we MUST
      // disable TCS control loop. We pass the raw torque request (Open Loop).
      tcs->state.limited_torque_nm = tcs->inputs.torque_request_nm;
    }
  } else {
    tcs->is_active = true;
  }
}
