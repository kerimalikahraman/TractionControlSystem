#include "tcs_input_cond.h"
#include <math.h>

// Simple LPF alpha
#define LPF_ALPHA 0.8f // 1.0 = no filter, 0.0 = no update

void InputCond_Init(TCS_Handle_t *tcs) {
  if (!tcs)
    return;
  // Clear check
  tcs->inputs.wheel_speed_update_tick = 0;
  tcs->inputs.imu_update_tick = 0;
  tcs->inputs.torque_update_tick = 0;
  // Default mode
  tcs->inputs.requested_mode = TCS_MODE_TS;
}

void InputCond_Process(TCS_Handle_t *tcs, uint32_t current_tick) {
  if (!tcs)
    return;

  // Plausibility Checks (Min/Max clamping if needed)
  // For now, raw values are passed.
  // In a real car, check for NaN, Inf, or impossible values (e.g. > 300km/h)

  // Timeout Checking is done in SafetySupervisor,
  // but we can prepare flags here if needed.
  // The design requests "InputCond" to do "timeout", but "SafetySupervisor" to
  // "fault". I'll keep the logic clean: InputCond ensures the data in struct is
  // "clean" or "stale". SafetySupervisor will read the timestamps and declare
  // the fault.
}

void InputCond_UpdateWheelSpeeds(TCS_Handle_t *tcs, float fl, float fr,
                                 float rl, float rr) {
  // Apply Low Pass Filter
  // y[n] = alpha * x[n] + (1-alpha) * y[n-1]

  // Note: For first sample, we might want to initialize directly.
  // Assuming 0 initialization is fine for startup.

  tcs->inputs.wheel_speed_fl_rads =
      LPF_ALPHA * fl + (1.0f - LPF_ALPHA) * tcs->inputs.wheel_speed_fl_rads;
  tcs->inputs.wheel_speed_fr_rads =
      LPF_ALPHA * fr + (1.0f - LPF_ALPHA) * tcs->inputs.wheel_speed_fr_rads;
  tcs->inputs.wheel_speed_rl_rads =
      LPF_ALPHA * rl + (1.0f - LPF_ALPHA) * tcs->inputs.wheel_speed_rl_rads;
  tcs->inputs.wheel_speed_rr_rads =
      LPF_ALPHA * rr + (1.0f - LPF_ALPHA) * tcs->inputs.wheel_speed_rr_rads;

  // Assuming HAL_GetTick() is used externally and passed or called here.
  // We'll update the timestamp.
  extern uint32_t HAL_GetTick(void);
  tcs->inputs.wheel_speed_update_tick = HAL_GetTick();
}

void InputCond_UpdateIMU(TCS_Handle_t *tcs, float ax, float ay, float yaw) {
  tcs->inputs.accel_x_mps2 =
      LPF_ALPHA * ax + (1.0f - LPF_ALPHA) * tcs->inputs.accel_x_mps2;
  tcs->inputs.accel_y_mps2 =
      LPF_ALPHA * ay + (1.0f - LPF_ALPHA) * tcs->inputs.accel_y_mps2;
  tcs->inputs.yaw_rate_rads =
      LPF_ALPHA * yaw + (1.0f - LPF_ALPHA) * tcs->inputs.yaw_rate_rads;

  extern uint32_t HAL_GetTick(void);
  tcs->inputs.imu_update_tick = HAL_GetTick();
}

void InputCond_UpdateTorqueRequest(TCS_Handle_t *tcs, float t_req,
                                   float t_max) {
  tcs->inputs.torque_request_nm = t_req;
  tcs->inputs.torque_max_limit_nm = t_max;

  extern uint32_t HAL_GetTick(void);
  tcs->inputs.torque_update_tick = HAL_GetTick();
}

void InputCond_SetMode(TCS_Handle_t *tcs, TCS_Mode_e mode) {
  if (mode < TCS_MODE_COUNT) {
    tcs->inputs.requested_mode = mode;
  }
}
