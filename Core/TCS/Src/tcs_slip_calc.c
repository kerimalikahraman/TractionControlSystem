#include "tcs_slip_calc.h"
#include <math.h>

void SlipCalc_Update(TCS_Handle_t *tcs) {
  float vx = tcs->state.vehicle_speed_est_mps;

  // Denominator protection
  float v_denom = (vx > TCS_MIN_VELOCITY_MPS) ? vx : TCS_MIN_VELOCITY_MPS;

  // Calculate Rear Wheel Linear Speeds
  float v_rl = tcs->inputs.wheel_speed_rl_rads * TCS_WHEEL_RADIUS_M;
  float v_rr = tcs->inputs.wheel_speed_rr_rads * TCS_WHEEL_RADIUS_M;

  // Calculate Slip
  // λ = (ω_R * R - Vx_hat) / max(Vx_hat, Vmin)
  tcs->state.slip_ratio_rl = (v_rl - vx) / v_denom;
  tcs->state.slip_ratio_rr = (v_rr - vx) / v_denom;

  // Determine Max Slip for Control
  tcs->state.slip_ratio_max =
      (tcs->state.slip_ratio_rl > tcs->state.slip_ratio_rr)
          ? tcs->state.slip_ratio_rl
          : tcs->state.slip_ratio_rr;

  // Calculate Derivative (Simple finite difference)
  // Note: This requires state persistence.
  // For MVP, just storing one previous value.
  static float prev_slip_max = 0.0f;
  float dt = (float)TCS_LOOP_PERIOD_MS / 1000.0f;

  tcs->state.slip_derivative = (tcs->state.slip_ratio_max - prev_slip_max) / dt;
  prev_slip_max = tcs->state.slip_ratio_max;
}
