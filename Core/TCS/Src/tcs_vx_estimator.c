#include "tcs_vx_estimator.h"
#include <math.h>

void VxEstimator_Update(TCS_Handle_t *tcs) {
  // Strategy: Vx_hat = min(wFL * R, wFR * R)
  // Assuming non-driven front wheels for RWD car.

  float v_fl = tcs->inputs.wheel_speed_fl_rads * TCS_WHEEL_RADIUS_M;
  float v_fr = tcs->inputs.wheel_speed_fr_rads * TCS_WHEEL_RADIUS_M;

  // Simple min estimation (valid for straight line acce/braking, RWD)
  // For cornering, average might be better, but prompt requested 'min'
  // explicitly? Prompt: "Vx_hat = min(ω_FL * R , ω_FR * R)"

  float vx_hat = (v_fl < v_fr) ? v_fl : v_fr;

  // Avoid negative speed issues if needed, though RWD logic usually assumes
  // forward
  if (vx_hat < 0.0f)
    vx_hat = 0.0f;

  tcs->state.vehicle_speed_est_mps = vx_hat;
}
