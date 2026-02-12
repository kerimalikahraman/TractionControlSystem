#include "tcs_controller_core.h"

void ControllerCore_Update(TCS_Handle_t *tcs) {
  if (!tcs)
    return;

  // Safety First: If hard slip detected, cut immediately (Controller-level fast
  // response) Prompt: if λ > 0.25 -> T_cmd = 0
  if (tcs->state.slip_ratio_max > tcs->params.hard_cut_threshold) {
    tcs->state.limited_torque_nm = 0.0f;
    return; // Early exit
  }

  float lambda = tcs->state.slip_ratio_max;
  float lambda_target = tcs->state.target_slip;
  float t_req = tcs->inputs.torque_request_nm;
  float t_cmd = t_req;

  // P-Control Strategy
  // if λ > λ_target: T_cmd = T_req - K * (λ - λ_target)
  if (lambda > lambda_target) {
    float correction = tcs->params.kp * (lambda - lambda_target);
    t_cmd = t_req - correction;
  } else {
    t_cmd = t_req;
  }

  // Clamping
  // 0 <= T_cmd <= T_req
  if (t_cmd > t_req)
    t_cmd = t_req;
  if (t_cmd < 0.0f)
    t_cmd = 0.0f;

  // Global Torque Limit Check (e.g. Battery Power Limit)
  if (t_cmd > tcs->inputs.torque_max_limit_nm) {
    t_cmd = tcs->inputs.torque_max_limit_nm;
  }

  tcs->state.limited_torque_nm = t_cmd;
}
