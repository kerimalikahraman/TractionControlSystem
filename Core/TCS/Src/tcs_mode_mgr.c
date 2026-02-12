#include "tcs_mode_mgr.h"

void ModeMgr_Init(TCS_Handle_t *tcs) {
  // Set default parameters for modes
  tcs->params.lambda_target_ac = 0.10f;
  tcs->params.lambda_target_sk = 0.10f;
  tcs->params.lambda_target_ax = 0.10f;
  tcs->params.lambda_target_en = 0.08f;
  tcs->params.lambda_target_es = 0.07f;
  tcs->params.lambda_target_wt = 0.05f;
  tcs->params.lambda_target_pit = 0.03f;
  tcs->params.lambda_target_ts = 0.10f;

  tcs->params.kp = 200.0f; // Example Gain, to be tuned
  tcs->params.hard_cut_threshold = 0.25f;
  tcs->params.timeout_ms = 100;
}

void ModeMgr_Update(TCS_Handle_t *tcs) {
  TCS_Mode_e mode = tcs->inputs.requested_mode;

  switch (mode) {
  case TCS_MODE_AC:
    tcs->state.target_slip = tcs->params.lambda_target_ac;
    break;
  case TCS_MODE_SK:
    tcs->state.target_slip = tcs->params.lambda_target_sk;
    break;
  case TCS_MODE_AX:
    tcs->state.target_slip = tcs->params.lambda_target_ax;
    break;
  case TCS_MODE_EN:
    tcs->state.target_slip = tcs->params.lambda_target_en;
    break;
  case TCS_MODE_ES:
    tcs->state.target_slip = tcs->params.lambda_target_es;
    break;
  case TCS_MODE_WT:
    tcs->state.target_slip = tcs->params.lambda_target_wt;
    break;
  case TCS_MODE_PIT:
    tcs->state.target_slip = tcs->params.lambda_target_pit;
    break;
  case TCS_MODE_TS:
    tcs->state.target_slip = tcs->params.lambda_target_ts;
    break;
  default:
    tcs->state.target_slip = 0.10f;
    break;
  }
}
