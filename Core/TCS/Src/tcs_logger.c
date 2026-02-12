#include "tcs_logger.h"

static TCS_Handle_t *log_tcs_ptr = 0;

void Logger_Update(TCS_Handle_t *tcs) {
  log_tcs_ptr = tcs;
  // Real-time logging logic could trigger transmission here
}

bool Logger_GetFrame(TCS_CAN_Frame_t *frame1, TCS_CAN_Frame_t *frame2) {
  if (!log_tcs_ptr)
    return false;

  // Frame 1: Slips and Speed
  // ID: 0x500 (Example)
  // Data: Vx(2B), lambda_RL(2B), lambda_RR(2B), lambda_max(2B)
  // Scale: Vx * 100, Lambda * 10000

  frame1->id = 0x500;
  frame1->dlc = 8;

  int16_t vx_int = (int16_t)(log_tcs_ptr->state.vehicle_speed_est_mps * 100.0f);
  int16_t l_rl_int = (int16_t)(log_tcs_ptr->state.slip_ratio_rl * 10000.0f);
  int16_t l_rr_int = (int16_t)(log_tcs_ptr->state.slip_ratio_rr * 10000.0f);
  int16_t l_max_int = (int16_t)(log_tcs_ptr->state.slip_ratio_max * 10000.0f);

  frame1->data[0] = vx_int & 0xFF;
  frame1->data[1] = (vx_int >> 8) & 0xFF;
  frame1->data[2] = l_rl_int & 0xFF;
  frame1->data[3] = (l_rl_int >> 8) & 0xFF;
  frame1->data[4] = l_rr_int & 0xFF;
  frame1->data[5] = (l_rr_int >> 8) & 0xFF;
  frame1->data[6] = l_max_int & 0xFF;
  frame1->data[7] = (l_max_int >> 8) & 0xFF;

  // Frame 2: Torques, Mode, Faults
  // ID: 0x501
  // Data: T_req(2B), T_cmd(2B), Mode(1B), Faults(1B), Res(2B)
  // Scale: Torque * 10

  frame2->id = 0x501;
  frame2->dlc = 8;

  int16_t treq_int = (int16_t)(log_tcs_ptr->inputs.torque_request_nm * 10.0f);
  int16_t tcmd_int = (int16_t)(log_tcs_ptr->state.limited_torque_nm * 10.0f);

  frame2->data[0] = treq_int & 0xFF;
  frame2->data[1] = (treq_int >> 8) & 0xFF;
  frame2->data[2] = tcmd_int & 0xFF;
  frame2->data[3] = (tcmd_int >> 8) & 0xFF;
  frame2->data[4] = (uint8_t)log_tcs_ptr->inputs.requested_mode;
  frame2->data[5] = log_tcs_ptr->fault_flags;
  frame2->data[6] = 0;
  frame2->data[7] = 0;

  return true;
}
