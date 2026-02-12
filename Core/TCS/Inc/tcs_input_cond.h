#ifndef TCS_INPUT_COND_H
#define TCS_INPUT_COND_H

#include "tcs_types.h"

// Initialize Input Conditioner
void InputCond_Init(TCS_Handle_t *tcs);

// Process Inputs: Check timeouts, plausibility, filtering
// Returns 0 if OK, non-zero if critical fault detected immediately
void InputCond_Process(TCS_Handle_t *tcs, uint32_t current_tick);

// Helpers to update raw values safely (called from CAN ISR or Task)
void InputCond_UpdateWheelSpeeds(TCS_Handle_t *tcs, float fl, float fr,
                                 float rl, float rr);
void InputCond_UpdateIMU(TCS_Handle_t *tcs, float ax, float ay, float yaw);
void InputCond_UpdateTorqueRequest(TCS_Handle_t *tcs, float t_req, float t_max);
void InputCond_SetMode(TCS_Handle_t *tcs, TCS_Mode_e mode);

#endif // TCS_INPUT_COND_H
