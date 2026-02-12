#include "tcs_app_main.h"

// Include all modules
#include "tcs_controller_core.h"
#include "tcs_input_cond.h"
#include "tcs_logger.h"
#include "tcs_mode_mgr.h"
#include "tcs_safety_supervisor.h"
#include "tcs_slip_calc.h"
#include "tcs_vx_estimator.h"


// Main Static Handle
static TCS_Handle_t tcs_handle;

// External Tick for Reference
extern uint32_t HAL_GetTick(void);

void TCS_Init(void) {
  InputCond_Init(&tcs_handle);
  ModeMgr_Init(&tcs_handle);
  // Other modules strictly functional, no init needed or init via handle clean

  tcs_handle.is_active = true;
}

void TCS_Step(void) {
  uint32_t current_tick = HAL_GetTick();

  // 1. Process Inputs (Filter, Timestamp check preparation)
  InputCond_Process(&tcs_handle, current_tick);

  // 2. Safety Check (Pre-Calculation) - Detect Timeouts
  SafetySupervisor_Check(&tcs_handle, current_tick);

  // Optimize: If hard fault (Timeout), maybe skip calculations?
  // For now, run everything deterministically.

  // 3. Mode Management (Determine Targets)
  ModeMgr_Update(&tcs_handle);

  // 4. State Estimation
  VxEstimator_Update(&tcs_handle);
  SlipCalc_Update(&tcs_handle);

  // 5. Control Law
  ControllerCore_Update(&tcs_handle);

  // 6. Safety Enforce (Post-Calculation) - Output Override
  SafetySupervisor_Check(
      &tcs_handle,
      current_tick); // Re-check to latch hard slip or override torque
  // (Note: ControllerCore might set torque to 0 on hard cut, Safety updates
  // flags and confirms disable)

  // 7. Logger
  Logger_Update(&tcs_handle);
}

TCS_Handle_t *TCS_GetHandle(void) { return &tcs_handle; }
