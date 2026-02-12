#ifndef TCS_TYPES_H
#define TCS_TYPES_H

#include <stdbool.h>
#include <stdint.h>


/* --- Constants --- */
#define TCS_WHEEL_RADIUS_M 0.25f // Example value, user to configure
#define TCS_MIN_VELOCITY_MPS                                                   \
  1.0f // Minimum velocity to calculate slip (avoid div/0)
#define TCS_MAX_TORQUE_NM 200.0f // Max torque physical limit
#define TCS_LOOP_PERIOD_MS 10    // 100 Hz

/* --- Enums --- */
typedef enum {
  TCS_MODE_AC = 0, // Acceleration
  TCS_MODE_SK,     // Skidpad
  TCS_MODE_AX,     // Autocross
  TCS_MODE_EN,     // Endurance
  TCS_MODE_ES,     // Efficiency? / Extra
  TCS_MODE_WT,     // Wet
  TCS_MODE_PIT,    // Pit Limiter / Low Grip
  TCS_MODE_TS,     // Test / Tuning
  TCS_MODE_COUNT
} TCS_Mode_e;

typedef enum {
  TCS_FAULT_NONE = 0x00,
  TCS_FAULT_TIMEOUT_WHEEL_FL = 0x01,
  TCS_FAULT_TIMEOUT_WHEEL_FR = 0x02,
  TCS_FAULT_TIMEOUT_WHEEL_RL = 0x04,
  TCS_FAULT_TIMEOUT_WHEEL_RR = 0x08,
  TCS_FAULT_TIMEOUT_IMU = 0x10,
  TCS_FAULT_TIMEOUT_TORQUE = 0x20,
  TCS_FAULT_HARD_SLIP = 0x40,
  TCS_FAULT_SYSTEM_ERROR = 0x80
} TCS_FaultFlags_e;

/* --- Structs --- */

// 1. Raw Inputs from Sensors/CAN
typedef struct {
  float wheel_speed_fl_rads;
  float wheel_speed_fr_rads;
  float wheel_speed_rl_rads;
  float wheel_speed_rr_rads;
  uint32_t wheel_speed_update_tick; // Timestamp of last update

  float accel_x_mps2;
  float accel_y_mps2;
  float yaw_rate_rads;
  uint32_t imu_update_tick;

  float torque_request_nm;
  float torque_max_limit_nm; // From BMS or other limits
  uint32_t torque_update_tick;

  TCS_Mode_e requested_mode;
} TCS_Inputs_t;

// 2. Calculated State
typedef struct {
  float vehicle_speed_est_mps; // Vx_hat
  float slip_ratio_rl;         // Lambda RL
  float slip_ratio_rr;         // Lambda RR
  float slip_ratio_max;        // Max(RL, RR)

  // For NN Data Collection
  float slip_derivative; // d(lambda)/dt (prepared for future)

  float target_slip;       // Defined by ModeMgr
  float limited_torque_nm; // Cmd output
} TCS_State_t;

// 3. Parameters (Tuning)
typedef struct {
  float kp; // Proportional gain

  // Slip Targets per Mode
  float lambda_target_ac;
  float lambda_target_sk;
  float lambda_target_ax;
  float lambda_target_en;
  float lambda_target_es;
  float lambda_target_wt;
  float lambda_target_pit;
  float lambda_target_ts;

  float hard_cut_threshold; // e.g. 0.25
  uint32_t timeout_ms;      // e.g. 100ms
} TCS_Params_t;

// 4. Main Handle
typedef struct {
  TCS_Inputs_t inputs;
  TCS_State_t state;
  TCS_Params_t params;

  uint8_t fault_flags; // Bitwise OR of TCS_FaultFlags_e
  bool is_active;      // Logic enable state
} TCS_Handle_t;

#endif // TCS_TYPES_H
