#ifndef TCS_APP_MAIN_H
#define TCS_APP_MAIN_H

#include "tcs_types.h"

// Initialize entire TCS system
void TCS_Init(void);

// Main Step Function to be called every 10ms
void TCS_Step(void);

// Access to handle for Debugging/Hooks
TCS_Handle_t *TCS_GetHandle(void);

#endif // TCS_APP_MAIN_H
