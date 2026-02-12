#ifndef TCS_LOGGER_H
#define TCS_LOGGER_H

#include "tcs_types.h"

// Define a structure for the CAN frame content if needed,
// or just a function to populate a buffer.
typedef struct {
  uint32_t id;
  uint8_t data[8];
  uint8_t dlc;
} TCS_CAN_Frame_t;

void Logger_Update(TCS_Handle_t *tcs);
bool Logger_GetFrame(TCS_CAN_Frame_t *frame1, TCS_CAN_Frame_t *frame2);

#endif // TCS_LOGGER_H
