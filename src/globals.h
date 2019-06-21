#ifndef GLOBALS_H
#define GLOBALS_H
//all these declared in main.c


#include "types.h"
#include "motor.h"
#include "configuration.h"

extern volatile operation_mode_t mode;

extern volatile uint16_t error_state;

extern volatile float mm_per_revolution;
extern volatile float operation_length; //mm.
extern volatile int32_t reference_pos; //start of operation point on threading.
extern volatile int32_t spindle_count;
extern volatile int32_t spindle_acceleration_target;
extern motor m1;

extern volatile motion_status_t status;
extern volatile float max_error;
extern configuration c;


#endif
