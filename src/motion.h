#ifndef MOTION_H
#define MOTION_H
#include "globals.h"
#define LOOP_PER 0.0002
#define LOOP_FREQ 5000
#define SPINDLE_PPR 2400



//#define SIMULATE_ENCODER

void init_motion(void);
void get_spindle_position(void);
void rpm_estimator(void);
void start_acceleration(void);
void accelerate(void);
void follow(void);
void decelerate(void);
void return_to_start(void);

#endif
