#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdint.h>
#include "types.h"
//#define INVERT_DIR

typedef struct  {
    volatile int64_t pos_accu;
    volatile int64_t pos_target; //counts
    volatile int64_t ref_pos;
    volatile int8_t direction;
    volatile uint16_t step_period;

    float cur_speed; //mm/s
    int32_t cur_speed_cnt; //counts/sec
    //float max_speed; //mm/s
    int32_t max_speed_cnt;
    //float max_accel; //mm/s^2
    //float max_stepgen_accel; //mm/s^2
    int32_t max_stepgen_accel_cnt;
    //float max_stepgen_accel_per_period;
    uint16_t dir_pin;
    uint32_t dir_port;
    uint16_t step_pin;
    uint32_t step_port;
} motor;

#define STEPGEN_FREQ 1800000UL
//#define STEPS_PER_MM 200

//#define MM_PER_STEP (float)1.0/STEPS_PER_MM
//#define STEP_LEN 10 //step pulse length in 1.8MHz clock cycles

//function prototypes
void init_motor(void);
void set_motor_speed(float mm_per_sec);
void set_motor_speed_cnt(int32_t cnt_per_sec);
void position_control(void);






#endif
