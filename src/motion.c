
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <stdlib.h>
#include <stdio.h>

#include "motor.h"

#include "motion.h"

#include "filter.h"
#include "types.h"
#include "trapez.h"
#include "globals.h"

volatile int32_t spindle_count;
volatile int16_t spindle_relative_pos;
volatile int32_t spindle_prev_count;
volatile uint16_t oldcount;

volatile int32_t spindle_acceleration_target;
volatile int32_t spindle_acceleration_start;

volatile int32_t acceleration_start_point;
volatile int32_t follow_target;
volatile float max_error; //debug


//RPM estimator filter
#define IIR_ORDER     4
#define IIR_NUMSTAGES (IIR_ORDER/2)

static float32_t m_biquad_state[IIR_ORDER];
static float32_t m_biquad_coeffs[5*IIR_NUMSTAGES] =
{
    2.4136e-08,
    4.8272e-08,
    2.4136e-08,
    1.9540e+00,
   -9.5462e-01,
    1.0000e+00,
    2.0000e+00,
    1.0000e+00,
    1.9803e+00,
   -9.8095e-01
};

arm_biquad_cascade_df2T_instance_f32 const iir_inst =
{
  IIR_ORDER/2,
  m_biquad_state,
  m_biquad_coeffs
};



volatile float spindle_speed; //rev per SECOND!
filter1Type* filt;

float alpha;
void init_motion()
{
    oldcount = 0;
    spindle_count = 0;
    spindle_prev_count = 0;
    spindle_relative_pos = 0;
    spindle_speed = 0.0;
    filt = filter1_create();
}
void get_spindle_position()
{
    #ifdef SIMULATE_ENCODER
        timer_set_counter(TIM4, timer_get_counter(TIM4)+1);
    #endif
	uint16_t now = timer_get_counter(TIM4);
	int16_t delta =  (int16_t)(now - oldcount);
    oldcount = now;
    spindle_count += delta;
/*
    int16_t shaft_pos_tmp = spindle_count % SPINDLE_PPR;
    if(shaft_pos_tmp < 0)
        spindle_relative_pos = SPINDLE_PPR + shaft_pos_tmp;
    else
        spindle_relative_pos = shaft_pos_tmp;
*/
}

void rpm_estimator()
{
    float inst_speed = ((float)(spindle_count-spindle_prev_count))/c.spindle_ppr*LOOP_FREQ;
    arm_biquad_cascade_df2T_f32(&iir_inst, &inst_speed, &spindle_speed, 1);
    //filter1_writeInput(filt, inst_speed);
    spindle_prev_count = spindle_count;
    //spindle_speed = filter1_readOutput(filt);

}
void start_acceleration()
{
    status = PENDING;
    float target_speed = spindle_speed * mm_per_revolution;
    if(target_speed > c.max_speed*0.8)
    {
        printf("Too high speed required");
        RAISE_ERROR(ERROR_TOO_HIGH_SPEED_REQUIRED)
    }
    acceleration_start_point = m1.pos_accu;
    int32_t required_motor_steps = reference_pos - acceleration_start_point;
    float accel_distance = (float)(required_motor_steps)/(float)c.steps_per_mm;
    float acceleration = 0.5*target_speed*target_speed/accel_distance;
    if (acceleration > c.max_accel*0.8)
    {
        printf("Too high acceleration required");
    }

    float required_time = target_speed/acceleration;
    //float required_accel_distance = 0.5*target_speed*required_time;
    int64_t required_spindle_steps = (int64_t)(required_time*spindle_speed*c.spindle_ppr);

    //on acceleration, motor_pos(spindle_pos) = alpha*spindle_pos^2 -> second order function of position -> linear velocity increase -> constant acceleration
    alpha = (float)required_motor_steps/(float)(required_spindle_steps*required_spindle_steps);

    spindle_acceleration_target = ((spindle_count+required_spindle_steps+c.spindle_ppr-1)/c.spindle_ppr)*(c.spindle_ppr); //next full round
    spindle_acceleration_start = spindle_acceleration_target - required_spindle_steps;
    int16_t steps_before_start = spindle_acceleration_start - spindle_count;
    //setup an interrupt for encoder timer to start accelerating
    timer_set_oc_value(TIM4, TIM_OC3, oldcount + steps_before_start);
    timer_clear_flag(TIM4, TIM_SR_CC3IF);
    timer_enable_irq(TIM4, TIM_DIER_CC3IE);

    follow_target = reference_pos + (int32_t)(operation_length*c.steps_per_mm);
    /*printf("starting acceleration in %d steps\n\r \
target speed %li\n\r \
acceleration %li\n\r \
accel distance %li\n\r \
required_time %li\n\r \
required spindle steps %li\n\r \
", steps_before_start, (long int)(target_speed*1000), (long int)(acceleration*1000), (long int)(accel_distance*1000), (long int)(required_time*1000), required_spindle_steps);
printf ("spindle_acceleration_target %li \n\r \
spindle_acceleration_start %li \n\r \
set interrupt at %u \n\r \
oldcount %u \n\r \
    ",spindle_acceleration_target, spindle_acceleration_start, oldcount + steps_before_start, oldcount);
printf("current spindle count %li \n\r", (long int)spindle_count);
printf("alpha %li \n\r", (long int)(alpha*1000000000));*/
}
void accelerate()
{

    volatile int32_t position_now = acceleration_start_point + (int32_t)(alpha*(spindle_count-spindle_acceleration_start)*(spindle_count-spindle_acceleration_start));
    m1.pos_target = position_now;
    position_control();



}
void follow()
{
    int32_t current_pos = reference_pos + (int32_t)((float)(spindle_count-spindle_acceleration_target)/(float)c.spindle_ppr*mm_per_revolution*c.steps_per_mm);
    m1.pos_target = current_pos;
    position_control();
}
void decelerate()
{
    //stop position control, decelerate as fast as possible
    float max_delta = c.max_accel*LOOP_PER;
    if(m1.cur_speed >= 0)
    {
        if(m1.cur_speed > max_delta)
            set_motor_speed(m1.cur_speed - max_delta);
        else
        {
            set_motor_speed(0.0);
            status = IDLE;
        }
    }
    else if(m1.cur_speed < 0)
    {
        if(m1.cur_speed < -max_delta)
            set_motor_speed(m1.cur_speed + max_delta);
        else
        {
            set_motor_speed(0.0);
            status = IDLE;
        }
    }
}
void return_to_start()
{
    if(status==DONE || status == IDLE)
    {

        float acceleration_distance = 0.5 * (spindle_speed*mm_per_revolution)*(spindle_speed*mm_per_revolution)/(c.max_accel*0.5);
        acceleration_start_point = reference_pos - (int32_t)(acceleration_distance*c.steps_per_mm);

        generate_trapez_motion(m1.pos_accu, acceleration_start_point, c.max_speed*0.5, c.max_accel*0.5, c.steps_per_mm, LOOP_PER);

        status = RETURN;

    }


    if (status == RETURN)
    {
        int32_t new_pos =get_trapez_point(LOOP_PER);
        m1.pos_target = new_pos;
        position_control();
    }
    if(status == IDLE)
        position_control();
}
void tim3_isr(void)
{
    if(timer_get_flag(TIM3, TIM_SR_UIF))
    {
        timer_clear_flag(TIM3, TIM_SR_UIF);
        get_spindle_position();
        rpm_estimator();
        switch (status) {
        case ACCELERATE:

            if(spindle_count >= spindle_acceleration_target)
            {
                status = FOLLOW;
                //printf ("Acceleration complete. Current pos: %li, should be %li. Max error %d\n\r", (long int)m1.pos_accu, acceleration_start_point + (int32_t)(alpha*(spindle_count-spindle_acceleration_start)*(spindle_count-spindle_acceleration_start)), (int16_t)(max_error*1000));
                break;
            }
            accelerate();
            break;

        case FOLLOW:
            if(mode == THREADING)
            {
                if(m1.pos_accu >= follow_target)
                {
                    status = DECELERATE;
                    //printf("Operation complete. Max error %d\n\r", (int)(max_error*1000));
                    break;
                }
            }
            follow();
            break;
        case DECELERATE:
            decelerate();
            break;
        case RETURN:
            return_to_start();
            break;
        case IDLE:
            position_control();
        case PENDING:
            position_control();
        default:

            break;
        }
        gpio_toggle(GPIOA, GPIO12);

    }
}

void tim4_isr(void)
{
	if (timer_get_flag(TIM4, TIM_SR_CC3IF)) {
        timer_clear_flag(TIM4, TIM_SR_CC3IF);
        if(status == PENDING)
        {

        //printf("Started acceleration\n\r");
        max_error=0;
        status = ACCELERATE;

        timer_disable_irq(TIM4, TIM_DIER_CC3IE);
        }

    }
}
