#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>
#include <stdio.h>
#include "motor.h"
#include "globals.h"
#include "math.h"
#include "motion.h"

float mm_per_step;

void change_direction(int8_t dir);

void init_motor()
{
    m1.pos_accu = 0;
    m1.ref_pos = 0;
    m1.direction = -1;
    m1.step_port = GPIOA;
    m1.dir_port = GPIOB;
    m1.step_pin = GPIO8;
    m1.dir_pin = GPIO15;

    //m1.max_speed = 150;
    m1.max_speed_cnt = c.max_speed*c.steps_per_mm;
    //m1.max_accel = 500;
    //m1.max_stepgen_accel = 700;
    m1.max_stepgen_accel_cnt = c.max_stepgen_accel*c.steps_per_mm;
    //m1.max_stepgen_accel_per_period = m1.max_stepgen_accel*LOOP_PER;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(m1.step_port, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, m1.step_pin);
    gpio_set_mode(m1.dir_port, GPIO_MODE_OUTPUT_50_MHZ,
                                GPIO_CNF_OUTPUT_PUSHPULL, m1.dir_pin);


    //Setup TIM1 for step generator. Prescaled clock should be 1.8MHz
    rcc_periph_clock_enable(RCC_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 39);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM1, TIM_OC1, c.step_len);

    timer_set_oc_slow_mode(TIM1,TIM_OC1);
    timer_set_oc_polarity_high(TIM1,TIM_OC1);
    timer_set_oc_idle_state_unset(TIM1,TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 0);
    nvic_set_priority(NVIC_TIM1_UP_IRQ, 0);
    nvic_enable_irq(NVIC_TIM1_UP_IRQ);
    timer_enable_irq(TIM1, TIM_DIER_UIE);

    timer_enable_counter(TIM1);
}

void tim1_up_isr(void)
{
    if(timer_get_flag(TIM1, TIM_SR_UIF))
    {
        m1.pos_accu += m1.direction;
        timer_clear_flag(TIM1, TIM_SR_UIF);

    }
}

void position_control(void)
{
    static int64_t old_target = 0;
    int32_t new_vel;
    int32_t vel_cmd = (m1.pos_target - old_target) * LOOP_FREQ; // ds/dt = velocity in step/sec
    old_target = m1.pos_target;

    //try to match the error at the end of next cycle
    int32_t velocity_to_fix_error;
    if(abs(m1.pos_target-m1.pos_accu) > 5)
    {
        velocity_to_fix_error = (m1.pos_target-m1.pos_accu)*LOOP_FREQ; // pos_error/dt == pos_error*freq
    }
    else
    {
        velocity_to_fix_error = 0;
    }


    int32_t velocity_delta = vel_cmd - m1.cur_speed_cnt + velocity_to_fix_error;
    if(velocity_delta > m1.max_stepgen_accel_cnt)
        velocity_delta = m1.max_stepgen_accel_cnt;
    else if(velocity_delta < -m1.max_stepgen_accel_cnt)
        velocity_delta = -m1.max_stepgen_accel_cnt;

    new_vel = m1.cur_speed_cnt + velocity_delta;
    if(new_vel > m1.max_speed_cnt)
        new_vel = m1.max_speed_cnt;
    else if (new_vel < -m1.max_speed_cnt)
        new_vel  = -m1.max_speed_cnt;

    set_motor_speed_cnt(new_vel);


}
/*
void position_control(float dt)
{



    static float prev_error = 0;
    static float integrated_error = 0;

    static int32_t prev_count = 0;

    float error = (float)(m1.pos_target-m1.pos_accu)/(float)STEPS_PER_MM;
    if(error>max_error)
        max_error=error;
    float out;

    float P = 100.0;
    float I = 0.0;
    float D = 0;
    float FF1 = 0;


    out = P*error;
    integrated_error += I*error;
    if(integrated_error > m1.max_speed)
        integrated_error = m1.max_speed;
    else if (integrated_error < -m1.max_speed)
        integrated_error = -m1.max_speed;
    out += integrated_error;
    out += D*(error-prev_error);
    out += FF1*(float)(m1.pos_target-prev_count)/(float)(STEPS_PER_MM);
    prev_error = error;
    prev_count = m1.pos_target;



    if(out>m1.max_speed)
        out = m1.max_speed;
    else if(out < -m1.max_speed)
        out = -m1.max_speed;

    float acc = fabs(out - m1.cur_speed);
    float max_delta = m1.max_stepgen_accel*dt;
    if(acc > max_delta)
    {
        if(out > m1.cur_speed)
            out = m1.cur_speed + max_delta;
        else
            out = m1.cur_speed - max_delta;
    }
    set_motor_speed(out);


}
*/

void change_direction(int8_t dir)
{
    if(dir>0)
    {
        if(c.invert_dir)
            gpio_clear(m1.dir_port, m1.dir_pin);
        else
            gpio_set(m1.dir_port, m1.dir_pin);
    }
    else
    {
        if(c.invert_dir)
            gpio_set(m1.dir_port, m1.dir_pin);
        else
            gpio_clear(m1.dir_port, m1.dir_pin);
    }
    m1.direction = dir;

}
void set_motor_speed(float mm_per_sec)
{
    int i;


    /* the unit of step period is one 1.8MHz clock period.
    steps_per_sec = mm_per_min*STEPS_PER_MM / 60
    step_period_sec = 1/steps_per_sec
    step_period_clocks = step_period_sec*1.8e6
    */
    if( (mm_per_sec > 0 && m1.direction<0) || (mm_per_sec < 0 && m1.direction > 0))
        change_direction(mm_per_sec >= 0 ? 1 : -1);
    float step_period_clocks_f;
    if(fabs(mm_per_sec) > 0.0001)
        step_period_clocks_f = 1.0/(fabs(mm_per_sec)*c.steps_per_mm)*1.8e6-1;
    else step_period_clocks_f = 65537;

    if (step_period_clocks_f > 65536)
    {
        //too slow. Stop the motor
        m1.step_period = 0;

    }
    else if (step_period_clocks_f < c.step_len*2)
    {
        //too fast. Go max speed
        m1.step_period = c.step_len*2;

    }
    else m1.step_period = (uint16_t)step_period_clocks_f;


    if((m1.step_period > 0) && timer_get_counter(TIM1) > m1.step_period)
        timer_set_counter(TIM1, m1.step_period-2);


    if(m1.step_period == 0)
    {
        timer_disable_counter(TIM1);
        timer_set_counter(TIM1, c.step_len+1);
    }
    else
    {
        timer_set_period(TIM1, m1.step_period);
        timer_enable_counter(TIM1);
    }

    m1.cur_speed = mm_per_sec;
}
void set_motor_speed_cnt(int32_t cnt_per_sec)
{
    int i;


    /* the unit of step period is one 1.8MHz clock period.*/


    uint32_t step_period_clocks;
    if(cnt_per_sec != 0)
        step_period_clocks = STEPGEN_FREQ/abs(cnt_per_sec)-1;
    else step_period_clocks = 65537;

    int32_t cur_speed = STEPGEN_FREQ/(step_period_clocks+1);
    if(cnt_per_sec < 0)
        cur_speed = -cur_speed;


    if (step_period_clocks > 65536)
    {
        //too slow. Stop the motor
        m1.step_period = 0;

    }
    else if (step_period_clocks < c.step_len*2)
    {
        //too fast. Go max speed
        m1.step_period = c.step_len*2;

    }
    else m1.step_period = (uint16_t)step_period_clocks;

    if(m1.step_period > 0 )
    {
        if( (cnt_per_sec > 0 && m1.direction<0) || (cnt_per_sec < 0 && m1.direction > 0))
            change_direction(cnt_per_sec >= 0 ? 1 : -1);
    }

    if((m1.step_period > 0) && timer_get_counter(TIM1) > m1.step_period)
        timer_set_counter(TIM1, m1.step_period-2);


    if(m1.step_period == 0)
    {
        timer_disable_counter(TIM1);
        timer_set_counter(TIM1, c.step_len+1);
    }
    else
    {
        timer_set_period(TIM1, m1.step_period);
        timer_enable_counter(TIM1);
    }

    m1.cur_speed_cnt = cur_speed;
}
