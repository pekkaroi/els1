
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <stdlib.h>
#include <stdio.h>


#include "configuration.h"
#include "motor.h"
#include "motion.h"
#include "types.h"
#include "trapez.h"
#include "usart.h"

volatile motion_status_t status;
motor m1;
configuration c;
volatile operation_mode_t mode;
volatile uint16_t error_state;


volatile float mm_per_revolution;
volatile float operation_length; //mm.
volatile int32_t reference_pos; //start of operation point on threading.

volatile int transfered;

/* Set STM32 to 72 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

	rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
}


static void timer_setup(void)
{

    //Setup TIM4 to read encoder.
    rcc_periph_clock_enable(RCC_TIM4);
    timer_set_period(TIM4, 65535);
    timer_slave_set_mode(TIM4, 0x3); // encoder
    timer_ic_set_input(TIM4, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM4, TIM_IC2, TIM_IC_IN_TI2);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
    		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO6|GPIO7);
    gpio_set(GPIOB, GPIO6|GPIO7);
    nvic_set_priority(NVIC_TIM4_IRQ, 4);
    nvic_enable_irq(NVIC_TIM4_IRQ);

    timer_disable_preload(TIM4);
    timer_disable_oc_preload(TIM4,TIM_OC3);
    timer_continuous_mode(TIM4);
    timer_enable_counter(TIM4);

    //Setup TIM3 for loop timer at 5kHz
    rcc_periph_clock_enable(RCC_TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM3, (36*400)-1);
    timer_enable_counter(TIM3);
    nvic_set_priority(NVIC_TIM3_IRQ, 4);
    nvic_enable_irq(NVIC_TIM3_IRQ);
    timer_enable_irq(TIM3, TIM_DIER_UIE);

}

int main(void)
{
    int i;
    load_config();
    init_motion();
    status = IDLE;
    mode = THREADING;
    error_state = 0;
	clock_setup();
    timer_setup();
    usart_setup();
    init_motor();

    reference_pos = 50;
    mm_per_revolution = 0.1;
    operation_length = 50.0;

    for (i = 0; i < 8000; i++)
    __asm__("nop");
    transfered=1;

    while (1) {
        //printf("motor pos: %d\n\r", (int)(spindle_count));

        if(transfered)
            prepare_status_package();
	}

	return 0;
}
