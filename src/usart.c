#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usart.h"
#include "motion.h"
#include "globals.h"

#define USART_BUF_LEN 100

char usartBuf[USART_BUF_LEN];
char outbuf[USART_BUF_LEN];
uint8_t usartCtr;

volatile uint8_t config_requested;



static void dma_write(char *data, int size)
{
	/*
	 * Using channel 4 for USART1_TX
	 */

	/* Reset DMA channel*/
	dma_channel_reset(DMA1, DMA_CHANNEL4);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

	dma_enable_channel(DMA1, DMA_CHANNEL4);

    usart_enable_tx_dma(USART1);
    transfered = 0;
}


void dma1_channel4_isr(void)
{
	if ((DMA1_ISR &DMA_ISR_TCIF4) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF4;


	}
    transfered = 1;
	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

	usart_disable_tx_dma(USART1);

	dma_disable_channel(DMA1, DMA_CHANNEL4);
}

void prepare_status_package()
{
    int i, cnt;
    if(!config_requested)
    {
        outbuf[0] = 0x01; //type is status
        cnt = sprintf(outbuf, "%d, %d, %li, %.4f, %.4f, %.4f, %li\n", mode, status, m1.pos_accu, mm_per_revolution, operation_length, spindle_speed, reference_pos);
        dma_write(outbuf, cnt);
        return;
    }
    else
    {
        outbuf[0] = 0x02;
        uint8_t *ptr = &c;
        for (i = 1; i < 1+sizeof(c); i++)
        {
                outbuf[i] = *(ptr+i);
        }
        outbuf[sizeof(c)+1] = 0xFF;
        outbuf[sizeof(c)+2] = 0xFF;
        outbuf[sizeof(c)+3] = 0xFF;
        dma_write(outbuf, sizeof(c)+4);
        config_requested = 0;
    }

}


//custom implementation for printf
//this is a blocking method. Not to be used for real-time operation
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {

        for(i = 0; i<len; i++)
            usart_send_blocking(USART1, ptr[i]);

		return i;
	}

	errno = EIO;
	return -1;
}
bool startsWith(const char *pre, const char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}


void handleUsart()
{
    char *d = ",";
    int index_count=0;
    char *ptr  =strtok(usartBuf, d);
    if(ptr == NULL)
        return;
    if(!strcmp(ptr, "SET_MODE"))
    {
        if(status != IDLE)
            return;
        ptr = strtok(NULL, d);
        if(!strcmp(ptr, "THREAD"))
        {
            mode = THREADING;
            return;
        }
        if(!strcmp(ptr, "BASIC"))
        {
            mode = BASIC;
            return;
        }
        if(!strcmp(ptr, "INDEX"))
        {
            mode = INDEXING;
            return;
        }
    }
    if(!strcmp(ptr, "SET_REF_POS"))
    {

        if(status != IDLE)
            return;
        if(mode != INDEXING)
            {ptr = strtok(NULL, d);
            if(ptr != NULL)
            {
                reference_pos = (int32_t)(atof(ptr) * c.steps_per_mm + m1.pos_accu);
            }
        }
        else
        {
            //we want to set the zero position for indexinghere.
            //roughest way to do that is to zero spindle counter
            //setting spindle count here - outside of the interrupt is probably dangerous really
            spindle_count = 0;
            index_count = 0;

        }


    }
    if(!strcmp(ptr, "SET_PITCH"))
    {
        if(status != IDLE)
            return;
        ptr = strtok(NULL, d);
        if(ptr != NULL)
        {
            mm_per_revolution = atof(ptr);
            return;
        }


    }
    if(!strcmp(ptr, "SET_LENGTH"))
    {
        if(status != IDLE)
            return;
        ptr = strtok(NULL, d);
        if(ptr != NULL)
        {
            operation_length = atof(ptr);
            return;
        }


    }
    if(!strcmp(ptr, "START"))
    {
        if(status != IDLE)
            return;
        if(mode == THREADING)
            start_acceleration();
        if(mode == BASIC)
        {
            spindle_acceleration_target = spindle_count;
            reference_pos = m1.pos_accu;
            status = FOLLOW;
        }

    }
    if(!strcmp(ptr, "RETURN"))
    {
        if(status != IDLE && status != DONE)
            return;
        if(mode == THREADING)
            return_to_start();
    }
    if(!strcmp(ptr, "STOP"))
    {
        status = IDLE; //stop where-ever you are.
    }
    if(!strcmp(ptr, "NEXT"))
    {
        if(mode != INDEXING)
            return;
        index_count++;
    }
    if(!strcmp(ptr, "PREV"))
    {
        if(mode != INDEXING)
            return;
        index_count--;
    }
    if(!strcmp(ptr, "GET_CONFIG"))
    {
        config_requested = 1;
    }

}
void usart_setup(void)
{

    usartCtr=0;
    transfered = 1;
    config_requested = 0;
    /* Enable the USART1 interrupt. */
    nvic_set_priority(NVIC_USART1_IRQ, 4);
	nvic_enable_irq(NVIC_USART1_IRQ);

	nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 4);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
  	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
  		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
}

void usart1_isr(void)
{
    while ((USART_SR(USART1) & USART_SR_RXNE) != 0)
    {
        usartBuf[usartCtr] = usart_recv(USART1);
        if (usartBuf[usartCtr] == '\n' || usartBuf[usartCtr] == 13)
        {
            handleUsart();
            usartCtr=0;
        }
        else if(usartCtr < USART_BUF_LEN-2)
            usartCtr++;


    }
}
