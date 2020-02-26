
#include "configuration.h"
#include "globals.h"
#include <libopencm3/stm32/flash.h>


#define FLASH_OPERATION_ADDRESS ((uint32_t)0x0800f000)
#define FLASH_PAGE_NUM_MAX 127
#define FLASH_PAGE_SIZE 0x800
#define FLASH_WRONG_DATA_WRITTEN 0x80
#define RESULT_OK 0


void load_config(void)
{
    //load configuration from flash
    flash_read_data(FLASH_OPERATION_ADDRESS, sizeof(c), &c);
    if(c.config_stored != 0xAAAA)
    {
        //non-initialized flash found. write defaults
        c.steps_per_mm = 800;
        c.spindle_ppr = 4800;
        c.step_len = 10;
        c.invert_dir = 1;
        c.max_speed = 20;
        c.max_accel=500;
        c.max_stepgen_accel = 700;
        c.config_stored = 0xAAAA;
        flash_program_data(FLASH_OPERATION_ADDRESS, &c, sizeof(c));
    }

}

static uint32_t flash_program_data(uint32_t start_address, uint8_t *input_data, uint16_t num_elements)
{
	uint16_t iter;
	uint32_t current_address = start_address;
	uint32_t page_address = start_address;
	uint32_t flash_status = 0;

	/*check if start_address is in proper range*/
	if((start_address - FLASH_BASE) >= (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM_MAX+1)))
		return 1;

	/*calculate current page address*/
	if(start_address % FLASH_PAGE_SIZE)
		page_address -= (start_address % FLASH_PAGE_SIZE);

	flash_unlock();

	/*Erasing page*/
	flash_erase_page(page_address);
	flash_status = flash_get_status_flags();
	if(flash_status != FLASH_SR_EOP)
		return flash_status;

	/*programming flash memory*/
	for(iter=0; iter<num_elements; iter += 4)
	{
		/*programming word data*/
		flash_program_word(current_address+iter, *((uint32_t*)(input_data + iter)));
		flash_status = flash_get_status_flags();
		if(flash_status != FLASH_SR_EOP)
			return flash_status;

		/*verify if correct data is programmed*/
		if(*((uint32_t*)(current_address+iter)) != *((uint32_t*)(input_data + iter)))
			return FLASH_WRONG_DATA_WRITTEN;
	}

	return 0;
}

static void flash_read_data(uint32_t start_address, uint16_t num_elements, uint8_t *output_data)
{
	uint16_t iter;
	uint32_t *memory_ptr= (uint32_t*)start_address;

	for(iter=0; iter<num_elements/4; iter++)
	{
		*(uint32_t*)output_data = *(memory_ptr + iter);
		output_data += 4;
	}
}
