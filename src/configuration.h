#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdlib.h>
#include <stdio.h>

typedef struct
{
    float steps_per_mm;
    uint32_t spindle_ppr;
    uint16_t step_len;
    uint16_t invert_dir;

    float max_speed; //mm/s
    float max_accel; //mm/s^2
    float max_stepgen_accel; //mm/s^2

    int32_t config_stored;

} configuration;

static uint32_t flash_program_data(uint32_t start_address, uint8_t *input_data, uint16_t num_elements);
static void flash_read_data(uint32_t start_address, uint16_t num_elements, uint8_t *output_data);
void load_config(void);


#endif
