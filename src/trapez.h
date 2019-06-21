
#ifndef TRAPEZ_H

#define TRAPEZ_H

#include <stdlib.h>
#include <stdio.h>
#include "types.h"
#include "globals.h"
void generate_trapez_motion(int32_t start_point, int32_t destination, float max_speed, float accel, int32_t steps_per_mm, float dt);
int32_t get_trapez_point(float dt);



#endif
