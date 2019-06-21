#include "trapez.h"
#include "math.h"
#define SHIFT 12
int32_t trapez_points[4]; //0=start, 1=end of acceleration, 2=start of deceleration, 3=end
float trapez_accel;
float trapez_max_speed;
float speed_increase_per_step;
int32_t sign;
int32_t current_point;
float current_speed;
int32_t trapez_valid=0;

void generate_trapez_motion(int32_t start_point, int32_t destination, float max_speed, float accel, int32_t steps_per_mm, float dt)
{

    trapez_points[0] = start_point<<SHIFT;
    current_point = trapez_points[0];
    current_speed = 0;
    trapez_points[3] = destination<<SHIFT;
    trapez_accel = accel * pow(2, SHIFT)*steps_per_mm; //steps/sec/sec
    trapez_max_speed = max_speed * pow(2, SHIFT)*steps_per_mm; //steps/sec
    speed_increase_per_step = trapez_accel*dt; //steps/sec
    int32_t total_steps = trapez_points[3] - trapez_points[0];
    sign = (total_steps > 0 ? 1:-1);
    int32_t required_accel_decel_steps = (int32_t)(0.5 * trapez_max_speed*trapez_max_speed/trapez_accel);


    if (sign*total_steps < required_accel_decel_steps*2)
    {
        trapez_points[1] = trapez_points[0]+total_steps/2;
        trapez_points[2] = trapez_points[1];
    }
    else
    {
        trapez_points[1] = trapez_points[0] + sign*required_accel_decel_steps;
        trapez_points[2] = trapez_points[3] - sign*required_accel_decel_steps;
    }
    trapez_valid=1;

}
int32_t get_trapez_point(float dt)
{
    if(trapez_valid != 1)
    {
        return current_point;
    }
    //accelerating motion
    if(sign*current_point < sign*trapez_points[1] && current_speed < trapez_max_speed)
    {
        current_speed += speed_increase_per_step*dt;
        if(current_speed>trapez_max_speed)
            current_speed = trapez_max_speed;
    }
    else if (sign*current_point > sign*trapez_points[2])
    {
        current_speed -= speed_increase_per_step*dt;
        if(current_speed < 0)
            current_speed = 0;
    }

    if(sign*current_point >= sign*trapez_points[3])
    {
        current_speed = 0;
        current_point = trapez_points[3];
        status =IDLE;
        return current_point >> SHIFT;

    }
    if(current_speed > 0)
        current_point += sign*current_speed;
    else
        current_point += sign; //increase at least by one count
    return current_point>>SHIFT;

}
