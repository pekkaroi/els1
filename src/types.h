#ifndef TYPES_H
#define TYPES_H
typedef enum {
    IDLE=0,
    PENDING=1,
    JOG=2,
    ACCELERATE=3,
    FOLLOW=4,
    DECELERATE=5,
    DONE=6,
    RETURN=7,

} motion_status_t;

typedef enum {
    BASIC=0,
    THREADING=1,
    INDEXING=2,
} operation_mode_t;

#define ERROR_TOO_HIGH_SPEED_REQUIRED (0)
#define ERROR_TOO_HIGH_ACCEL_REQUIRED (1)
#define ERROR_OPERATION_NOT_COMPLETE (2)

#define RAISE_ERROR(x) error_state|=(1<<x);
#define CLEAR_ERROR(x) error_state&=~(1<<x);


#endif
