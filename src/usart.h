#ifndef USART_H
#define USART_H
#include "types.h"
#include "motor.h"
#include "globals.h"
void usart_setup(void);
int _write(int file, char *ptr, int len);
void handleUsart(void);
bool startsWith(const char *pre, const char *str);
void prepare_status_package(void);

extern volatile int transfered;
extern volatile float spindle_speed;

#endif
