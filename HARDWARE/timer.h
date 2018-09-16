#ifndef __timer_h
#define __timer_h

#include "main.h"
#include "stm32f1xx_hal.h"

void tim6_delay_us(uint16_t us);
void tim6_delay_ms(uint16_t ms);
void tim6_delay_s(uint16_t s);

#endif
