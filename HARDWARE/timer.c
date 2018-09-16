#include "timer.h"

extern TIM_HandleTypeDef htim6;

void tim6_delay_us(uint16_t us)
{
	uint16_t differ = 0xffff-us-5;
	
	HAL_TIM_Base_Start(&htim6);
	
	__HAL_TIM_SET_COUNTER(&htim6,differ);
	
	while(differ < 0xffff-5)
	{
		differ = __HAL_TIM_GET_COUNTER(&htim6);
	}
	
	HAL_TIM_Base_Stop(&htim6);
}

void tim6_delay_ms(uint16_t ms)
{
	while(ms)
	{
		tim6_delay_us(1000);
		ms--;
	}
}

void tim6_delay_s(uint16_t s)
{
	while(s)
	{
		tim6_delay_ms(1000);
		s--;
	}
}
