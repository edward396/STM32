#include "main.h"

#define GPIOA_BASE_ADDR 0x40020000
#define GPIOD_BASE_ADDR 0x40020C00

void LedInit(){
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	*GPIOD_MODER &= ~(0b11 << 24);
	*GPIOD_MODER |= (0b01 << 24);
}

void ButtonInit(){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR+ 0x00);
	*GPIOA_MODER &= ~(0b11 << 0);
}

void LedCtrl(int on_off)
{
#if 0
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14);
	if(on_off == 1)
	{
		*GPIOD_ODR |= (1 << 12);
	}
	else
	{
		*GPIOD_ODR &= ~(1<<12);
	}
#else
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_BASE_ADDR + 0x18);
	if(on_off == 1)
	{
		*GPIOD_BSRR |= (1<<12);
	}
	else
	{
		*GPIOD_BSRR |= (1<<28);
	}
#endif
}

int main(){
	HAL_Init();
	LedInit();
	ButtonInit();
	while(1){
		LedCtrl(1);
		HAL_Delay(1000);
		LedCtrl(0);
		HAL_Delay(1000);
	}
	return 0;
}
