#include "main.h"

#define GPIOA_BASE_ADD 0x40020000
#define GPIOD_BASE_ADD 0X40020C00

void LedInit()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER= (uint32_t*)(GPIOD_BASE_ADD + 0x00);
	*GPIOD_MODER &= ~(0b11<<28);
	*GPIOD_MODER |= (0b01<<28);
	*GPIOD_MODER &= ~(0b11<<30);
	*GPIOD_MODER |= (0b01<<30);
}

void ButtonInit()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER= (uint32_t*)(GPIOA_BASE_ADD + 0x00);
	*GPIOA_MODER &= ~(0b11<<0);
}

int ButtonRead()
{
	uint32_t* GPIOA_IDR= (uint32_t*)(GPIOA_BASE_ADD + 0x10);
	uint32_t read= (*GPIOA_IDR & (1<<0));
	return read;
}

void LedCtrl(int on_off, int lednum)
{
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_BASE_ADD + 0x18);
	if (on_off == 1)
	{
		*GPIOD_BSRR = (1 << lednum);
	}
	else if (on_off==0) {
		*GPIOD_BSRR = (1 << (lednum + 16));
	}
}


int main()
{
	HAL_Init();
	LedInit();
	ButtonInit();
	ButtonRead();
	while(1)
	{
		LedCtrl(1,14);
		HAL_Delay(700);
		LedCtrl(0,14);
		HAL_Delay(700);
		LedCtrl(ButtonRead(),15);
	}
	return 0;
}
