/*
 * LEDs.c
 *
 *  Created on: Apr 13, 2025
 *      Author: truon
 */

#include <registerAddress.h>
#include "LEDs.h"

//Initialize LEDs
void LED_Green_Init(){ //PD12
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Enable RCC Clock
	GPIOD_REG -> MODER &= ~(0b11 << 24);
	GPIOD_REG -> MODER |= (0b01 << 24);
}

void LED_Orange_Init(){ //PD13
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Enable RCC Clock
	GPIOD_REG -> MODER &= ~(0b11 << 26);
	GPIOD_REG -> MODER |= (0b01 << 26);
}

void LED_Red_Init(){
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Enable RCC Clock
	GPIOD_REG -> MODER &= ~(0b11 << 28);
	GPIOD_REG -> MODER |= (0b01 << 28);
}

void LED_Blue_Init(){
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Enable RCC Clook
	GPIOD_REG -> MODER &= ~(0b11 << 30);
	GPIOD_REG -> MODER |= (0b01 << 30);
}

//Activate LEDs
void LED_Control(LED_t LED_Color, int on_off){
	if (on_off == 1){ //Turn on the LED
		GPIOD_REG -> BSRR = (1 << LED_Color);
	}
	else{
		GPIOD_REG -> BSRR = (1 << (LED_Color + 16)); //Otherwise turn off the LED
	}
}
