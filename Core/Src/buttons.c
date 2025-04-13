/*
 * buttons.c
 *
 *  Created on: Apr 13, 2025
 *      Author: truon
 */

#include "buttons.h"
#include "LEDs.h"

void buttonB1Init(){ //Initialize the button
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Enable RCC Clock
	GPIOA_REG -> MODER &= ~(0b11 << 0); //Set the pin A0 as inpupt

	GPIOA_REG -> PUPDR &= ~(0b11 << 0); //Reset bits 0 and 1 of PUPDR
	GPIOA_REG -> PUPDR |= (0b10 << 0); //Set this pin as a pull-down config
}

char buttonState(){
	return (GPIOA_REG -> IDR & 0x01); //return 1 for high and 0 for low
}

void buttonInterrupt(){
	if(buttonState()){
		//The button is pressed
		LED_Control(LED_Green, 1);
	}
	else{
		LED_Control(LED_Green, 0);
	}
}
