#include <stdio.h>
#include "LEDs.h"
#include "buttons.h"
#include "registerAddress.h"
#include "interrupts.h"
#include "l3gd20.h"

int LED_Delay = 300;

void EXTI0_IRQHandler(){
	if (buttonState()){
		LED_Control(LED_Green, 1);
	}
	else{
		LED_Control(LED_Green, 0);
	}
	EXTI_REG -> PR = (1 << 0); //Clear the bit so there is no pending flag
}

int main(void){
	HAL_Init();
	EXTI0_Init();
	LED_Green_Init();
	LED_Red_Init();
	buttonB1Init();

	while(1){
		LED_Control(LED_Red, 1);
		HAL_Delay(LED_Delay);
		LED_Control(LED_Red, 0);
		HAL_Delay(LED_Delay);
	}
}
