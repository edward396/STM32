/*
 * LEDs.h
 *
 *  Created on: April 13, 2025
 *      Author: truon
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

#include <registerAddress.h>
#include "stm32f4xx_hal.h"

//#define LED3_PIN 12 //PD12
//#define LED4_PIN 13 //PD13
//#define LED5_PIN 14 //PD14
//#define LED6_PIN 15 //PD15

typedef enum{
	LED_Green = 12,
	LED_Orange,
	LED_Red,
	LED_Blue
}LED_t;

//Functions Declaration
void LED_Green_Init();
void LED_Orange_Init();
void LED_Red_Init();
void LED_Blue_Init();

void LED_Control(LED_t LED_pin, int on_off);

#endif /* INC_LEDS_H_ */
