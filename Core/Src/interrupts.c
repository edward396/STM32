/*
 * interrupts.c
 *
 *  Created on: Apr 13, 2025
 *      Author: truon
 */

#include"interrupts.h"

void EXTI0_Init(){
	EXTI_REG -> RTSR |= (1 << 0);
	EXTI_REG -> FTSR |= (1 << 0);
	EXTI_REG -> IMR |= (1 << 0);
	*NVIC_ISER0 |= (1 << 6);
}
