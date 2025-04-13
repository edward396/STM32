/*
 * buttons.h
 *
 *  Created on: April 13, 2025
 *      Author: truong
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include <registerAddress.h>
#include"stm32f4xx_hal.h"

void buttonB1Init();
char buttonState();
void buttonInterrupt();



#endif /* INC_BUTTONS_H_ */
