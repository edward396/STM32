/*
 * RES_BASE_ADDR.h
 *
 *  Created on: April 13, 2025
 *      Author: truong
 */

#ifndef INC_RES_ADDR_H_
#define INC_RES_ADDR_H_

#include<stdio.h>
#include <stdint.h>

//Base Addresses for GPIO ports A-H
//UL is unsigned Long
#define GPIOA_BASE_ADDR 0x40020000UL
#define GPIOB_BASE_ADDR 0x40020400UL
#define GPIOC_BASE_ADDR 0x40020800UL
#define GPIOD_BASE_ADDR 0x40020C00UL
#define GPIOE_BASE_ADDR 0x40021000UL
#define GPIOH_BASE_ADDR 0x40021C00UL

//GPIO Registers Offset Value
typedef struct{
	volatile uint32_t MODER; 	//0x00 (Mode Register)
	volatile uint32_t OTYPER; 	//0x04 (Output Type Register)
	volatile uint32_t OSPEEDR; 	//0x08 (Output Speed Register)
	volatile uint32_t PUPDR;	//0x0C (Pull-up/pull-down Register)
	volatile uint32_t IDR;		//0x10 (Input Data Register)
	volatile uint32_t ODR;		//0x14 (Output Data Register)
	volatile uint32_t BSRR;		//0x18 (Bit Set/Reset Register)
}GPIO_Register_Offset_t;


//Create new definition and it is a pointer to a struct name GPIO_Register_Offset_t locate at base memory address
#define GPIOA_REG ((GPIO_Register_Offset_t*)GPIOA_BASE_ADDR)
#define GPIOB_REG ((GPIO_Register_Offset_t*)GPIOB_BASE_ADDR)
#define GPIOC_REG ((GPIO_Register_Offset_t*)GPIOC_BASE_ADDR)
#define GPIOD_REG ((GPIO_Register_Offset_t*)GPIOD_BASE_ADDR)
#define GPIOE_REG ((GPIO_Register_Offset_t*)GPIOE_BASE_ADDR)
#define GPIOH_REG ((GPIO_Register_Offset_t*)GPIOH_BASE_ADDR)


//EXTI Registers
#define EXTI_BASE_ADDR	0x40013C00UL

//External Interrupt Registers Offset Value
typedef struct{
	volatile uint32_t IMR; 		//0x00 (Interrupt Mask Register)
	volatile uint32_t EMR; 		//0x04 (Event Mask Register)
	volatile uint32_t RTSR;		//0x08 (Rising Trigger Selection Register)
	volatile uint32_t FTSR;		//0x0C (Falling Trigger Selection Register)
	volatile uint32_t SWIER; 	//0x10 (Software Interrupt Event Register)
	volatile uint32_t PR;		//0x14 (Pending Register)
}EXTI_Register_Offset_t;

//Create a new definition and it is a pointer to a struct name EXTI_Register_Offset_t locate at base memory address
#define EXTI_REG ((EXTI_Register_Offset_t*)EXTI_BASE_ADDR)

//NVIC Registers Cortex-M4
#define NVIC_ISER0 ((volatile uint32_t*)(0xE000E100))

#endif /* INC_RES_ADDR_H_ */
