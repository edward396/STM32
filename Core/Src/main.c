
//#define DMA2_BASE_ADDR 0x40026400  //fixme

//char rx_buf[32];
//void DMA_Init(){
//	//use DMA 2, stream 2, channel 4 for UART1_RX
//
//	//Peripheral address register
//	uint32_t* DMA_S2PAR = (uint_32*) (DMA2_BASE_ADDR + 0x18 + 0x18 * 2);
//	*DMA_S2PAR = 0x40011004;
//
//	//Memory address register
//	uint32_t* DMA_M0AR = (uint_32*) (DMA2_BASE_ADDR + 0x1C + 0x18 * 2);
//	*DMA_M0AR = rx_buf;
//
//	//Number of data registers
//	uint32_t* DMA_S2NDTR = (uint_32*) (DMA2_BASE_ADDR + 0x14 + 0x18 * 2);
//	*DMA_S2NDTR = sizeof(rx_buf);
//
//	//Enable DMA 2, stream 2, channel 4
//	uint32_t* DMA_S2CR = (uint_32*) (DMA2_BASE_ADDR + 0x10 + 0x18 * 2);
//	*DMA_S2CR &= ~(0b111 << 25);
//	*DMA_S2CR |= (0b1000 << 25); //Select channel 4 for stream 2
//	*DMA_S2CR |= (1 << 0); //Enable stream 2
//}

#include "main.h"
#include <string.h>

#define GPIOA_BASE_ADDR 0x40020000
#define GPIOD_BASE_ADDR 0X40020C00
#define GPIOE_BASE_ADDR 0x40021000
#define I2C1_BASE_ADDR 0x40005400
#define SPI1_BASE_ADDR 0x40013000

int cnt = 1000;

void LedInit()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER= (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	*GPIOD_MODER &= ~(0xff << 24);
	*GPIOD_MODER |= (0b01 << 24)| (0b01 << 26)|(0b01 << 28)|(0b01 << 30);
}

void ButtonInit()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER= (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &= ~(0b11<<0);
}

typedef enum {
	LED_GREEN = 12,
	LED_ORANGE,
	LED_RED,
	LED_BLUE
}LED_t;

void LedCtrl(LED_t led, int on_off)
{
#if 0
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14);
	if(on_off == 1)
	{
		*GPIOD_ODR |= (1 << led);
	}
	else
	{
		*GPIOD_ODR &= ~(1<<led);
	}
#else
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_BASE_ADDR + 0x18);
	if(on_off == 1)
	{
		*GPIOD_BSRR |= (1<<led);
	}
	else
	{
		*GPIOD_BSRR |= (1<<(led+16));
	}
#endif
}

char ButtonState()
{
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
	return (*GPIOA_IDR >> 0) & 1;
}

void function()
{
	if(ButtonState())
		LedCtrl(LED_RED, 1);
	else
		LedCtrl(LED_RED, 0);
}

#define EXTI_BASE_ADDR	0x40013C00

void EXTI0Init()
{
	/*
	- Làm sao để EXTI0 gửi interrupt signal lên ARM?
		+ chọn cạnh (rising/falling/cả hai)
			+ set trong thanh ghi EXTI_RTSR và EXTI_FTSR
		+ enable exti0(set mark)
			+ set trong thanh ghi EXTI_IMR
	- ARM (NVIC) phải chấp nhận interrupt signal từ EXTI gửi lên?
		+ bước 1: xác định EXTI0 nằm ở position bao nhiêu trong vector table? (mở vector table ở chapter "10: interrupts and events" trong reference manual) --> 6
		+ bước 2: enable interrupt cho position 6
	*/
	uint32_t* EXTI_RTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x08);
	*EXTI_RTSR |= (1<<0);
	uint32_t* EXTI_FTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x0C);
	*EXTI_FTSR |= (1<<0);
	uint32_t* EXTI_IMR = (uint32_t*)(EXTI_BASE_ADDR + 0x00);
	*EXTI_IMR |= (1<<0);

	uint32_t* NVIC_ISER0 = (uint32_t*)0xE000E100;
	*NVIC_ISER0 |= 1<<6;

	//Move vector table to RAM (0x20000000)
	uint8_t* src = 0;
	uint8_t* dis = 0x20000000;
	for(int i = 0; i < 0x198; i ++){ //0->0x198 is the size of the vector table
		*(dis + i) = *(src + i);
	}

	//Telling ARM that the vector table was moved
	uint32_t* VTOR = (uint32_t*) 0XE000ED08;
	*VTOR = 0x20000000;

	int *ptr;
	ptr = 0x58;
	*ptr = function;
}

void EXTI0_IRQHandler()
{
	if(ButtonState())
		LedCtrl(LED_BLUE, 1);
	else
		LedCtrl(LED_BLUE, 0);

	//clear interrupt flag
	uint32_t* EXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14);
	*EXTI_PR |= (1<<0);
}

#define UART1_BASE_ADDR	0x40011000
#define GPIOB_BASE_ADDR	0x40020400
void UART_Init()
{

	/* 	CONFIG GPIOB
		set PB6 as UART1_Tx, PB7 as UART1_Rx
		PB6 alternate function 07
		PB7 alternate function 07
	*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
	*GPIOB_MODER &= ~(0b1111 << 12);
	*GPIOB_MODER |= (0b10 << 12) | (0b10 << 14);

	uint32_t* GPIOB_AFLR = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	*GPIOB_AFLR &= ~(0xff << 24);
	*GPIOB_AFLR |= (0b0111 << 24) | (0b0111 << 28);

	/* 	CONFIG UART
		- set baud rate: 9600 (bps)
		- data frame:
			+ data size: 8bit	-> CR1 bit 12 M =
			+ parity: even		-> CR1 bit 10 PCE = 1 (parity control enable)
						-> CR1 bit 9  PS = 0 (parity selection)
		- Enable transmitter, receiver	-> CR1 bit 3 TE, bit 2 RE
		- Enable UART			-> CR1 bit 13 UE
	*/
	__HAL_RCC_USART1_CLK_ENABLE();	// 16Mhz
	uint32_t* BRR = (uint32_t*)(UART1_BASE_ADDR + 0x08);
	*BRR = (104 << 4) | (3 << 0);

	uint32_t* CR1 = (uint32_t*)(UART1_BASE_ADDR + 0x0C);
	*CR1 |= (1 << 12) | (1 << 10) | (1 << 3) | (1 << 2) | (1 << 13);

	/* enable interrupt */
	*CR1 |= (1 << 5);
	uint32_t* NVIC_ISER1 = (uint32_t*)0xE000E104;
	*NVIC_ISER1 |= 1<<5;
}


void UART_Transmit(uint8_t data){
    uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR + 0x00);
    uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR + 0x04);
    while(((*SR >> 7) & 1) == 0);
    *DR = data;
    while(((*SR >> 6) & 1) == 0);
}

void UART_Print_Log(char* msg)
{
	int msg_len = strlen(msg);
	for(int i = 0; i < msg_len; i++)
	{
		UART_Transmit((uint8_t)msg[i]);
	}
}

char UART_Receive()
{
    uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR + 0x00);
    uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR + 0x04);
    while(((*SR >> 5) & 1) == 0);
    char data = *DR;
    return data;
}

char storeData[100];
int idx;

void USART1_IRQHandler(){
	storeData[idx++] = UART_Receive();
	if(strstr(storeData, "\n"))
	{
		if(strstr(storeData, "blue led on"))
		{
			LedCtrl(LED_BLUE, 1);
			UART_Print_Log("--> ON LED OK\n");
		}
		else if(strstr(storeData, "blue led off"))
		{
			LedCtrl(LED_BLUE, 0);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else if(strstr(storeData, "red led on"))
		{
			LedCtrl(LED_RED, 1);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else if(strstr(storeData, "red led off"))
		{
			LedCtrl(LED_RED, 0);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else
		{
			UART_Print_Log("--> COMMAND NOT FOUND\n");
		}

		memset(storeData, 0,  sizeof(storeData));
		idx = 0;
	}
}

void I2C_Init(){
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR);

	*GPIOB_MODER &= ~(0b11 << 12);
	*GPIOB_MODER |= (0b10 << 12);

	*GPIOB_MODER &= ~(0b11 << 18);
	*GPIOB_MODER |= (0b10 << 18);

	uint32_t* GPIOB_AFRL = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	*GPIOB_AFRL &= ~(0b1111 << 24);
	*GPIOB_AFRL |= (0b0100 << 24);

	uint32_t* GPIOB_AFRH = (uint32_t*)(GPIOB_BASE_ADDR + 0x24);
	*GPIOB_AFRH &= ~(0b1111 << 4);
	*GPIOB_AFRH |= (0b0100 << 4);

	__HAL_RCC_I2C1_CLK_ENABLE();

	uint32_t* I2C_CR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x04);
	*I2C_CR2 &= ~(0b111111);
	*I2C_CR2 |= 16;

	uint32_t* I2C_CCR = (uint32_t*)(I2C1_BASE_ADDR + 0x1C);
	*I2C_CCR &= ~(0xFFF);
	*I2C_CCR |= 160;

	uint32_t* I2C_CR1 = (uint32_t*)(I2C1_BASE_ADDR);
	*I2C_CR1 &= ~(0b1);
	*I2C_CR1 |= 0b1;
}

void I2C_Write_Data(uint8_t slave_reg_addr, uint8_t slave_reg_val){
	//- Generate start bit
	uint32_t* I2C_CR1 = (uint32_t*)(I2C1_BASE_ADDR);
	*I2C_CR1 |= (1 << 8);

	//wait SB in SR1
	uint32_t* I2C_SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
	while (((I2C_SR1 >> 0) & 1) == 0); //OR while (((I2C_SR1 & (1 << 0)) == 0);

	//- Send 7 slave add + 1 bit write (0b0011001 << 1 | 0)
	uint32_t* I2C_DR = (uint32_t*)(I2C1_BASE_ADDR + 0x10);
	*I2C_DR = 0b00110010;
	while (((I2C_SR1 >> 1) & 1) == 0);
	uint32_t* I2C_SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
	uint32_t temp = *I2C_SR2;

	//- Check ACK
	while (((I2C_SR1 >> 10) & 1) == 1);

	//- Send command frame.       (0x1F)
	*I2C_DR = 0b00110010;
	while (((I2C_SR1 >> 2) & 1) == 0);

	//- Generate stop bit
	*I2C_CR1 |= (1 << 9);

}

void I2C_Read_Data(uint8_t slave_reg_addr){
	// - Generate start bit
	uint32_t* I2C_CR1 = (uint32_t*)(I2C1_BASE_ADDR);
	*I2C_CR1 |= (1 << 8);
	uint32_t* I2C_SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
	while (((I2C_SR1 >> 0) & 1) == 0); //OR while (((I2C_SR1 & (1 << 0)) == 0);

	//- Send 7 slave add + 1 bit write (0b0011001 << 1 | 0)
	uint32_t* I2C_DR = (uint32_t*)(I2C1_BASE_ADDR + 0x10);
	*I2C_DR = 0b00110010;
	while (((I2C_SR1 >> 1) & 1) == 0);
	uint32_t* I2C_SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
	uint32_t temp = *I2C_SR2;

	// - Check ACK
	while (((I2C_SR1 >> 10) & 1) == 1);

	// - Send command frame.
	*I2C_DR = 0b00110010;
	while (((I2C_SR1 >> 2) & 1) == 0);

	// - Check ACK
	while (((I2C_SR1 >> 10) & 1) == 1);

	// - Generate start bit
	*I2C_CR1 |= (1 << 8);

	// - Send 7 slave add + 1 bit read
	*I2C_DR = 0b00110010;
	while (((I2C_SR1 >> 1) & 1) == 0);
	temp = *I2C_SR2;

	// - Check ACK
	while (((I2C_SR1 >> 10) & 1) == 1);

	// - Read data
	while (((I2C_SR1 >> 6) & 1) == 0);
	uint8_t data = *I2C_DR;

	// - Generate stop bit
	*I2C_CR1 |= (1 << 9);
	return data;
}

void SPI_Init(){
	/*
	 - config
	 PA5 -> SPI1_SCLK
	 PA7 -> SPI1_MOSI
	 PA6 -> SPI1_MISO
	 PE3 -> output
	 - stm32 works in master mode
	 - use software slave management
	 - clock
	 - enable SPI
	*/

	__HAL_RCC_GPIOA_CLK_ENABLE();

	//Map Pin, PA5 - SCLK, PA6 - MISO, PA7 - MOSI
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &= ~(0b111111 << 10);
	*GPIOA_MODER |= (0b10 << 10) | (0b10 << 12) | (0b10 << 14);

	//Alternative function for PA5, PA6, PA7
	uint32_t* GPIOA_AFRL = (uint32_t*)(GPIOA_BASE_ADDR + 0x20);
	*GPIOA_AFRL &= ~(0xfff << 20);
	*GPIOA_AFRL |= (0b0101 << 20) | (0b0101 << 24) | (0b0101 << 28);

	//Set CS output - PE3
	__HAL_RCC_GPIOE_CLK_ENABLE();

	uint32_t* GPIOE_MODER = (uint32_t*)(GPIOE_BASE_ADDR + 0x00);
	*GPIOE_MODER &= ~(0b11 << 6);
	*GPIOE_MODER |= (0b01 << 6); //output is 01

	__HAL_RCC_SPI1_CLK_ENABLE(); //16 Mhz
	//stm32 as master, speed of SCLK, use software slave management, enable SPI

	uint16_t* SPI1_CR1 = (uint16_t*)(SPI1_BASE_ADDR + 0x00);
	//Master config
	*SPI1_CR1 |= (1 << 2);
	//Software slave management
	*SPI1_CR1 |= (1 << 9) | (1 << 8);
	//Baud rate control
	*SPI1_CR1 |= (0b011 << 3);
	//SPI enable
	*SPI1_CR1 |= (1 << 6);
}

void SPI_READ(char reg_addr){
	/*
	SPI_Read
	- set PE3 (CS) to LOW -> active slave
	- send register address of slave (write register address to DR)
	- read DR -> clear garbage data
	- send dumy data (write 0xFF to DR) (to generate clock for slave send data to master)
	- read DR
	- set PE3 (CS) to HIGH -> inactive slave
	*/

	// Set PE3 to LOW to active slave
	uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
	*GPIOE_ODR &= ~(1 << 3);

	uint16_t* SPI1_DR = (uint16_t*)(SPI1_BASE_ADDR + 0x0C);
	uint16_t* SPI1_SR = (uint16_t*)(SPI1_BASE_ADDR + 0x08);

	while (((*SPI1_SR >> 7) & 1) == 1);
	*SPI1_DR = reg_addr | (1 << 7);
	while (((*SPI1_SR >> 1) & 1) == 0);
	while (((*SPI1_SR >> 7) & 1) == 1);

	while (((*SPI1_SR >> 0) & 1) == 0);
	char data = *SPI1_DR;

	while (((*SPI1_SR >> 7) & 1) == 1);
	*SPI1_DR = 0xff;
	while (((*SPI1_SR >> 1) & 1) == 0);
	while (((*SPI1_SR >> 7) & 1) == 1);

	while (((*SPI1_SR >> 0) & 1) == 0);
	data = *SPI1_DR;

	return data;
}

void SPI_WRITE(char reg_addr, char reg_val){
	// Set PE3 to LOW to active slave
	uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
	*GPIOE_ODR &= ~(1 << 3);

	uint16_t* SPI1_DR = (uint16_t*)(SPI1_BASE_ADDR + 0x0C);
	uint16_t* SPI1_SR = (uint16_t*)(SPI1_BASE_ADDR + 0x08);

	while (((*SPI1_SR >> 7) & 1) == 1);
	*SPI1_DR = reg_addr;
	while (((*SPI1_SR >> 1) & 1) == 0);
	while (((*SPI1_SR >> 7) & 1) == 1);

	while (((*SPI1_SR >> 0) & 1) == 0);
	char data = *SPI1_DR;

	while (((*SPI1_SR >> 7) & 1) == 1);
	*SPI1_DR = reg_addr;
	while (((*SPI1_SR >> 1) & 1) == 0);
	while (((*SPI1_SR >> 7) & 1) == 1);

	while (((*SPI1_SR >> 0) & 1) == 0);
	data = *SPI1_DR;

	*GPIOE_ODR |= (1 << 3); //Set PE3 to HIGH to inactive slave
}

#define TIM1_BASE_ADDR 0x40010000
void time_init()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
	//1s, PSC = 16000, ARR = 1000
	uint16_t* TIM1_PSC = (uint16_t*)(TIM1_BASE_ADDR + 0X28);
	uint16_t* TIM1_ARR = (uint16_t*)(TIM1_BASE_ADDR + 0X2C);
	uint16_t* TIM1_CR1 = (uint16_t*)(TIM1_BASE_ADDR + 0X00);

	*TIM1_PSC = 16000 - 1;
	*TIM1_ARR = 1000;
	*TIM1_CR1 |= (1 << 0); //enable TIMER1
}

void delay_1s()
{
	uint16_t* TIM1_SR = (uint16_t*)(TIM1_BASE_ADDR + 0X10);
	while (((*TIM1_SR >> 0) & 1) == 0);
	*TIM1_SR &= ~(1 << 0);
}

int main()
{
	HAL_Init();
	LedInit();
	ButtonInit();
	EXTI0Init();
	UART_Init();
	SPI_Init();
	char id = SPI_READ(0x0f);
	id = SPI_READ(0x20);

	SPI_WRITE(0x20, 0x0f);
	id = SPI_READ(0x20);

	time_init();

	while(1)
	{
		LedCtrl(LED_RED, 1);
		delay_1s();
		LedCtrl(LED_RED, 0);
		delay_1s();
    }
	return 0;
}
