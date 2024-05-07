#include "map_shield.h"

#define USE_SPI_LED

void Init8LED(void)
{
	if(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN))
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOCRST;
		RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOCRST;
	}

	GPIOC->MODER &= 0xffff0000;
	GPIOC->MODER |= 0x00005555;

	GPIOC->OTYPER &= 0xff00;

	GPIOC->OSPEEDR |= 0x0000ffff;
	return;
}

void Write8LED(uint8_t val)
{
	GPIOC->ODR = (GPIOC->ODR & 0xff00) | (uint16_t)val;
	return;
}

void InitSPILED(void)
{
	STM_SetPinGPIO(GPIOA, 8, ioPortOutputPP); // SHIFT – LE. 595 - RCLK
	GPIOWrite(GPIOA, 8, 0);

	STM_SetPinGPIO(GPIOA, 9, ioPortOutputPP); // Output enable
	GPIOWrite(GPIOA, 9, 0);

#ifdef USE_SPI_LED
	STM_SetPinGPIO(GPIOA, 5, ioPortAlternatePP);
	STM_SetAFGPIO(GPIOA, 5, 5);
	STM_SetPinGPIO(GPIOA, 7, ioPortAlternatePP);
	STM_SetAFGPIO(GPIOA, 7, 5);
	if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
	{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	}
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; // CPOL = 0, CPHA = 0, DFF = 0 = 8b data, BR = 0 = clk/2
	SPI1->CR2 = 0; // zadne preruseni ani DMA
	SPI1->CR1 |= SPI_CR1_SPE; // SPI enable
#else
	STM_SetPinGPIO(GPIOA, 7, ioPortOutputPP); // SER in

	STM_SetPinGPIO(GPIOA, 5, ioPortOutputPP); // SHIFT – CLK, 595 - SRCLK
	GPIOWrite(GPIOA, 5, 0);
#endif
}

void writeSPILED(uint8_t val)
{
#ifdef USE_SPI_LED
	SPI1->DR = val;

	while(!(SPI1->SR & SPI_SR_TXE) || (SPI1->SR & SPI_SR_BSY))	//dokud je TXE 0 nebo BSY 1
	;



#else
	for (int i = 0;i<8;i++)
	{
		GPIOWrite(GPIOA, 7, (val & 0x80) != 0);
		GPIOWrite(GPIOA, 5, 1);
		GPIOWrite(GPIOA, 5, 0);

		val <<= 1;
	}
#endif
	GPIOWrite(GPIOA, 8, 1);
	GPIOWrite(GPIOA, 8, 0);
}
