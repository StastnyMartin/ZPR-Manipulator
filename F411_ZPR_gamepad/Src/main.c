#include "map_shield.h"
#include <stdio.h>
#include "stm_usart.h"

#define ADC_CH0	ADC1,0
#define ADC_CH1	ADC1,1

volatile uint32_t _ticks = 0;

void SysTick_Handler(void)
{
	_ticks++;
}

uint16_t readADC(ADC_TypeDef *adcpin, int adc_channel)
{
	uint16_t w = 0;
	adcpin->SQR3 = adc_channel;
	adcpin->CR2 |= ADC_CR2_SWSTART;		//Start conversion

	while (!(adcpin->SR & ADC_SR_EOC))
		;
	w = adcpin->DR;

	return w;
}

uint8_t read_btns()
{
	uint8_t result = 0;

	if (GPIORead(BTN_A))
		result = 1;
	if (GPIORead(BTN_B))
		result |= 1 << 1;
	if (GPIORead(BTN_C))
		result |= 1 << 2;
	if (GPIORead(BTN_D))
		result |= 1 << 3;
	if (GPIORead(BTN_E))
		result |= 1 << 4;
	if (GPIORead(BTN_F))
		result |= 1 << 5;
	if (GPIORead(BTN_K))
		result |= 1 << 6;

	return result;
}

int main(void)
{

	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000 - 1);

	Usart1Init(38400);

	STM_SetPinGPIO(BTN_A, ioPortInputFloat);
	STM_SetPinGPIO(BTN_B, ioPortInputFloat);
	STM_SetPinGPIO(BTN_C, ioPortInputFloat);
	STM_SetPinGPIO(BTN_D, ioPortInputFloat);
	STM_SetPinGPIO(BTN_E, ioPortInputFloat);
	STM_SetPinGPIO(BTN_F, ioPortInputFloat);
	STM_SetPinGPIO(BTN_K, ioPortInputFloat);

	STM_SetPinGPIO(GPIOA, 0, ioPortAnalog);
	STM_SetPinGPIO(GPIOA, 1, ioPortAnalog);

	if (!(RCC->APB2ENR & RCC_APB2ENR_ADC1EN))
	{
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
	}

	ADC1->CR1 = (1 << 25) | ADC_CR1_SCAN;// RES = 10 = 8bitu, SCAN mode ON
	ADC1->CR2 = 0; // ALIGN = 0 = right align
	ADC1->SMPR1 = 0;
	ADC1->SMPR2 = 2; // 010 pro ch0 = 28 cycles
	ADC1->SQR1 = 0; // L = 0000 = 1 conversion
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 0; // SQ1 = 00000 = AD channel 0
	ADC->CCR = 0;
	// ADCPRE = 00 = APB2 / 2

	ADC1->CR2 |= ADC_CR2_ADON; // AD turn ON

	uint32_t tm = 0;
	uint16_t w = 0;

	while (1)
	{

		if (_ticks >= tm)
		{
			tm = _ticks + 10;
			w = readADC(ADC_CH0);
			Usart1Send(w);
			w = readADC(ADC_CH1);
			Usart1Send(w);
			w = read_btns();
			Usart1Send(w);
		}
	}

}
