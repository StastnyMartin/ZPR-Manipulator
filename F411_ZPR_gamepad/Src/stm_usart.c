#include "stm_usart.h"


int Usart2Send(char c)
{
while(!(USART2->SR & USART_SR_TXE))
; // cekej dokud neni volny TDR
//c = 48;
USART2->DR = c; // zapis do TDR k odeslani
return c;
}

int Usart2Recv(void)
{
while(!(USART2->SR & USART_SR_RXNE)) // cekej dokud neprijde
;
return USART2->DR; // vycti a vrat jako hodnotu
}

bool IsUsart2Recv(void) // priznak, ze je neco v bufferu
{
return (USART2->SR & USART_SR_RXNE) != 0;
// podminka vynuti true/false vysledek
}

// addon for CubeIDE with new structure of syscalls.c
int __io_putchar(int ch) { return Usart2Send(ch); }
int __io_getchar(void) { return Usart2Recv(); }
// end of addon

void Usart2String(char *txt)
{
while(*txt) // projdi znak po znaku do ukoncovaci 0
{
Usart2Send(*txt);
txt++;
}
}

void Usart2Init(int baudSpeed)
{
	STM_SetPinGPIO(GPIOA, 2, ioPortAlternatePP); // USART2 Tx
		STM_SetAFGPIO(GPIOA, 2, 7); // AF = 07
		STM_SetPinGPIO(GPIOA, 3, ioPortAlternatePP); // USART2 Rx
		STM_SetAFGPIO(GPIOA, 3, 7); // AF = 07
		if (!(RCC->APB1ENR & RCC_APB1ENR_USART2EN)) // neni povolen USART2
		{
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
		}
		USART2->CR1 = USART_CR1_RE | USART_CR1_TE; // potreba povoleni prijmu a vysilani
		USART2->CR2 = 0; // nic specialniho
		USART2->CR3 = 0; // nic specialniho
		//TODO doplnit vypocet BRR podle pozadovaneho a podle clocku
		USART2->BRR = 0x1A1; // rychlost 38400 pri 16MHz - spocitano predem
		USART2->CR1 |= USART_CR1_UE; // az na zaver povolen blok USARTu

		// zrusit bufferovani vystupu i vstupu
		setvbuf(stdout, NULL, _IONBF, 0);
		setvbuf(stdin, NULL, _IONBF, 0);
}

void Usart1Init(int baudSpeed)
{
		STM_SetPinGPIO(GPIOB, 6, ioPortAlternatePP); // USART1 Tx
		STM_SetAFGPIO(GPIOB, 6, 7); // AF = 07
		STM_SetPinGPIO(GPIOB, 7, ioPortAlternatePP); // USART1 Rx
		STM_SetAFGPIO(GPIOB, 7, 7); // AF = 07
		if (!(RCC->APB2ENR & RCC_APB2ENR_USART1EN)) // neni povolen USART1
		{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		}
		USART1->CR1 = USART_CR1_RE | USART_CR1_TE; // potreba povoleni prijmu a vysilani
		USART1->CR2 = 0; // nic specialniho
		USART1->CR3 = 0; // nic specialniho
		//TODO doplnit vypocet BRR podle pozadovaneho a podle clocku
		USART1->BRR = 0x1A1; // rychlost 38400 pri 16MHz - spocitano predem
		USART1->CR1 |= USART_CR1_UE; // az na zaver povolen blok USARTu

		// zrusit bufferovani vystupu i vstupu
		//setvbuf(stdout, NULL, _IONBF, 0);
		//setvbuf(stdin, NULL, _IONBF, 0);
}

int Usart1Send(char c)
{
while(!(USART1->SR & USART_SR_TXE))
; // cekej dokud neni volny TDR
//c = 48;
USART1->DR = c; // zapis do TDR k odeslani
return c;
}
