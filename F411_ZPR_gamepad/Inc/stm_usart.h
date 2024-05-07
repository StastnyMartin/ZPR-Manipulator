#ifndef STM_USART_H_
#define STM_USART_H_
#include "stm_core.h"
#include <stdio.h>
void Usart2Init(int baudSpeed);
void Usart1Init(int baudSpeed);
bool IsUsart2Recv(void);
/* nepotrebujeme "verejne"
int Usart2Send(char c);
void Usart2String(char *txt);
int Usart2Recv(void);
int Usart1Send(char c)
*/
#endif /* STM_USART_H_ */
