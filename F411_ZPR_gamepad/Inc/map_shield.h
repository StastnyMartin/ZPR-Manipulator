
#ifndef MAP_SHIELD_H_
#define MAP_SHIELD_H_

#include "stm_core.h"

#define RGB_BLUE  GPIOB,13
#define RGB_GREEN GPIOB,14
#define RGB_RED   GPIOB,15

#define BTN_LEFT GPIOB, 5
#define BTN_RIGHT GPIOB, 4

#define USE_SPI_LED

void Init8LED(void);
void Write8LED(uint8_t val);
void InitSPILED(void);
void writeSPILED(uint8_t val);

#define X_Axis 	GPIOA,0
#define Y_Axis 	GPIOA,1
#define BTN_A	GPIOA,10
#define BTN_B	GPIOB,3
#define BTN_C	GPIOB,5
#define BTN_D	GPIOB,4
#define BTN_E	GPIOB,10
#define BTN_F	GPIOA,8
#define BTN_K	GPIOA,9


//#define LED_V10	GPIOC,	0
//#define LED_V11	GPIOC,	1
//#define LED_V12	GPIOC,	2
//#define LED_V13	GPIOC,	3
//#define LED_V14	GPIOC,	4
//#define LED_V15	GPIOC,	5
//#define LED_V16	GPIOC,	6
//#define LED_V17	GPIOC,	7

#endif /* MAP_SHIELD_H_ */

