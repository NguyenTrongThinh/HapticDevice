/****************************************************************************/
/*LED library.																											*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 15/02/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/
#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stdlib.h>


#define TSVN_LED_PORT 		GPIOD
#define TSVN_LED_D4_Pin 	GPIO_Pin_2
#define TSVN_LED_D5_Pin 	GPIO_Pin_3
#define TSVN_LED_D6_Pin 	GPIO_Pin_4
#define TSVN_LED_D7_Pin 	GPIO_Pin_7

enum{LED_D4, LED_D5, LED_D6, LED_D7, ALL};
/*
LED_NAME gom cac gia tri nhu dinh nghia enum ben tren.
Ten led xem tren Kit.
*/

void TSVN_Led_Init(unsigned char LED_NAME);
/*Khoi tao GPIO cho các led*/
void TSVN_Led_On(unsigned char LED_NAME);
/*Bat led voi LED_NAME*/
void TSVN_Led_Off(unsigned char LED_NAME);
/*Tat LED voi LED_NAME*/
void TSVN_Led_Toggle(unsigned char LED_NAME);
/*Toggle Led voi LED_NAME*/
void TSVN_Led_Deinit(void);
/*Xoa bo nho dung cho Led*/
