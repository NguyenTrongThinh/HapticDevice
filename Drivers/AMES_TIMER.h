/****************************************************************************/
/*TIMER library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 22/02/2015																													*/
/*Author: Thinh Nguyen									    																*/ 																
/****************************************************************************/

#include "stm32f10x.h" 
#include <stm32f10x_tim.h>
#include <stdlib.h>

void TSVN_TIM6_Init(unsigned long us);
void TSVN_TIM7_Init(unsigned long us);

/*
Khai bao ngat timer
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TSVN_Led_Toggle(LED_D5);
	}
}
*/
