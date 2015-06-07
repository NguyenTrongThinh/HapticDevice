/****************************************************************************/
/*QEI library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 17/02/2015																													*/
/*Author: Thinh Nguyen									    																*/ 																
/****************************************************************************/

#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stdlib.h>
#include <stdio.h>

void TSVN_QEI_TIM1_Init(unsigned int MAX_CNT);
/*
Khoi tao encoder mode TIM1:PE9 PE11
Timer 1 *_*
*/
void TSVN_QEI_TIM1_Deinit(void);
long TSVN_QEI_TIM1_Value(void);
/*
Lay so xung encoder
*/

void TSVN_QEI_TIM2_Init(unsigned int MAX_CNT);
/*
Khoi tao encoder mode TIM2:PA0 PA1
Timer 2 *_*
*/
void TSVN_QEI_TIM2_Deinit(void);
long TSVN_QEI_TIM2_Value(void);

void TSVN_QEI_TIM3_Init(unsigned int MAX_CNT);
/*
Khoi tao encoder mode TIM3:PC6 PC7
Timer 3 *_*
*/
void TSVN_QEI_TIM3_Deinit(void);
long TSVN_QEI_TIM3_Value(void);

void TSVN_QEI_TIM4_Init(unsigned int MAX_CNT);
/*
Khoi tao encoder mode TIM4:PB6 PB7
Timer 4 *_*
*/
void TSVN_QEI_TIM4_Deinit(void);
long TSVN_QEI_TIM4_Value(void);
