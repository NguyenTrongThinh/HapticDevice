/****************************************************************************/
/*BUTTON library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 15/02/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/
#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stdlib.h>

#define TSVN_S3_BUT_PORT GPIOC			  //TEMP
#define TSVN_S2_BUT_PORT GPIOA			  //WKUP	

#define TSVN_S3_BUT_PIN GPIO_Pin_13   //TEMP
#define TSVN_S2_BUT_PIN GPIO_Pin_0	  //WKUP

#define TRUE   1
#define FALSE  0
#define CountTimes 40

enum{S2, S3};
enum{Up, Down, Hold, Null};
/*
Button_Name la S2 S3 xem tren KIT
*/

void TSVN_BUT_Init(unsigned char Button_Name);
//Khoi tao nut nhan voi Button_Name


/*Thu vien dung ngat ngoai de bat su kien nhan nut.
S2: bat canh len.
S3 bat canh xuong.
De dung ngat phai khai bao ham ngat nhu sau:
void EXTI0_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; //Trang thai tac vu trong ngat.
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)								//Lay trang thai vector ngat 0
  {
    EXTI_ClearITPendingBit(EXTI_Line0);										//Xoa co ngat
		xSemaphoreGiveFromISR(xCountingSemaphore[0], &xHigherPriorityTaskWoken);			//Unblock task
		if (xHigherPriorityTaskWoken == pdTRUE)
	  vPortYieldFromISR();																//Chuyen vao thuc thi task da unblock.
  }
}

void EXTI15_10_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if(EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
		xSemaphoreGiveFromISR(xCountingSemaphore[1], &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken == pdTRUE)
	  vPortYieldFromISR();
	}
}

Task luon trong trang thai cho ngat va se thuc thi khi duoc unlock
void LED4_TASK(void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(xCountingSemaphore[0], portMAX_DELAY);
		TSVN_Led_Toggle(LED_D4);
	}
}

*/
