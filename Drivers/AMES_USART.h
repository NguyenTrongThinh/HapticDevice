/****************************************************************************/
/*USART library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 15/02/2015																													*/
/*Author: Thinh Nguyen									    																*/ 																
/****************************************************************************/

#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define TSVN_USART_PORT GPIOA

#define TSVN_USART_RX_PIN GPIO_Pin_10
#define TSVN_USART_TX_PIN GPIO_Pin_9
#define TSVN_USART_BAURATE 9600

#define OUT_OF_MEM "NULL"

enum{None, End};

unsigned char TSVN_USART_Init(void);
/*Khoi tao USART:
1. Baurate: TSVN_USART_BAURATE
2. None handshake
3. Receive interupt enable.
4. 8 bits data
5. No parity.
*/
void TSVN_USART_Deinit(void);
/*
Huy bo USART
*/
unsigned char TSVN_USART_Create_Frame(char Rx);
/*
Ket noi du lieu nhan thanh 1 fram truyen hoan chinh
Tra ve End neu nhan du frame.
Tra ve None neu khong nhan duoc Frame.
*/
char* TSVN_USART_Get_Frame(void);
/*
Lay Frame truyen hien hanh.
*/
unsigned char TSVN_Find(char chrfind, char* Source, unsigned char start);
/*
Tim chuoi chrfind trong Source.
Chuoi bat dau tu 1.
Tra ve None neu khong tim thay.
*/
char* TSVN_Get_Parameters(unsigned char pvParameters, char* Frame);
/*
Lay tham so thu pvParameters trong frame truyen.
pvParameters bat dau tu 1.
*/

/*
Ngat nhan USART
void USART1_IRQHandler(void)
{
	xData ReceiveData;
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	ReceiveData.ID = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      ReceiveData.Value =(unsigned char)USART_ReceiveData(USART1);
      xQueueSendToBackFromISR(RxQueue, &ReceiveData, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);
		}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

Task USART Demo
void USART_TASK(void *pvParameters)
{
	xData ReadValue;
	portBASE_TYPE xStatus;
	char *Frame;
	char *Cmd;
	char i=0;
	while(1)
	{
		xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
		if (uxQueueMessagesWaiting(RxQueue) != NULL)
		{
			xStatus = xQueueReceive(RxQueue, &ReadValue, 0);
			if (xStatus == pdPASS)
			{
				if (TSVN_USART_Create_Frame(ReadValue.Value) == End)
				{
					for (i = 0; i<10; i++)
					{
					Cmd = TSVN_Get_Parameters(i, TSVN_USART_Get_Frame());
				  printf("%s\n", Cmd);	
					}
				}
			}
		}
	}
}
*/
