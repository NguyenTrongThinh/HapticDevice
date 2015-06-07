/****************************************************************************/
/*CAN library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 12/03/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/

#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_can.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


void TSVN_CAN_Init(void);
void TSVN_CAN_Deinit(void);
/*
void CAN_TASK(void *pvParameters)
{
	static CanTxMsg TMessage;
	unsigned char Data = 0;
		TMessage.StdId = CAN_MASTER_STD_ID;
		TMessage.ExtId = CAN_MASTER_EXT_ID;
		TMessage.IDE = CAN_ID_STD;
		TMessage.RTR = CAN_RTR_DATA;
		TMessage.DLC = 1;
		TMessage.Data[0] = 200;
	while(1)
	{			
		Data += 2;
		TMessage.Data[0] = Data;
		CAN_Transmit(CAN1, &TMessage);
		vTaskDelay(500);
	}
}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if (CAN_GetITStatus (CAN1, CAN_IT_FMP0))
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		if ((RxMessage.StdId == CAN_SLAVE_STD_ID) && (RxMessage.IDE == CAN_ID_STD) && (RxMessage.DLC == 1))
			xQueueSendToBackFromISR(Rx_CAN_Queue, &RxMessage, &xHigherPriorityTaskWoken);
	}
	xSemaphoreGiveFromISR(CAN_xCountingSemaphore, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

*/
