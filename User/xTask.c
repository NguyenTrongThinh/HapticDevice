#include <xTask.h>

#define MOTOR1_PWM				PWM_CHANNEL_2
#define MOTOR2_PWM				PWM_CHANNEL_3
#define MOTOR3_PWM				PWM_CHANNEL_4

#define CAN_DATA_LENGTH		6

xQueueHandle Pos_Queue;
xQueueHandle CanRxQueue;
xQueueHandle Moment_Queue;

xSemaphoreHandle CAN_CountingSemaphore;

Moment_Typedef Moment;

enum  {MOTOR1 = 0x00, MOTOR2, MOTOR3};
enum  {RESERVE, FORWARD};
enum  {USART_ID};

//***********************************Global*********************

//**************************************************************

unsigned char Application_Init(void)
{
	
	TSVN_FOSC_Init();
	TSVN_Led_Init(ALL);
	
	TSVN_QEI_TIM1_Init(400);
	TSVN_QEI_TIM3_Init(400);
	TSVN_QEI_TIM4_Init(400);
	
	TSVN_CAN_Init();
	
	Pos_Queue = xQueueCreate(20, sizeof(Pos_TypeDef));
	CanRxQueue = xQueueCreate(20, sizeof(CanRxMsg));
	Moment_Queue = xQueueCreate(20, sizeof(Moment_Typedef));
	CAN_CountingSemaphore = xSemaphoreCreateCounting(10, 0);
	
	if (Pos_Queue != NULL && CanRxQueue != NULL && CAN_CountingSemaphore != NULL &&
			Moment_Queue != NULL)
		return SUCCESS;
	return ERROR;
}

void Application_Run(void)
{
	xTaskCreate(POS_TASK, 	"POS", POS_TASK_STACK_SIZE, NULL, POS_TASK_PRIORITY, NULL);	
	xTaskCreate(TRANSFER_TASK, 	"TRANSFER", TRANSFER_TASK_STACK_SIZE, NULL, TRANSFER_TASK_PRIORITY, NULL);	
	xTaskCreate(MOMENT_TASK, 	"MOMENT", MOMENT_TASK_STACK_SIZE, NULL, MOMENT_TASK_PRIORITY, NULL);	

	vTaskStartScheduler();
}

void MOMENT_TASK(void *pvParameters)
{
	float Theta[3];
	float Cordinate[3];
	float F[3] = {0.0, 15.0, 0.0};
	float Phi[3];
	float Moment[3];
	portBASE_TYPE xStatus;
	CanRxMsg CanReceiveData;
	Moment_Typedef M;
	unsigned char Status;
	while(1)
	{
		xSemaphoreTake(CAN_CountingSemaphore, portMAX_DELAY);
		if (uxQueueMessagesWaiting(CanRxQueue) != NULL)
		{
			xStatus = xQueueReceive(CanRxQueue, &CanReceiveData, 1);
			if (xStatus == pdPASS)
			{
				F[0] = CanReceiveData.Data[1];
				F[1] = CanReceiveData.Data[3];
				F[2] = CanReceiveData.Data[5];
				if (CanReceiveData.Data[0] == 0)
					F[0] = -F[0];
				if (CanReceiveData.Data[2] == 0)
					F[1] = -F[1];
				if (CanReceiveData.Data[4] == 0)
					F[2] = -F[2];
				Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
				Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
				Theta[2] = ((float)TSVN_QEI_TIM3_Value()*0.9)/7.0;		
				Status = (char)Delta_CalcForward(Theta[0], Theta[1], Theta[2], &Cordinate[0], &Cordinate[1], &Cordinate[2]);
				if (Status == 0)
				{
					MomentCalculate(Theta, Phi, Cordinate, F, &Moment);
					M.Mx = Moment[0];
					M.My = Moment[1];
					M.Mz = Moment[2];
					xQueueSendToBack(Moment_Queue, &M, 1);
				}
			}
		}	
		TSVN_Led_Toggle(LED_D6);
	}
}
void TRANSFER_TASK(void *pvParameters)
{
	while(1)
	{	
		TSVN_Led_Toggle(LED_D5);
		vTaskDelay(500);
	}
}

void POS_TASK(void *pvParameters)
{
	float Theta[3];
	Pos_TypeDef Pos;
	char Status = 0;
	CanTxMsg CanSendData;
	while(1)
	{
		Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
		Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
		Theta[2] = ((float)TSVN_QEI_TIM3_Value()*0.9)/7.0;		
		Status = (char)Delta_CalcForward(Theta[0], Theta[1], Theta[2], &Pos.Px, &Pos.Py, &Pos.Pz);
		if (Status == 0)
		{
				if (Pos.Px<0)
				{
					CanSendData.Data[0] = 0;
					Pos.Px = -Pos.Px;
					CanSendData.Data[1] = (unsigned char)Pos.Px;
				}
				else
				{
					CanSendData.Data[0] = 1;
					CanSendData.Data[1] = (unsigned char)Pos.Px;
				}
				if (Pos.Py<0)
				{
					CanSendData.Data[2] = 0;
					Pos.Py = -Pos.Py;
					CanSendData.Data[3] = (unsigned char)Pos.Py;
				}
				else
				{
					CanSendData.Data[2] = 1;
					CanSendData.Data[3] = (unsigned char)Pos.Py;
				}
				if (Pos.Pz<0)
				{
					CanSendData.Data[4] = 0;
					Pos.Pz = -Pos.Pz;
					CanSendData.Data[5] = (unsigned char)Pos.Pz;
				}
				else
				{
					CanSendData.Data[4] = 1;
					CanSendData.Data[5] = (unsigned char)Pos.Pz;
				}
				CAN_Transmit(CAN1, &CanSendData);
		}
		TSVN_Led_Toggle(LED_D4);
		vTaskDelay(200);
	}
}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if (CAN_GetITStatus (CAN1, CAN_IT_FMP0))
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		if ((RxMessage.StdId == CAN_SLAVE_STD_ID) && (RxMessage.IDE == CAN_ID_STD) && (RxMessage.DLC == CAN_DATA_LENGTH))
		xQueueSendToBackFromISR(CanRxQueue, &RxMessage, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
