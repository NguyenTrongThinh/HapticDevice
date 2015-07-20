#include <xTask.h>

#define MOTOR1_PWM				PWM_CHANNEL_2
#define MOTOR2_PWM				PWM_CHANNEL_3
#define MOTOR3_PWM				PWM_CHANNEL_4

#define CAN_DATA_LENGTH		6
#define TIMOUT_ID					1

//#define USE_VIRTUAL_WALL 		

xQueueHandle Pos_Queue;
xQueueHandle CanRxQueue;
xQueueHandle RxQueue;
TimerHandle_t TimeoutTimer;

xSemaphoreHandle UART_xCountingSemaphore;
Moment_Typedef Moment;

enum  {MOTOR1 = 0x00, MOTOR2, MOTOR3};
enum  {RESERVE, FORWARD};
enum  {USART_ID};



static __IO bool __FORCE_REQUEST = false;

//*******************Global*********************

//**********************************************

void vTimeoutCallback(TimerHandle_t pxTimer);

unsigned char Application_Init(void)
{	
	TSVN_FOSC_Init();
	TSVN_Led_Init(ALL);
	
	TSVN_QEI_TIM1_Init(400);
	TSVN_QEI_TIM2_Init(400);
	TSVN_QEI_TIM4_Init(400);
	
	TSVN_CAN_Init();
	
	TSVN_USART_Init();
	
	Pos_Queue = xQueueCreate(200, sizeof(Pos_TypeDef));
	CanRxQueue = xQueueCreate(200, sizeof(CanRxMsg));
	RxQueue = xQueueCreate(200, sizeof(xData));
	
	UART_xCountingSemaphore = xSemaphoreCreateCounting(200, 0);
	
	if (Pos_Queue != NULL && CanRxQueue != NULL  && UART_xCountingSemaphore != NULL && RxQueue != NULL)
		return SUCCESS;
	return ERROR;
}

void Application_Run(void)
{
	TimeoutTimer = xTimerCreate("Timeout", 1000/portTICK_PERIOD_MS , pdTRUE, (void*) TIMOUT_ID, vTimeoutCallback);
	if (TimeoutTimer != NULL)
		xTimerStart(TimeoutTimer, 0);
	xTaskCreate(POS_TASK, 	"POS", POS_TASK_STACK_SIZE, NULL, POS_TASK_PRIORITY, NULL);	
	xTaskCreate(MOMENT_TASK, 	"MOMENT", MOMENT_TASK_STACK_SIZE, NULL, MOMENT_TASK_PRIORITY, NULL);	
	vTaskStartScheduler();
}

void MOMENT_TASK(void *pvParameters)
{
	static float Theta[3];
	float Cordinate[3];
	float F[3] = {0.0, 0.0, 0.0};
	float Phi[3];
	float Moment[3];
	TickType_t xLastWakeTime;
	#ifndef USE_VIRTUAL_WALL
		portBASE_TYPE xStatus;
		CanRxMsg CanReceiveData;
	#endif
	Moment_Typedef M;
	unsigned char Status;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		#ifndef USE_VIRTUAL_WALL
			if (uxQueueMessagesWaiting(CanRxQueue) != NULL)
			{
				xStatus = xQueueReceive(CanRxQueue, &CanReceiveData, 1);
				if (xStatus == pdPASS)
				{
					F[2] = CanReceiveData.Data[5];
					if (CanReceiveData.Data[4] == 0)
						F[2] = -F[2];
					F[2] += 10;
				}
				__FORCE_REQUEST = false;
			}
			Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
			Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
			Theta[2] = ((float)TSVN_QEI_TIM2_Value()*0.9)/7.0;		
			Status = (unsigned char)Delta_CalcForward(Theta[0], Theta[1], Theta[2], &Cordinate[0], &Cordinate[1], &Cordinate[2]);
			if (Status == 0)
			{
				if (Cordinate[0] >= -44.165 && Cordinate[0] <= 44.165 && 
						Cordinate[1] >= -44.165 && Cordinate[1] <= 44.165 &&
						Cordinate[2] >= -203.648 && Cordinate[2] <= -115.318)
				{
						Phi[0] = -90.0;
						Phi[1] = 30.0;
						Phi[2] = 150;
						MomentCalculate(Theta, Phi, Cordinate, F, &Moment);
						M.Mx = (Moment[0]*1500.0 > 1500.0)?1500.0:Moment[0]*1500.0;
						M.My = (Moment[1]*1500.0 > 1500.0)?1500.0:Moment[1]*1500.0;
						M.Mz = (Moment[2]*1500.0 > 1500.0)?1500.0:Moment[2]*1500.0;
						printf("{M %0.5f %0.5f %0.5f}", M.Mx, M.My, M.Mz);
				}
			}
			TSVN_Led_Toggle(LED_D6);
			vTaskDelayUntil( &xLastWakeTime, 20);
		#else
			Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
			Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
			Theta[2] = ((float)TSVN_QEI_TIM2_Value()*0.9)/7.0;
			Status = (unsigned char)Delta_CalcForward(Theta[0], Theta[1], Theta[2], &Cordinate[0], &Cordinate[1], &Cordinate[2]);	
			if (Status == 0)
			{
				if (Cordinate[0] >= -44.165  && Cordinate[0] <= 44.165 && 
						Cordinate[1] >= -44.165  && Cordinate[1] <= 44.165 &&
						Cordinate[2] >= -203.648 && Cordinate[2] <= -115.318)
				{
						Phi[0] = -90.0;
						Phi[1] = 30.0;
						Phi[2] = 150.0;
						//F[2] = (float)Cordinate[2]*0.178 + 25.7;
						F[2] = (Cordinate[2] >= -170.0)?15:10;
						MomentCalculate(Theta, Phi, Cordinate, F, &Moment);
						M.Mx = (Moment[0]*1500.0 > 1500.0)?1500.0:Moment[0]*1500.0;
						M.My = (Moment[1]*1500.0 > 1500.0)?1500.0:Moment[1]*1500.0;
						M.Mz = (Moment[2]*1500.0 > 1500.0)?1500.0:Moment[2]*1500.0;
						printf("{M %0.5f %0.5f %0.5f}", M.Mx, M.My, M.Mz);
				}
			}
			TSVN_Led_Toggle(LED_D6);
			vTaskDelayUntil( &xLastWakeTime, 20);
		#endif
	}
}
void POS_TASK(void *pvParameters)
{
	float Theta[3];
	Pos_TypeDef Pos;
	char Status = 0;
	static __IO uint8_t Timeout = 0;
	static __IO bool isSend = false;
	TickType_t xLastWakeTime;
	CanTxMsg CanSendData;
	
	CanSendData.StdId = CAN_MASTER_STD_ID;
	CanSendData.IDE = 	CAN_ID_STD;
	CanSendData.RTR = 	CAN_RTR_DATA;
	CanSendData.DLC = 	CAN_DATA_LENGTH;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
			if (__FORCE_REQUEST)
			{
				if (!isSend)
				{
					CanSendData.Data[0] = 'F';
					CAN_Transmit(CAN1, &CanSendData);
					isSend = true;
				}
				if (Timeout++ >= 5)
				{
					Timeout = 0;
					isSend = false;
					__FORCE_REQUEST = false;
				}
			}
			else
			{
				Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
				Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
				Theta[2] = ((float)TSVN_QEI_TIM2_Value()*0.9)/7.0;		
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
				}
			}
			if (!__FORCE_REQUEST)
			{
				CAN_Transmit(CAN1, &CanSendData);
			}
			TSVN_Led_Toggle(LED_D4);
			vTaskDelayUntil( &xLastWakeTime, 100);
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
		{
			xQueueSendToBackFromISR(CanRxQueue, &RxMessage, &xHigherPriorityTaskWoken);
		}
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
void USART1_IRQHandler(void)
{
	xData ReceiveData;
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	ReceiveData.ID = USART_ID;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		ReceiveData.Value =(unsigned char)USART_ReceiveData(USART1);
		xQueueSendToBackFromISR(RxQueue, &ReceiveData, &xHigherPriorityTaskWoken);
		xSemaphoreGiveFromISR(UART_xCountingSemaphore, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
void vTimeoutCallback(TimerHandle_t pxTimer)
{
	TSVN_Led_Toggle(LED_D7);
		__FORCE_REQUEST = true;
}

