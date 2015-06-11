#include <DeltaTask.h>

#define MOTOR1_PWM				PWM_CHANNEL_2
#define MOTOR2_PWM				PWM_CHANNEL_3
#define MOTOR3_PWM				PWM_CHANNEL_4
#define CAN_DATA_LENGTH		6

xTaskHandle	UART_TASK_HANDLER;
xTaskHandle	ENCODER_TASK_HANDLER;

xQueueHandle 	RxQueue;
xQueueHandle  PidQueue;
xQueueHandle  CanRxQueue;


xSemaphoreHandle UART_xCountingSemaphore;

enum  {MOTOR1 = 0x00, MOTOR2, MOTOR3};
enum  {RESERVE, FORWARD};
enum	{USART_ID, MO1_ID, MO2_ID, MO3_ID, SEN1, SEN2, SEN3};

unsigned char Application_Init(void)
{
	unsigned int PWM_Max_Value = 0;
	TSVN_FOSC_Init();
	TSVN_ACS712_Init();
	FIR_Init();
	TSVN_Led_Init(ALL);
	
	PWM_Max_Value = TSVN_PWM_TIM5_Init(5000);
	
	TSVN_PWM_TIM5_Set_Duty(0, MOTOR1_PWM);
	TSVN_PWM_TIM5_Set_Duty(0, MOTOR2_PWM);
	TSVN_PWM_TIM5_Set_Duty(0, MOTOR3_PWM);
	
	TSVN_PWM_TIM5_Start();
	
	DIR_Init();
	
	PID_Mem_Create(3);
	PID_WindUp_Init(MOTOR1, PWM_Max_Value);
	PID_WindUp_Init(MOTOR2, PWM_Max_Value);
	PID_WindUp_Init(MOTOR3, PWM_Max_Value);
	PID_Init(MOTOR1, 1, 0, 0);
	PID_Init(MOTOR2, 1, 0, 0);
	PID_Init(MOTOR3, 1, 0, 0);
	
	TSVN_USART_Init();
	TSVN_CAN_Init();
	
	TSVN_QEI_TIM1_Init(400);
	TSVN_QEI_TIM3_Init(400);
	TSVN_QEI_TIM4_Init(400);
	RxQueue  = xQueueCreate(50, sizeof(xData));
	CanRxQueue  = xQueueCreate(100, sizeof(CanRxMsg));
	PidQueue = xQueueCreate(100, sizeof(xMomentData));
	UART_xCountingSemaphore = xSemaphoreCreateCounting(50, 0);
	
	if (CanRxQueue != NULL && PidQueue != NULL
		&& RxQueue != NULL && UART_xCountingSemaphore != NULL )		
			return SUCCESS;
	else
		return ERROR;
}
void Application_Run(void)
{
	xTaskCreate(UART_TASK,  		(signed char*)  "UART", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, &UART_TASK_HANDLER);
	xTaskCreate(ENCODER_TASK,   (signed char*) 	"ENCODER", ENCODER_TASK_STACK_SIZE, NULL, ENCODER_TASK_PRIORITY, &ENCODER_TASK_HANDLER);	
	vTaskStartScheduler();
}
void ENCODER_TASK(void *pvParameters)
{
	float Theta[3];
	float Phi[3];
	float F[3] = {0.0, 15.0, 0.0};
	float Cordinate[3];
	float Moment[3];
	char Status = 0;
	CanTxMsg CanSendData;
	CanRxMsg CanReceiveData;
	portBASE_TYPE xStatus;
	xMomentData SendMoment;
	
	CanSendData.StdId = CAN_MASTER_STD_ID;
	CanSendData.IDE = CAN_ID_STD;
	CanSendData.RTR = CAN_RTR_DATA;
	CanSendData.DLC = CAN_DATA_LENGTH;
	
	while(1)
	{	
		Phi[0] = -90.0;
		Phi[1] = 30.0;
		Phi[2] = 150;
		if (uxQueueMessagesWaiting(CanRxQueue) != NULL)
		{
			xStatus = xQueueReceive(CanRxQueue, &CanReceiveData, 0);
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
			}
		}	
		Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
		Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
		Theta[2] = ((float)TSVN_QEI_TIM3_Value()*0.9)/7.0;		
		Status = (char)Delta_CalcForward(Theta[0], Theta[1], Theta[2], &Cordinate[0], &Cordinate[1], &Cordinate[2]);
		MomentCalculate(Theta, Phi, Cordinate, F, &Moment);
		SendMoment.Moment_MOTOR1 = Moment[0]*1000.0;
		SendMoment.Moment_MOTOR2 = Moment[1]*1000.0;
		SendMoment.Moment_MOTOR3 = Moment[2]*1000.0;
		xQueueSendToBack(PidQueue, &SendMoment, 1);
		if (Status == 0)
		{		
				if (Cordinate[0]<0)
				{
					CanSendData.Data[0] = 0;
					Cordinate[0] = -Cordinate[0];
					CanSendData.Data[1] = (unsigned char)Cordinate[0];
				}
				else
				{
					CanSendData.Data[0] = 1;
					CanSendData.Data[1] = (unsigned char)Cordinate[0];
				}
				if (Cordinate[1]<0)
				{
					CanSendData.Data[2] = 0;
					Cordinate[1] = -Cordinate[1];
					CanSendData.Data[3] = (unsigned char)Cordinate[1];
				}
				else
				{
					CanSendData.Data[2] = 1;
					CanSendData.Data[3] = (unsigned char)Cordinate[1];
				}
				if (Cordinate[2]<0)
				{
					CanSendData.Data[4] = 0;
					Cordinate[2] = -Cordinate[2];
					CanSendData.Data[5] = (unsigned char)Cordinate[2];
				}
				else
				{
					CanSendData.Data[4] = 1;
					CanSendData.Data[5] = (unsigned char)Cordinate[2];
				}
				CAN_Transmit(CAN1, &CanSendData);
			}
			TSVN_Led_Toggle(LED_D5);
			vTaskDelay(100);
	}
}

void UART_TASK(void *pvParameters)
{
	xData ReadValue;
	portBASE_TYPE xStatus;
	char *Cmd;
	double Temp;
	while(1)
	{
		xSemaphoreTake(UART_xCountingSemaphore, portMAX_DELAY);
		if (uxQueueMessagesWaiting(RxQueue) != NULL)
		{
			xStatus = xQueueReceive(RxQueue, &ReadValue, 0);
			if (xStatus == pdPASS)
			{
				if (ReadValue.ID == USART_ID)
				{
					if (TSVN_USART_Create_Frame(ReadValue.Value) == End)
					{		
							Cmd = TSVN_Get_Parameters(1, TSVN_USART_Get_Frame());
							
							if (!strcmp(Cmd, "PWM1"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atol(Cmd);
								TSVN_PWM_TIM5_Set_Duty((unsigned int)Temp, MOTOR1_PWM);
								printf("PWM1 is set to: %d\n", (unsigned int)Temp);
							}
							else if (!strcmp(Cmd, "PWM2"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atol(Cmd);
								TSVN_PWM_TIM5_Set_Duty((unsigned int)Temp, MOTOR2_PWM);
								printf("PWM2 is set to: %d\n", (unsigned int)Temp);
							}
							else if (!strcmp(Cmd, "PWM3"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atol(Cmd);
								TSVN_PWM_TIM5_Set_Duty((unsigned int)Temp, MOTOR3_PWM);
								printf("PWM3 is set to: %d\n", (unsigned int)Temp);
							}
							else if (!strcmp(Cmd, "DIR1"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atoi(Cmd);
								DIR_Change(MOTOR1, (unsigned char)Temp);
								printf("Dir1 is change: %d\n", (unsigned int)Temp);
							}		
							else if (!strcmp(Cmd, "DIR2"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atoi(Cmd);
								DIR_Change(MOTOR2, (unsigned char)Temp);
								printf("Dir2 is change: %d\n", (unsigned int)Temp);
							}
							else if (!strcmp(Cmd, "DIR3"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atoi(Cmd);
								DIR_Change(MOTOR3, (unsigned char)Temp);
								printf("Dir3 is change: %d\n", (unsigned int)Temp);
							}
							else if (!strcmp(Cmd, "Who"))
								printf("Delta\n");
					}		
				}
			}
		}
	}
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

