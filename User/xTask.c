#include <xTask.h>

#define MOTOR1_PWM				PWM_CHANNEL_2
#define MOTOR2_PWM				PWM_CHANNEL_3
#define MOTOR3_PWM				PWM_CHANNEL_4
#define CAN_DATA_LENGTH		6


xQueueHandle  PidQueue;
xQueueHandle  CanRxQueue;
xQueueHandle  UsartRxQueue;

xSemaphoreHandle UsartCountingSemaphore;


enum  {MOTOR1 = 0x00, MOTOR2, MOTOR3};
enum  {RESERVE, FORWARD};

//***********************************Global*********************

//**************************************************************

unsigned char Application_Init(void)
{
	unsigned int PWM_Max_Value = 0;
	
	TSVN_FOSC_Init();
	TSVN_Led_Init(ALL);
	TSVN_ACS712_Init();
	FIR_Init();
	TSVN_USART_Init();
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
	PID_Init(MOTOR1, 5, 0.001, 0.00001);
	PID_Init(MOTOR2, 5, 0.001, 0.00001);
	PID_Init(MOTOR3, 5, 0.001, 0.00001);
	
	TSVN_CAN_Init();
	
	TSVN_QEI_TIM1_Init(400);
	TSVN_QEI_TIM3_Init(400);
	TSVN_QEI_TIM4_Init(400);
	
	CanRxQueue  = xQueueCreate(50, sizeof(CanRxMsg));
	UsartRxQueue = xQueueCreate(50, sizeof(xData));
	PidQueue = xQueueCreate(50, sizeof(xMomentData));
	UsartCountingSemaphore = xSemaphoreCreateCounting(50, 0);
	
	if (CanRxQueue != NULL && PidQueue != NULL && UsartRxQueue != NULL 
			 && UsartCountingSemaphore != NULL)		
			return SUCCESS;
	else
		return ERROR;
	
}

void Application_Run(void)
{
	xTaskCreate(PID_TASK, "PID", PID_TASK_STACK_SIZE, NULL, PID_TASK_PRIORITY, NULL);	
	xTaskCreate(ENCODER_TASK,	"ENCODER", ENCODER_TASK_STACK_SIZE, NULL, ENCODER_TASK_PRIORITY, NULL);	
	xTaskCreate(USART_TASK, "USART", USART_TASK_STACK_SIZE, NULL, USART_TASK_PRIORITY, NULL);	
	vTaskStartScheduler();
}

void PID_TASK(void *pvParameters)
{
	portTickType xLastWakeTime;
	portBASE_TYPE xStatus;
	xMomentData ReadValue;
	float CurentValue_MOTOR[3];
	long PWM_MOTOR[3] = {0, 0, 0}; 
	unsigned char i = 0;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		while(FIR_CollectData(SEN_MOTOR1, TSVN_ACS712_Read(ACS_1)) != DONE);
		CurentValue_MOTOR[MOTOR1] = AMES_Filter(SEN_MOTOR1);
		while(FIR_CollectData(SEN_MOTOR2 ,TSVN_ACS712_Read(ACS_2)) != DONE);
		CurentValue_MOTOR[MOTOR2] = AMES_Filter(SEN_MOTOR2);
		while(FIR_CollectData(SEN_MOTOR3,TSVN_ACS712_Read(ACS_3))  != DONE);
		CurentValue_MOTOR[MOTOR3] = AMES_Filter(SEN_MOTOR3);
		if (uxQueueMessagesWaiting(PidQueue) != NULL)
		{
			xStatus = xQueueReceive(PidQueue, &ReadValue, 0);
			if (xStatus == pdPASS)
			{
				PWM_MOTOR[MOTOR1] = PID_Calculate(MOTOR1, ReadValue.Moment_MOTOR1 ,CurentValue_MOTOR[MOTOR1]);
				PWM_MOTOR[MOTOR2] = PID_Calculate(MOTOR2, ReadValue.Moment_MOTOR2 ,CurentValue_MOTOR[MOTOR2]);
				PWM_MOTOR[MOTOR3] = PID_Calculate(MOTOR3, ReadValue.Moment_MOTOR3 ,CurentValue_MOTOR[MOTOR3]);
				for(i = 0; i< (MOTOR3 + 1); i++)
				{
					if (PWM_MOTOR[i] < 0)
						DIR_Change(i, RESERVE);
					else
						DIR_Change(i, FORWARD);
					if (i == MOTOR1)
						TSVN_PWM_TIM5_Set_Duty(abs(PWM_MOTOR[i]), MOTOR1_PWM);
					else if (i == MOTOR2)
						TSVN_PWM_TIM5_Set_Duty(abs(PWM_MOTOR[i]), MOTOR2_PWM);
					else if (i == MOTOR3)
						TSVN_PWM_TIM5_Set_Duty(abs(PWM_MOTOR[i]), MOTOR3_PWM);	
				}	
				//*******For Debug******************
				//vTaskSuspendAll();
				//printf("%0.5f,\t%0.5f,\t%0.5f,\n", CurentValue_MOTOR[MOTOR1], CurentValue_MOTOR[MOTOR2], CurentValue_MOTOR[MOTOR3]);
				//xTaskResumeAll();
				//*************End Debug************
			}
		}
		TSVN_Led_Toggle(LED_D4);
		vTaskDelayUntil(&xLastWakeTime, 1);
	}
}
void ENCODER_TASK(void *pvParameters)
{
	portTickType xLastWakeTime;
	
	float Theta[3];
	float Phi[3];
	float F[3] = {0.0, 0.0, 20.0};
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
	
	xLastWakeTime = xTaskGetTickCount();
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
		TSVN_Led_Toggle(LED_D6);
		vTaskDelayUntil(&xLastWakeTime, 51);
	}
}
void USART_TASK(void *pvParameters)
{
	portTickType xLastWakeTime;
	portBASE_TYPE xStatus;
	xData USartReceiveData;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if (uxQueueMessagesWaiting(UsartRxQueue) != NULL)
		{
			xStatus = xQueueReceive(UsartRxQueue, &USartReceiveData, 0);
			if (xStatus == pdPASS)
			{
				vTaskSuspendAll();
				printf("%c\n", USartReceiveData.Value);
				xTaskResumeAll();
			}
		}
		TSVN_Led_Toggle(LED_D7);
		vTaskDelayUntil(&xLastWakeTime, 500);
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
void USART1_IRQHandler(void)
{
	xData ReceiveData;
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	ReceiveData.ID = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      ReceiveData.Value =(unsigned char)USART_ReceiveData(USART1);
      xQueueSendToBackFromISR(UsartRxQueue, &ReceiveData, &xHigherPriorityTaskWoken);
		}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
