#include <DeltaTask.h>

#define MOTOR1_PWM				PWM_CHANNEL_2
#define MOTOR2_PWM				PWM_CHANNEL_3
#define MOTOR3_PWM				PWM_CHANNEL_4
#define CAN_DATA_LENGTH		6

xTaskHandle LED_TASK_HANDLER;
xTaskHandle	UART_TASK_HANDLER;
xTaskHandle	PID_TASK_HANDLER;
xTaskHandle	ENCODER_TASK_HANDLER;

xQueueHandle 	RxQueue;
xQueueHandle  PidQueue;
xQueueHandle  CanRxQueue;


xSemaphoreHandle UART_xCountingSemaphore;
xSemaphoreHandle PID_xCountingSemaphore;
xSemaphoreHandle CANRx_xCountingSemaphore;

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
	PID_Init(MOTOR1, 0, 0, 0);
	PID_Init(MOTOR2, 0, 0, 0);
	PID_Init(MOTOR3, 0, 0, 0);
	
	TSVN_USART_Init();
	TSVN_CAN_Init();
	
	TSVN_QEI_TIM1_Init(400);
	TSVN_QEI_TIM3_Init(400);
	TSVN_QEI_TIM4_Init(400);
	
	RxQueue  = xQueueCreate(100, sizeof(xData));
	CanRxQueue  = xQueueCreate(100, sizeof(CanRxMsg));
	PidQueue = xQueueCreate(100, sizeof(xMomentData));
	UART_xCountingSemaphore = xSemaphoreCreateCounting(100, 0);
	PID_xCountingSemaphore = xSemaphoreCreateCounting(100, 0);
	CANRx_xCountingSemaphore = xSemaphoreCreateCounting(100, 0);

	if (CanRxQueue != NULL && PidQueue != NULL
		&& RxQueue != NULL && UART_xCountingSemaphore != NULL 
	  && PID_xCountingSemaphore != NULL
		&& CANRx_xCountingSemaphore != NULL)		
			return SUCCESS;
	else
		return ERROR;
}
void Application_Run(void)
{
	xTaskCreate(UART_TASK,  		(signed char*)  "UART", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, &UART_TASK_HANDLER);
	xTaskCreate(PID_TASK,   		(signed char*) 	"PID", PID_TASK_STACK_SIZE, NULL, PID_TASK_PRIORITY, &PID_TASK_HANDLER);	
	xTaskCreate(LED_TASK,   		(signed char*) 	"LED", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_TASK_HANDLER);
	xTaskCreate(ENCODER_TASK,   (signed char*) 	"ENCODER", ENCODER_TASK_STACK_SIZE, NULL, ENCODER_TASK_PRIORITY, &ENCODER_TASK_HANDLER);	
	vTaskStartScheduler();
}
void ENCODER_TASK(void *pvParameters)
{
	float Theta[3];
	float Phi[3];
	float F[3] = {0.0, 0.0, 15.0};
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
		SendMoment.Moment_MOTOR1 = Moment[0];
		SendMoment.Moment_MOTOR2 = Moment[1];
		SendMoment.Moment_MOTOR3 = Moment[2];
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
			vTaskDelay(50);
	}
}

void PID_TASK(void  *pvParameters)
{
	portBASE_TYPE xStatus;
	xMomentData ReadValue;
	float CurentValue_MOTOR[3];
	float SetPoint_MOTOR[3] = {0, 0, 0};
	long PWM_MOTOR[3] = {0, 0, 0}; 
	
	PID_Init(MOTOR1, 1, 0, 0.0001);
	PID_Init(MOTOR2, 1, 0, 0.0001);
	PID_Init(MOTOR3, 1, 0, 0.0001);
	
	TSVN_PWM_TIM5_Set_Duty(0, MOTOR3_PWM);
	TSVN_PWM_TIM5_Set_Duty(0, MOTOR2_PWM);
	TSVN_PWM_TIM5_Set_Duty(0, MOTOR1_PWM);
	
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
				 //printf("%0.5f,\n", CurentValue_MOTOR[MOTOR1]);
				 //printf("%0.5f,\t%0.5f,\t%0.5f\n", ReadValue.Moment_MOTOR1, ReadValue.Moment_MOTOR2, ReadValue.Moment_MOTOR3);
			}
		}
		vTaskDelay(100);
	}
}

void LED_TASK(void  *pvParameters)
{
	while(1)
	{
		TSVN_Led_On(LED_D4);
		vTaskDelay(20);
		TSVN_Led_Off(LED_D4);
		vTaskDelay(500);
	}
}

void UART_TASK(void *pvParameters)
{
	xData ReadValue;
	xMomentData SendValue;
	portBASE_TYPE xStatus;
	char *Cmd;
	double Temp;
	static float Kp = 0, Ki = 0, Kd = 0;
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
							if (!strcmp(Cmd, "PID1"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Kp = atof(Cmd);
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								Ki = atof(Cmd);
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								Kd = atof(Cmd);
								PID_Init(MOTOR1, Kp, Ki, Kd);
								printf("PID1 change to: %0.2f\t%0.2f\t%0.2f\n", Kp, Ki, Kd);
							}
							else if (!strcmp(Cmd, "PID2"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Kp = atof(Cmd);
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								Ki = atof(Cmd);
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								Kd = atof(Cmd);
								PID_Init(MOTOR2, Kp, Ki, Kd);
								printf("PID2 change to: %0.2f\t%0.2f\t%0.2f\n", Kp, Ki, Kd);
							}
							else if (!strcmp(Cmd, "PID3"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Kp = atof(Cmd);
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								Ki = atof(Cmd);
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								Kd = atof(Cmd);
								PID_Init(MOTOR1, Kp, Ki, Kd);
								printf("PID3 change to: %0.2f\t%0.2f\t%0.2f\n", Kp, Ki, Kd);
							}
							else if (!strcmp(Cmd, "SETP"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								Temp = atof(Cmd);
								SendValue.Moment_MOTOR1 = (float)Temp;
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								Temp = atof(Cmd);
								SendValue.Moment_MOTOR2 = Temp;
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								Temp = atof(Cmd);
								SendValue.Moment_MOTOR3 = Temp;
								printf("Moment Set: %0.2f\t%0.2f\t%0.2f\n", SendValue.Moment_MOTOR1, SendValue.Moment_MOTOR2, SendValue.Moment_MOTOR3);
							}
							else if (!strcmp(Cmd, "PWM1"))
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
				else if (ReadValue.ID == SEN1)
					{
						printf("Dong Motor 1: %0.5f\t", ReadValue.Value);
					}
				else if (ReadValue.ID == SEN2)
				{
					printf("Dong Motor 2: %0.5f\t", ReadValue.Value);
				}
				else if (ReadValue.ID == SEN3)
				{
					printf("Dong Motor 3: %0.5f\n", ReadValue.Value);
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

