#include <xTask.h>

#define MOTOR1_PWM				PWM_CHANNEL_2
#define MOTOR2_PWM				PWM_CHANNEL_3
#define MOTOR3_PWM				PWM_CHANNEL_4

#define CAN_DATA_LENGTH		6
#define TIMOUT_ID					1

xQueueHandle Pos_Queue;
xQueueHandle CanRxQueue;
xQueueHandle RxQueue;
xQueueHandle Moment_Queue;
TimerHandle_t TimeoutTimer;

static __IO long Timeout_CNT = 30;

xSemaphoreHandle UART_xCountingSemaphore;
Moment_Typedef Moment;

enum  {MOTOR1 = 0x00, MOTOR2, MOTOR3};
enum  {RESERVE, FORWARD};
enum  {USART_ID};

float ACS1_CALIB = -100.0f;
float ACS2_CALIB = -100.0f;
float ACS3_CALIB =  790.0f;

static __IO bool __FORCE_REQUEST = false;

//*******************Global*********************

//**********************************************

void vTimeoutCallback(TimerHandle_t pxTimer);

unsigned char Application_Init(void)
{
	unsigned int PWM_Max_Value = 0;
	PIDCoff Coff_MOTOR = {4.0, 0.01, 0.000001};

	TSVN_FOSC_Init();
	TSVN_Led_Init(ALL);
	TSVN_ACS712_Init();
	
	TSVN_QEI_TIM1_Init(400);
	TSVN_QEI_TIM3_Init(400);
	TSVN_QEI_TIM4_Init(400);
	
	TSVN_CAN_Init();
	
	TSVN_USART_Init();
	FIR_Init();
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
	
	PID_Init(MOTOR1, Coff_MOTOR);
	PID_Init(MOTOR2, Coff_MOTOR);
	PID_Init(MOTOR3, Coff_MOTOR);
	
	TSVN_TIM6_Init(2000);
	Pos_Queue = xQueueCreate(200, sizeof(Pos_TypeDef));
	CanRxQueue = xQueueCreate(200, sizeof(CanRxMsg));
	Moment_Queue = xQueueCreate(200, sizeof(Moment_Typedef));
	RxQueue = xQueueCreate(200, sizeof(xData));
	
	UART_xCountingSemaphore = xSemaphoreCreateCounting(200, 0);
	
	if (Pos_Queue != NULL && CanRxQueue != NULL  && 
			Moment_Queue != NULL && UART_xCountingSemaphore != NULL && RxQueue != NULL)
		return SUCCESS;
	return ERROR;
}

void Application_Run(void)
{
	TimeoutTimer = xTimerCreate("Timeout", 100/portTICK_PERIOD_MS , pdTRUE, (void*) TIMOUT_ID, vTimeoutCallback);
	if (TimeoutTimer != NULL)
		xTimerStart(TimeoutTimer, 0);
	xTaskCreate(POS_TASK, 	"POS", POS_TASK_STACK_SIZE, NULL, POS_TASK_PRIORITY, NULL);	
	xTaskCreate(TRANSFER_TASK, 	"TRANSFER", TRANSFER_TASK_STACK_SIZE, NULL, TRANSFER_TASK_PRIORITY, NULL);	
	xTaskCreate(MOMENT_TASK, 	"MOMENT", MOMENT_TASK_STACK_SIZE, NULL, MOMENT_TASK_PRIORITY, NULL);	
	vTaskStartScheduler();
}

void MOMENT_TASK(void *pvParameters)
{
	float Theta[3];
	float Cordinate[3];
	float F[3] = {0.0, 0.0, 0.0};
	float Phi[3];
	float Moment[3];
	portBASE_TYPE xStatus;
	CanRxMsg CanReceiveData;
	Moment_Typedef M;
	unsigned char Status;
	while(1)
	{
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
				F[2] += 6;
			}
			__FORCE_REQUEST = false;
		}
		Theta[0] = ((float)TSVN_QEI_TIM1_Value()*0.9)/7.0;
		Theta[1] = ((float)TSVN_QEI_TIM4_Value()*0.9)/7.0;
		Theta[2] = ((float)TSVN_QEI_TIM3_Value()*0.9)/7.0;		
		Status = (char)Delta_CalcForward(Theta[0], Theta[1], Theta[2], &Cordinate[0], &Cordinate[1], &Cordinate[2]);
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
				  M.Mx = (Moment[0]*2000.0 > 550.0)?550.0:Moment[0]*2000.0;
					M.My = (Moment[1]*2000.0 > 550.0)?550.0:Moment[1]*2000.0;
					M.Mz = (Moment[2]*2000.0 > 550.0)?550.0:Moment[2]*2000.0;
					xQueueSendToBack(Moment_Queue, &M, 1);
			}
		}
		TSVN_Led_Toggle(LED_D6);
		vTaskDelay(50);
	}
}
void TRANSFER_TASK(void *pvParameters)
{
	portBASE_TYPE xStatus;
	xData ReadValue;
	char *Cmd;
	unsigned char i = 0;
	PIDCoff PID_Coff;
	float CurentValue_MOTOR[3];
	while(1)
	{	
		xSemaphoreTake(UART_xCountingSemaphore, portMAX_DELAY);
		if (uxQueueMessagesWaiting(RxQueue) != NULL)
		{
			xStatus = xQueueReceive(RxQueue, &ReadValue, 1);
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
								PID_Coff.Kp = atof(Cmd);
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								PID_Coff.Ki = atof(Cmd);
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								PID_Coff.Kd = atof(Cmd);
								PID_Init(MOTOR1, PID_Coff);
								vTaskSuspendAll();
								printf("PID MOTOR1: %0.5f\t%0.5f\t%0.5f\n", PID_Coff.Kp, PID_Coff.Ki, PID_Coff.Kd);
								xTaskResumeAll();
							}
							else if (!strcmp(Cmd, "PID2"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								PID_Coff.Kp = atof(Cmd);
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								PID_Coff.Ki = atof(Cmd);
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								PID_Coff.Kd = atof(Cmd);
								PID_Init(MOTOR2, PID_Coff);
								vTaskSuspendAll();
								printf("PID MOTOR2: %0.5f\t%0.5f\t%0.5f\n", PID_Coff.Kp, PID_Coff.Ki, PID_Coff.Kd);
								xTaskResumeAll();
							}
							else if (!strcmp(Cmd, "PID3"))
							{
								Cmd = TSVN_Get_Parameters(2, TSVN_USART_Get_Frame());
								PID_Coff.Kp = atof(Cmd);
								Cmd = TSVN_Get_Parameters(3, TSVN_USART_Get_Frame());
								PID_Coff.Ki = atof(Cmd);
								Cmd = TSVN_Get_Parameters(4, TSVN_USART_Get_Frame());
								PID_Coff.Kd = atof(Cmd);
								PID_Init(MOTOR3, PID_Coff);
								vTaskSuspendAll();
								printf("PID MOTOR3: %0.5f\t%0.5f\t%0.5f\n", PID_Coff.Kp, PID_Coff.Ki, PID_Coff.Kd);
								xTaskResumeAll();
							}
							else if (!strcmp(Cmd, "AUTO"))
							{
								vTaskSuspendAll();
								CurentValue_MOTOR[MOTOR1] = 0;
								CurentValue_MOTOR[MOTOR2] = 0;
								CurentValue_MOTOR[MOTOR3] = 0;
								for (i = 0; i<200; i++)
								{
									CurentValue_MOTOR[MOTOR1] += TSVN_ACS712_Read(ACS_1);
									CurentValue_MOTOR[MOTOR2] += TSVN_ACS712_Read(ACS_2);
									CurentValue_MOTOR[MOTOR3] += TSVN_ACS712_Read(ACS_3);
								}
								ACS1_CALIB = CurentValue_MOTOR[MOTOR1]/200.0;
								ACS2_CALIB = CurentValue_MOTOR[MOTOR2]/200.0;
								ACS3_CALIB = CurentValue_MOTOR[MOTOR3]/200.0;
								printf("Calib: %0.5f\t%0.5f\t%0.5f\n", ACS1_CALIB, ACS2_CALIB, ACS3_CALIB);
								xTaskResumeAll();
							}
						}		
					}
			}
		}
	}
}

void POS_TASK(void *pvParameters)
{
	float Theta[3];
	Pos_TypeDef Pos;
	char Status = 0;
	static __IO uint8_t Timeout = 0;
	static __IO bool isSend = false;
	CanTxMsg CanSendData;
	CanSendData.StdId = CAN_MASTER_STD_ID;
	CanSendData.IDE = 	CAN_ID_STD;
	CanSendData.RTR = 	CAN_RTR_DATA;
	CanSendData.DLC = 	CAN_DATA_LENGTH;
	while(1)
	{
			if (__FORCE_REQUEST)
			{
				if (!isSend)
				{
					CanSendData.Data[0] = 'F';
					vTaskSuspendAll();
					CAN_Transmit(CAN1, &CanSendData);
					xTaskResumeAll();
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
						
				}
			}
			if (!__FORCE_REQUEST)
			{
				vTaskSuspendAll();
				CAN_Transmit(CAN1, &CanSendData);
				xTaskResumeAll();
			}
			TSVN_Led_Toggle(LED_D4);
			vTaskDelay(100);
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
void TIM6_IRQHandler(void)
{
	static float CurentValue_MOTOR[3];
	static portBASE_TYPE xStatus;
	static Moment_Typedef Moment;
	static float Moments[3];
	static long PWM_MOTOR[3]; 
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		FIR_CollectData(SEN_MOTOR1, TSVN_ACS712_Read(ACS_1));
		CurentValue_MOTOR[MOTOR1] = AMES_Filter(SEN_MOTOR1) - ACS1_CALIB;
		FIR_CollectData(SEN_MOTOR2 ,TSVN_ACS712_Read(ACS_2));
		CurentValue_MOTOR[MOTOR2] = AMES_Filter(SEN_MOTOR2) - ACS2_CALIB;
		FIR_CollectData(SEN_MOTOR3,TSVN_ACS712_Read(ACS_3));
		CurentValue_MOTOR[MOTOR3] = AMES_Filter(SEN_MOTOR3) - ACS3_CALIB;
		PWM_MOTOR[MOTOR1] = PID_Calculate(MOTOR1, Moments[0], CurentValue_MOTOR[MOTOR1]);
		if (PWM_MOTOR[MOTOR1] < 0)
				DIR_Change(MOTOR1, RESERVE);
		else
				DIR_Change(MOTOR1, FORWARD);
		TSVN_PWM_TIM5_Set_Duty(abs(PWM_MOTOR[MOTOR1]), MOTOR1_PWM);
		
		PWM_MOTOR[MOTOR2] = PID_Calculate(MOTOR2, Moments[1], CurentValue_MOTOR[MOTOR2]);
		if (PWM_MOTOR[MOTOR2] < 0)
				DIR_Change(MOTOR2, RESERVE);
		else
				DIR_Change(MOTOR2, FORWARD);
		TSVN_PWM_TIM5_Set_Duty(abs(PWM_MOTOR[MOTOR2]), MOTOR2_PWM);
		
		PWM_MOTOR[MOTOR3] = PID_Calculate(MOTOR3, Moments[2], CurentValue_MOTOR[MOTOR3]);
		
		if (PWM_MOTOR[MOTOR3] < 0)
				DIR_Change(MOTOR3, RESERVE);
		else
				DIR_Change(MOTOR3, FORWARD);
		TSVN_PWM_TIM5_Set_Duty(abs(PWM_MOTOR[MOTOR3]), MOTOR3_PWM);
		
		if (uxQueueMessagesWaitingFromISR(Moment_Queue) != NULL)
		{
			xStatus = xQueueReceiveFromISR(Moment_Queue, &Moment, 0);
			if (xStatus == pdPASS)
			{
				Moments[0] = Moment.Mx;
				Moments[1] = Moment.My;
				Moments[2] = Moment.Mz;
			}
		}
		
	}
}
void vTimeoutCallback(TimerHandle_t pxTimer)
{
	TSVN_Led_Toggle(LED_D7);
	if (Timeout_CNT-- == 0)
	{
		Timeout_CNT = 30;
		__FORCE_REQUEST = true;
	}
}

