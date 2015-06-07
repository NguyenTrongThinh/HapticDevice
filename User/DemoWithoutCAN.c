#include <FreeRTOS.h>
#include <task.h>
#include <list.h>
#include <Queue.h>
#include <semphr.h>
#include <croutine.h>
#include <AMES_LED.h> 
#include <AMES_CLOCK.h>
#include <AMES_BUTTON.h>
#include <AMES_USART.h>
#include <AMES_QEI.h>
#include <AMES_PWM.h>
#include <AMES_TIMER.h>
#include <AMES_EEPROM.h>
#include <AMES_CAN.h>


#define USART_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE)
#define LED_TASK_STACK_SIZE				(configMINIMAL_STACK_SIZE)

#define USART_TASK_PRIORITY				(tskIDLE_PRIORITY + 10)
#define LED_TASK_PRIORITY					(tskIDLE_PRIORITY + 1)

typedef struct Data_Structure
{
	char ID;
	char Value;
}xData;

xTaskHandle 	USART_TASK_HANDLER;
xTaskHandle 	LED_TASK_HANDLER;
xQueueHandle 	RxQueue;
xSemaphoreHandle xCountingSemaphore;

unsigned int Temp;

void USART_TASK(void *pvParameters);
void LED_TASK(void *pvParameters);

int main(void)
{
	TSVN_FOSC_Init();
	TSVN_USART_Init();
	TSVN_Led_Init(ALL);
	
	Temp = TSVN_PWM_TIM5_Init(5000);
	TSVN_PWM_TIM5_Start();
	TSVN_TIM6_Init(500000);
	TSVN_TIM7_Init(200000);
	TSVN_EEPROM_Init();
	RxQueue = xQueueCreate(255, sizeof(xData));
	xCountingSemaphore = xSemaphoreCreateCounting(10, 0);
	if (RxQueue != NULL && xCountingSemaphore != NULL)
	{
		xTaskCreate(USART_TASK, (signed char*) 	"USART", USART_TASK_STACK_SIZE, NULL, USART_TASK_PRIORITY, &USART_TASK_HANDLER);
		xTaskCreate(LED_TASK, (signed char*) 	"LED", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_TASK_HANDLER);
		vTaskStartScheduler();
	}
	while(1);
}

void USART_TASK(void *pvParameters)
{
	unsigned int Temper = 0;
	unsigned char Data;
	Temper+=5000;
	TSVN_PWM_TIM5_Set_Duty(Temper, PWM_CHANNEL_2);
	TSVN_PWM_TIM5_Set_Duty(Temper, PWM_CHANNEL_3);
	TSVN_PWM_TIM5_Set_Duty(Temper, PWM_CHANNEL_4);
//	for(Temper = 0; Temper<256; Temper++)
//	{
//		TSVN_EEPROM_Write_Byte(Temper, Temper);
//		vTaskDelay(1);
//	}
	Temper = 0;
	while(1)
	{	
		Data = TSVN_EEPROM_Read_Byte(Temper);
		printf("%d\n", Data);
		Temper++;
		vTaskDelay(200);
	}
//	xData ReadValue;
//	portBASE_TYPE xStatus;
//	char *Cmd;
//	char i=0;
//	while(1)
//	{
//		
//		xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
//		if (uxQueueMessagesWaiting(RxQueue) != NULL)
//		{
//			xStatus = xQueueReceive(RxQueue, &ReadValue, 0);
//			if (xStatus == pdPASS)
//			{
//				if (TSVN_USART_Create_Frame(ReadValue.Value) == End)
//				{
//					for (i = 0; i<10; i++)
//					{
//					Cmd = TSVN_Get_Parameters(i, TSVN_USART_Get_Frame());
//				  printf("%s\n", Cmd);	
//					}
//				}
//			}
//		}
//	}
}
void LED_TASK(void *pvParameters)
{
	while(1)
	{
		TSVN_Led_Toggle(LED_D4);
		vTaskDelay(500);
	}
}
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
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TSVN_Led_Toggle(LED_D5);
	}
}
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		TSVN_Led_Toggle(LED_D6);
	}
}


