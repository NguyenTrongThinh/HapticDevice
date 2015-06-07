#include <AppTask.h>

xTaskHandle LED4_TASK_HANDLER;
xTaskHandle LED5_TASK_HANDLER;
xTaskHandle LED6_TASK_HANDLER;
xSemaphoreHandle xCountingSemaphore[2];

typedef struct Button_Structure
{
	unsigned char ID;
	unsigned char Value;
}xButton;


void Application_Init(void)
{
	TSVN_FOSC_Init();
	TSVN_Led_Init(LED_D4);
	TSVN_Led_Init(LED_D5);
	TSVN_Led_Init(LED_D6);
	TSVN_BUT_Init(S2);
	TSVN_BUT_Init(S3);
	xCountingSemaphore[0] = xSemaphoreCreateCounting(10, Null);
	xCountingSemaphore[1] = xSemaphoreCreateCounting(10, Null);
	
}
void Application_Run(void)
{
	if(xCountingSemaphore[1] != NULL && xCountingSemaphore[0] != NULL)
	{
		xTaskCreate(LED4_TASK, (signed char*) 	"LED4", LED4_TASK_STACK_SIZE, NULL, LED4_TASK_PRIORITY, &LED4_TASK_HANDLER);
		xTaskCreate(LED5_TASK, (signed char*) 	"LED5", LED5_TASK_STACK_SIZE, NULL, LED5_TASK_PRIORITY, &LED5_TASK_HANDLER);
		xTaskCreate(LED6_TASK, (signed char*) 	"LED6", LED6_TASK_STACK_SIZE, NULL, LED6_TASK_PRIORITY, &LED6_TASK_HANDLER);		
		vTaskStartScheduler();
	}
}
void Application_Exit(void)
{
	
}

void LED4_TASK(void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(xCountingSemaphore[0], portMAX_DELAY);
		TSVN_Led_Toggle(LED_D4);
	}
}
void LED5_TASK(void *pvParameters)
{
		while(1)
	{
		xSemaphoreTake(xCountingSemaphore[1], portMAX_DELAY);
		TSVN_Led_Toggle(LED_D5);
		vTaskDelay(500);
	}
}
void LED6_TASK(void *pvParameters)
{
		while(1)
	{
		TSVN_Led_Toggle(LED_D6);
		vTaskDelay(50);
	}
}
void vApplicationIdleHook(void)
{
	
}

void EXTI0_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line0);
		xSemaphoreGiveFromISR(xCountingSemaphore[0], &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken == pdTRUE)
	  vPortYieldFromISR();
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

