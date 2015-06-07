#include <FreeRTOS.h>
#include <task.h>
#include <list.h>
#include <Queue.h>
#include <semphr.h>
#include <croutine.h>
#include <AMES_LED.h> 
#include <AMES_CLOCK.h>
#include <AMES_BUTTON.h>


#define LED4_TASK_STACK_SIZE 			(configMINIMAL_STACK_SIZE)
#define LED5_TASK_STACK_SIZE 			(configMINIMAL_STACK_SIZE)
#define LED6_TASK_STACK_SIZE 			(configMINIMAL_STACK_SIZE)

#define LED4_TASK_PRIORITY				(tskIDLE_PRIORITY + 1)
#define LED5_TASK_PRIORITY				(tskIDLE_PRIORITY + 1)
#define LED6_TASK_PRIORITY				(tskIDLE_PRIORITY + 5)

void Application_Init(void);
void Application_Run(void);
void Application_Exit(void);


void LED4_TASK(void *pvParameters);
void LED5_TASK(void *pvParameters);
void LED6_TASK(void *pvParameters);
