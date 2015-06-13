#include <FreeRTOS.h>
#include <task.h>
#include <Queue.h>
#include <semphr.h>
#include <Delta.h>
#include <Jacobian.h>
#include <Hardware.h>
#include <AMES_LED.h> 
#include <AMES_CLOCK.h>
#include <AMES_QEI.h>
#include <AMES_PID.h>
#include <AMES_PWM.h>
#include <AMES_FIR.h>
#include <AMES_CAN.h>
#include <AMES_ACS712.h>
#include <AMES_USART.h>

#define PID_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE + 256)
#define ENCODER_TASK_STACK_SIZE	(configMINIMAL_STACK_SIZE + 512)
#define USART_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 128)

#define PID_TASK_PRIORITY					(tskIDLE_PRIORITY + 1)
#define ENCODER_TASK_PRIORITY			(tskIDLE_PRIORITY + 1)
#define USART_TASK_PRIORITY				(tskIDLE_PRIORITY + 1)

#define CAN_MASTER_STD_ID 			0x00
#define CAN_MASTER_EXT_ID 			0x01

#define CAN_SLAVE_STD_ID 				0x02
#define CAN_SLAVE_EXT_ID 				0x03

typedef struct Data_Structure
{
	char ID;
	unsigned char Value;
}xData;

typedef struct Moment_Structure
{
	float Moment_MOTOR1;
	float Moment_MOTOR2;
	float Moment_MOTOR3;
}xMomentData;

unsigned char Application_Init(void);
void Application_Run(void);
void PID_TASK(void *pvParameters);
void ENCODER_TASK(void *pvParameters);
void USART_TASK(void *pvParameters);

