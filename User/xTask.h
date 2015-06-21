/****************************************************************************/
/*Delta Robot task library.																									*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 20/06/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/
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
#include <AMES_FIR.h>
#include <AMES_CAN.h>
#include <AMES_ACS712.h>
#include <AMES_USART.h>
#include <AMES_TIMER.h>
#include <AMES_PID.h>
#include <AMES_PWM.h>

#define POS_TASK_STACK_SIZE					(configMINIMAL_STACK_SIZE)
#define TRANSFER_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE)
#define MOMENT_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE + 128)

#define POS_TASK_PRIORITY						(tskIDLE_PRIORITY + 1)
#define TRANSFER_TASK_PRIORITY			(tskIDLE_PRIORITY + 1)
#define MOMENT_TASK_PRIORITY				(tskIDLE_PRIORITY + 1)

#define CAN_MASTER_STD_ID 			0x00
#define CAN_MASTER_EXT_ID 			0x01

#define CAN_SLAVE_STD_ID 				0x02
#define CAN_SLAVE_EXT_ID 				0x03

typedef struct Data_Structure
{
	char ID;
	unsigned char Value;
}xData;

typedef struct Pos_Structure
{
	float Px;
	float Py;
	float Pz;
}Pos_TypeDef;

typedef struct Moment_Struct
{
	float Mx;
	float My;
	float Mz;
}Moment_Typedef;

unsigned char Application_Init(void);
void Application_Run(void);
void POS_TASK(void *pvParameters);
void TRANSFER_TASK(void *pvParameters);
void MOMENT_TASK(void *pvParameters);
