/****************************************************************************/
/*FIR Filter library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 12/03/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define MAX_INPUT_LEN   100
#define MAX_FLT_LEN     63
#define BUFFER_LEN      (MAX_FLT_LEN - 1 + MAX_INPUT_LEN)
#define FILTER_LEN  		63
#define NUM_SAMPLES   	10
#define NUM_CHANNEL			3

typedef enum
{
	SEN_MOTOR1 = 0x00,
	SEN_MOTOR2,
	SEN_MOTOR3,
}SEN_CHANNEL;

enum{NONE, DONE};

void FIR_Init(void);
void Median_Filter(double *input, double *output, int length);
void FIR_Filter(double *Coeffs, double *input, double *output, int length, int filterlength);
char FIR_CollectData(SEN_CHANNEL Channel, double Input);
double AMES_Filter(SEN_CHANNEL Chennel);

/*
void SENSOR_TASK(void *pvParameters)
{
	xData ReceiveData;
	double SensorValue = 0;
	unsigned int i = 0;
	while(1)
	{	
		if (FIR_CollectData(TSVN_ACS712_Read(ACS_2)) == DONE)
		{
			SensorValue = AMES_Filter();
			ReceiveData.ID = 1;
			ReceiveData.Value = SensorValue;
			xQueueSendToBack(RxQueue, &ReceiveData, 1);
			xSemaphoreGive(UART_xCountingSemaphore);
		}
		vTaskDelay(2);
	}
}

*/

