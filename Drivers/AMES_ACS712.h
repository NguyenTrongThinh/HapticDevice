/****************************************************************************/
/*ACS712 library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 12/03/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/

#ifndef __AMES_ACS712_H__
	#define __AMES_ACS712_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_dma.h>
#include <stdlib.h>

#define USE_ADC_PB0  1
#define USE_ADC_PB1	 1
#define USE_ADC_PA4	 1
#define USE_ADC_PC0	 0
#define ARRAYSIZE 	 32
#define OUT_SEN 		 185

/*
USE 185 for 5A
100 for 20A
66 for 30A
*/
#define ADC1_ADDRESS ADC1->DR
#define CALIB_ADC 196



enum{ACS_1, ACS_2, ACS_3, ACS_4};

void TSVN_ACS712_Init(void);
double TSVN_ACS712_Read(unsigned char Channel);
int TSVN_ACS712_Read_ADC(unsigned char Channel);

#ifdef __cplusplus
}
#endif
#endif /* __AMES_ACS712_H__ */
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
//DMA dung noi boj Trong thu vien
//Struct dung noi bo trong thu vien
//Dung ham de tra ra gia tri sau khi tinh toan ra dong.

