#include <AMES_ACS712.h>

const float CALIB1 = 0.0;
const float	CALIB2 = 0.0;
const float CALIB3 = 0.0;

__IO uint16_t ADC_Value[4];


static void __GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	#if ((USE_ADC_PB0 == 1) || (USE_ADC_PB1 == 1))
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		#if (USE_ADC_PB0 == 1)
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
		#endif
		#if (USE_ADC_PB1 == 1)
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
		#endif
	#endif
	#if (USE_ADC_PA4 == 1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	
	#if (USE_ADC_PC0 == 1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	#endif
}
static void __ADC_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
  ADC_InitStructure.ADC_NbrOfChannel = (USE_ADC_PB0 + USE_ADC_PB1 + USE_ADC_PA4 + USE_ADC_PC0); //Change
	ADC_Init(ADC1, &ADC_InitStructure);
	#if (USE_ADC_PB0 == 1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);
	#endif
	#if (USE_ADC_PB1 == 1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_55Cycles5);
	#endif
	#if (USE_ADC_PA4 == 1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
	#endif
	#if (USE_ADC_PC0 == 1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 4, ADC_SampleTime_55Cycles5);
	#endif
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void __DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = USE_ADC_PB0 + USE_ADC_PB1 + USE_ADC_PA4 + USE_ADC_PC0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;

	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}
void TSVN_ACS712_Init(void)
{
	__GPIO_Init();
	__DMA_Init();
	__ADC_Init();
	
}
int TSVN_ACS712_Read_ADC(unsigned char Channel)
{
	switch(Channel)
	{
		case ACS_1: return ADC_Value[0] - 2048 - CALIB_ADC;
		case ACS_2: return ADC_Value[1] - 2048 - CALIB_ADC;
		case ACS_3: return ADC_Value[2] - 2048 - CALIB_ADC;
		case ACS_4: return ADC_Value[3] - 2048 - CALIB_ADC;
	}
	return 0;
}
double TSVN_ACS712_Read(unsigned char Channel)
{
	static double Voltage;
	static double Amps;
	switch (Channel)
	{
		case ACS_1:{
								Voltage = (float)ADC_Value[0]*3300.0/4096.0;
								Amps = (Voltage-1650.0) / 185.0;
								return (Amps + CALIB1)*1000.0;
							 }
		case ACS_2:{
								Voltage = (float)ADC_Value[1]*3300.0/4096.0;
								Amps = (Voltage-1650.0) / 185.0;
								return (Amps + CALIB2)*1000.0;
							 }
		case ACS_3:{
								Voltage = (float)ADC_Value[2]*3300.0/4096.0;
								Amps = (Voltage-1650.0) / 185.0;
								return (Amps + CALIB3)*1000.0;
							 }
		case ACS_4:{
								Voltage = (float)ADC_Value[3]*3300.0/4096.0;
								Amps = (Voltage-1650.0) / 185.0;
								return Amps*1000.0;
							 }
	}
	return 0;
}
