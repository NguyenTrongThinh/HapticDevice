#include <AMES_CLOCK.h>

void TSVN_FOSC_Init(void)
{
	RCC_DeInit();								 //Reset RCC to default
	RCC_HSEConfig(RCC_HSE_ON);						 //Enable HSE
	RCC_HSICmd(DISABLE);							 //Enable HSI
	RCC_PREDIV1Config(RCC_PREDIV1_Source_HSE, RCC_PREDIV1_Div2); //Predivide scale factor 2
	RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_4);		 //x4PLL Prediv Source 
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);			 //Select PLL as system Clock
	RCC_HCLKConfig(RCC_SYSCLK_Div2);					 //AHB Register clock 25Mhz
	RCC_PCLK1Config(RCC_HCLK_Div2);					 //APB clock source 25Mhz
	RCC_PCLK2Config(RCC_HCLK_Div2);					 //APB clock source 25Mhz	
	RCC_MCOConfig(RCC_MCO_NoClock);					 //Disable clock on MCO pin
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);					 //ADC clock 3.125Mhz 
  RCC_AdjustHSICalibrationValue(0);					 //Clib HSI value
	RCC_ClockSecuritySystemCmd(ENABLE);					 //Enable Security clock
	RCC_LSICmd(DISABLE);							 //Disable low Speed internal clock
	RCC_PLLCmd(ENABLE);							 //Enable PLL
	while(RCC_WaitForHSEStartUp() != SUCCESS);			 //Wait for HSE start up	
	SystemInit();
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); //25Mhz / 8
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
}
