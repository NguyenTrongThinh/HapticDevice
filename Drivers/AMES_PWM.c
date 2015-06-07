#include <AMES_PWM.h>
unsigned int *TimerPeriod;
unsigned int *PrescaleValue;


static void Timer_Calculate(unsigned long Hz, unsigned int *TimerPeriod, unsigned int *Prescale)
{
	unsigned int Du = 0;
	unsigned char Restrict = 0;
	*Prescale = 0;
	*TimerPeriod = 0;
	do
	{
		(*Prescale)++;
		*TimerPeriod = SystemCoreClock / (((*Prescale) + 1)*Hz) - 1;
		Du = SystemCoreClock % ((*Prescale + 1)*Hz);
		if ((*Prescale) == 0xffff)
		{
			Restrict = 1;
			*Prescale = 0;
			*TimerPeriod = 0;
			break;
		}
	} while ((*TimerPeriod) > 65535 || Du != 0 || Restrict);
}
unsigned int TSVN_PWM_TIM5_Init(unsigned long Hz)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TimerPeriod = (unsigned int*)malloc(sizeof(unsigned int));
	PrescaleValue = (unsigned int*)malloc(sizeof(unsigned int));
	if (TimerPeriod != 0 && PrescaleValue != 0)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		#if (LQFP144 == 1)
				GPIO_InitStructure.GPIO_Pin = TIM5_CHANNEL_1_PIN;
				GPIO_Init(TIM5_CHANNEL_1_PORT, &GPIO_InitStructure);
		#endif
		
		GPIO_InitStructure.GPIO_Pin = TIM5_CHANNEL_2_PIN | TIM5_CHANNEL_3_PIN | TIM5_CHANNEL_4_PIN;
		GPIO_Init(TIM5_CHANNEL_2_PORT, &GPIO_InitStructure);	
		Timer_Calculate(Hz, TimerPeriod, PrescaleValue);
		TIM_TimeBaseStructure.TIM_Period = *TimerPeriod;
		TIM_TimeBaseStructure.TIM_Prescaler = *PrescaleValue;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//Setduty
		TIM_OCInitStructure.TIM_Pulse = 0;					//Set duty
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		#if (LQFP144 == 1)
			TIM_OC1Init(TIM5, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
		#endif
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;					//Set duty
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;					//Set duty
		TIM_OC3Init(TIM5, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;					//Set duty
		TIM_OC4Init(TIM5, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM5, ENABLE);
		TIM_Cmd(TIM5, ENABLE);			
	}
	return *TimerPeriod;
}
void TSVN_PWM_TIM5_Set_Duty(unsigned int Ratio, unsigned char Channel)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//Setduty
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = Ratio;	
	switch(Channel)
	{
		case PWM_CHANNEL_1:	
		#if (LQFP144 == 1)
			TIM_OC1Init(TIM5, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
		#endif
			break;
		case PWM_CHANNEL_2:
			TIM_OC2Init(TIM5, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
			break;
		case PWM_CHANNEL_3:
			TIM_OC3Init(TIM5, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
			break;
		case PWM_CHANNEL_4:
			TIM_OC4Init(TIM5, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
			break;
	}
}

void TSVN_PWM_TIM5_Start(void)
{
	TIM_Cmd(TIM5, ENABLE);
}
void TSVN_PWM_TIM5_Stop(void)
{
	TIM_Cmd(TIM5, DISABLE);
}

