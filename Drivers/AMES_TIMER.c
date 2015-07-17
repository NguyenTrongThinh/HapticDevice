#include <AMES_TIMER.H>
unsigned int *Period;
unsigned int *Prescale;

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
void TSVN_TIM6_Init(unsigned long us)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	unsigned long Freq = (unsigned long)1000000/us;
	Period = (unsigned int*)malloc(sizeof(unsigned int));
	Prescale = (unsigned int*)malloc(sizeof(unsigned int));
	if (Period != 0 && Prescale != 0)
	{
		Timer_Calculate(Freq, Period, Prescale);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		TIM_TimeBaseInitStructure.TIM_Prescaler = *Prescale;  //Tinh lai sau
		TIM_TimeBaseInitStructure.TIM_Period = *Period; 					//Tinh lai sau
		TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
		TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TIM_Cmd(TIM6, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	free(Period);
	free(Prescale);
}

void TSVN_TIM7_Init(unsigned long us)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	unsigned long Freq = (unsigned long)1000000/us;
	Period = (unsigned int*)malloc(sizeof(unsigned int));
	Prescale = (unsigned int*)malloc(sizeof(unsigned int));
	if (Period != 0 && Prescale != 0)
	{
		Timer_Calculate(Freq, Period, Prescale);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		TIM_TimeBaseInitStructure.TIM_Prescaler = *Prescale;  //Tinh lai sau
		TIM_TimeBaseInitStructure.TIM_Period = *Period; 					//Tinh lai sau
		TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);
		TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		TIM_Cmd(TIM7, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	free(Period);
	free(Prescale);
}
