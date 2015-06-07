#include <AMES_QEI.h>

unsigned int *TIM1_MAXCNT;
long *TIM1_Over_Load;

unsigned int *TIM2_MAXCNT;
long *TIM2_Over_Load;

unsigned int *TIM3_MAXCNT;
long *TIM3_Over_Load;

unsigned int *TIM4_MAXCNT;
long *TIM4_Over_Load;

/**********************************************************************/
//QEI Timer 1
void TSVN_QEI_TIM1_Init(unsigned int MAX_CNT)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM1_MAXCNT = (unsigned int*)malloc(sizeof(unsigned int));
	TIM1_Over_Load = (long*)malloc(sizeof(long));
	*TIM1_Over_Load = 0;
	*TIM1_MAXCNT = MAX_CNT;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = MAX_CNT;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
 	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM1, ENABLE);
}
void TSVN_QEI_TIM1_Deinit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
	free(TIM1_MAXCNT);
	free(TIM1_Over_Load);
}
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		if((TIM1->CR1)&0x0010)
			(*TIM1_Over_Load)--;
		else
			(*TIM1_Over_Load)++;
	}
}
long TSVN_QEI_TIM1_Value(void)
{
	return (*TIM1_Over_Load)*(*TIM1_MAXCNT) + (long)TIM1->CNT;
}
/*********************************************************************/
//QEI Timer 2
void TSVN_QEI_TIM2_Init(unsigned int MAX_CNT)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM2_MAXCNT = (unsigned int*)malloc(sizeof(unsigned int));
	TIM2_Over_Load = (long*)malloc(sizeof(long));
	*TIM2_Over_Load = 0;
	*TIM2_MAXCNT = MAX_CNT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = MAX_CNT;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM2, ENABLE);
}
void TSVN_QEI_TIM2_Deinit(void)
{
	free(TIM2_MAXCNT);
  free(TIM2_Over_Load);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		if((TIM2->CR1)&0x0010)
			(*TIM2_Over_Load)--;
		else
			(*TIM2_Over_Load)++;
	}
}

long TSVN_QEI_TIM2_Value(void)
{
	return (*TIM2_Over_Load)*(*TIM2_MAXCNT) + (long)TIM2->CNT;
}
/************************************************************************/
//QEI Timer 3
void TSVN_QEI_TIM3_Init(unsigned int MAX_CNT)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM3_MAXCNT = (unsigned int*)malloc(sizeof(unsigned int));
	TIM3_Over_Load = (long*)malloc(sizeof(long));
	*TIM3_Over_Load = 0;
	*TIM3_MAXCNT = MAX_CNT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = MAX_CNT;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM3, ENABLE);
	
}
void TSVN_QEI_TIM3_Deinit(void)
{
	free(TIM3_MAXCNT);
  free(TIM3_Over_Load);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		if((TIM3->CR1)&0x0010)
			(*TIM3_Over_Load)--;
		else
			(*TIM3_Over_Load)++;
	}
}
long TSVN_QEI_TIM3_Value(void)
{
		return (*TIM3_Over_Load)*(*TIM3_MAXCNT) + (long)TIM3->CNT;
}
/**********************************************************************/
//QEI Timer 4
void TSVN_QEI_TIM4_Init(unsigned int MAX_CNT)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM4_MAXCNT = (unsigned int*)malloc(sizeof(unsigned int));
	TIM4_Over_Load = (long*)malloc(sizeof(long));
	*TIM4_Over_Load = 0;
	*TIM4_MAXCNT = MAX_CNT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = MAX_CNT;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM4, ENABLE);
	
}
void TSVN_QEI_TIM4_Deinit(void)
{
	free(TIM4_MAXCNT);
  free(TIM4_Over_Load);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
}
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		if((TIM4->CR1>>4)&0x0001)
			(*TIM4_Over_Load)--;
		else
			(*TIM4_Over_Load)++;
	}
}

long TSVN_QEI_TIM4_Value(void)
{
	return (*TIM4_Over_Load)*(*TIM4_MAXCNT) + (long)TIM4->CNT;
}

