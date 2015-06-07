#include <AMES_BUTTON.h>

void TSVN_BUT_Init(unsigned char Button_Name)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	switch(Button_Name)
	{
		case S2: GPIO_InitStructure.GPIO_Pin = TSVN_S2_BUT_PIN; 
						 GPIO_Init(TSVN_S2_BUT_PORT, &GPIO_InitStructure);
						 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
						 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
						 EXTI_InitStructure.EXTI_Line = EXTI_Line0;
						 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
						 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
						 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
						 EXTI_Init(&EXTI_InitStructure);
						 NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
						 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
						 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
						 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
						 NVIC_Init(&NVIC_InitStructure);
						 break;
		case S3: GPIO_InitStructure.GPIO_Pin = TSVN_S3_BUT_PIN; 
						 GPIO_Init(TSVN_S3_BUT_PORT, &GPIO_InitStructure);
						 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
						 GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
						 EXTI_InitStructure.EXTI_Line = EXTI_Line13;
						 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
						 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
						 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
						 EXTI_Init(&EXTI_InitStructure);
						 NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
						 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
						 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
						 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
						 NVIC_Init(&NVIC_InitStructure);		
						 break;
	}	
}

