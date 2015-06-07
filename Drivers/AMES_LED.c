#include <AMES_LED.h>

unsigned char *Toggle;
void TSVN_Led_Init(unsigned char LED_NAME)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	Toggle = (unsigned char*)malloc(sizeof(unsigned char));
	if (!Toggle) return;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	switch(LED_NAME)
	{
		case LED_D4: 
									GPIO_InitStructure.GPIO_Pin = TSVN_LED_D4_Pin; 
									break;
		case LED_D5:
									GPIO_InitStructure.GPIO_Pin = TSVN_LED_D5_Pin; 
									break;
		case LED_D6: 
									GPIO_InitStructure.GPIO_Pin = TSVN_LED_D6_Pin; 
									break;
		case LED_D7: 
									GPIO_InitStructure.GPIO_Pin = TSVN_LED_D7_Pin; 
									break;
		case ALL: 
									GPIO_InitStructure.GPIO_Pin = TSVN_LED_D7_Pin | TSVN_LED_D6_Pin | TSVN_LED_D5_Pin | TSVN_LED_D4_Pin; 
									break;
	}
	GPIO_Init(TSVN_LED_PORT, &GPIO_InitStructure);
	
}
void TSVN_Led_On(unsigned char LED_NAME)
{
	switch(LED_NAME)
	{
		case LED_D4: GPIO_SetBits(TSVN_LED_PORT, TSVN_LED_D4_Pin);
								 break;
		case LED_D5: GPIO_SetBits(TSVN_LED_PORT, TSVN_LED_D5_Pin);
								 break;
		case LED_D6: GPIO_SetBits(TSVN_LED_PORT, TSVN_LED_D6_Pin);
								 break;
		case LED_D7: GPIO_SetBits(TSVN_LED_PORT, TSVN_LED_D7_Pin);
								 break;
		case ALL:		 GPIO_SetBits(TSVN_LED_PORT, TSVN_LED_D4_Pin | TSVN_LED_D5_Pin | TSVN_LED_D6_Pin | TSVN_LED_D7_Pin);
								 break;
	}
}
void TSVN_Led_Off(unsigned char LED_NAME)
{
		switch(LED_NAME)
	{
		case LED_D4: GPIO_ResetBits(TSVN_LED_PORT, TSVN_LED_D4_Pin);
								 break;
		case LED_D5: GPIO_ResetBits(TSVN_LED_PORT, TSVN_LED_D5_Pin);
								 break;
		case LED_D6: GPIO_ResetBits(TSVN_LED_PORT, TSVN_LED_D6_Pin);
								 break;
		case LED_D7: GPIO_ResetBits(TSVN_LED_PORT, TSVN_LED_D7_Pin);
								 break;
		case ALL:		 GPIO_ResetBits(TSVN_LED_PORT, TSVN_LED_D4_Pin | TSVN_LED_D5_Pin | TSVN_LED_D6_Pin | TSVN_LED_D7_Pin);
								 break;
	}
}
void TSVN_Led_Toggle(unsigned char LED_NAME)
{
	*Toggle = (unsigned char)((*Toggle + 1)%2);
	if (*Toggle)
		TSVN_Led_On(LED_NAME);
	else
		TSVN_Led_Off(LED_NAME);
}
void TSVN_Led_Deinit(void)
{
	free(Toggle);
}
