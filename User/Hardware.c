#include <Hardware.h>

enum  {MOTOR1, MOTOR2, MOTOR3};

void DIR_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
}
void DIR_Change(unsigned char Channel, unsigned char DIR)
{
	switch (Channel)
	{
		case MOTOR1: {
			if (DIR)
				GPIO_SetBits(GPIOE, GPIO_Pin_8);
			else
				GPIO_ResetBits(GPIOE, GPIO_Pin_8);
			break;
		}
		case MOTOR2: {
			if (DIR)
				GPIO_SetBits(GPIOE, GPIO_Pin_10);
			else
				GPIO_ResetBits(GPIOE, GPIO_Pin_10);
			break;
		}
		case MOTOR3: {
			if (DIR)
				GPIO_SetBits(GPIOE, GPIO_Pin_12);
			else
				GPIO_ResetBits(GPIOE, GPIO_Pin_12);
			break;
		}
	}
}
