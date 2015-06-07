#include <DC_Servo.h>

void DC_Init(void)
{
	unsigned int PWM_MAX_Value; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
	PWM_MAX_Value = TSVN_PWM_TIM5_Init(5000);
	TSVN_PWM_TIM5_Set_Duty(0, PWM_CHANNEL_2);
	TSVN_PWM_TIM5_Start();
	TSVN_QEI_TIM4_Init(960);
	PID_Mem_Create(1);
	PID_WindUp_Init(1, PWM_MAX_Value);
	PID_Init(1, 0, 0, 0);
}
void DC_Change_Dir(unsigned char Dir)
{
	if (Dir)
		GPIO_SetBits(GPIOE, GPIO_Pin_0);
	else
		GPIO_ResetBits(GPIOE, GPIO_Pin_0);
}
void DC_Change_PWM(unsigned int PWM_Value)
{
	TSVN_PWM_TIM5_Set_Duty(PWM_Value, PWM_CHANNEL_2);
}