#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <AMES_PWM.h>
#include <AMES_QEI.h>
#include <AMES_PID.h>


void DC_Init(void);
void DC_Change_Dir(unsigned char Dir);
void DC_Change_PWM(unsigned int PWM_Value);