/****************************************************************************/
/*EEPROM library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 23/02/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/

#include "stm32f10x.h" 
#include <stm32f10x_gpio.h>
#include <stm32f10x_i2c.h>

#define EEPROM_ADDRESS 0x50
#define I2C_SPEED 100000
#define Time_out 10000					//ms

#define I2C_SCL GPIO_Pin_6
#define I2C_SDA GPIO_Pin_7
#define I2C_PORT GPIOB

void TSVN_EEPROM_Init(void);
void TSVN_EEPROM_Write_Byte(unsigned char Address, unsigned char Byte);
unsigned char TSVN_EEPROM_Read_Byte(unsigned char Address);
