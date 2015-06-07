#include <stm32f10x_glcd.h>
#include <stm32f10x.h>
void Glcd_Delay(unsigned char cir)
{
	while(72000*cir--);
}
void Enable_Pulse(void)
{
	GPIO_SetBits(EN_Port, EN_Pin);
	Glcd_Delay(50);
	GPIO_ResetBits(EN_Port, EN_Pin);
	Glcd_Delay(50);
}
void Glcd_On(void)
{
	GPIO_Write(Data_Port, 0x3F);
	GPIO_ResetBits(CS1_Port, CS1_Pin);
	GPIO_ResetBits(CS2_Port, CS2_Pin);
	GPIO_ResetBits(RS_Port, RS_Pin);
	GPIO_ResetBits(RW_Port, RW_Pin);
	Enable_Pulse();
}
void Set_Start_Line(unsigned char line)
{
	GPIO_ResetBits(RS_Port, RS_Pin);
	GPIO_ResetBits(RW_Port, RW_Pin);
	GPIO_ResetBits(CS1_Port, CS1_Pin);
	GPIO_ResetBits(CS2_Port, CS2_Pin);
	GPIO_Write(Data_Port, 0xC0 | line);
	Enable_Pulse();
}
void Glcd_Goto_Col(unsigned char col)
{
	unsigned short Col_Data;
	GPIO_ResetBits(RS_Port, RS_Pin);
	GPIO_ResetBits(RW_Port, RW_Pin);
	if (col < 64)
	{
		GPIO_ResetBits(CS1_Port, CS1_Pin);
		GPIO_SetBits(CS2_Port, CS2_Pin);
		Col_Data = col;
	}
	else
	{
		GPIO_SetBits(CS1_Port, CS1_Pin);
		GPIO_ResetBits(CS2_Port, CS2_Pin);
		Col_Data = col - 64;
	}
	Col_Data = (Col_Data | 0x40) & 0x7F;
	GPIO_Write(Data_Port, Col_Data);
	Enable_Pulse();
}
void Glcd_Goto_Row(unsigned char Row)
{
	unsigned short Col_Data;
	GPIO_ResetBits(RS_Port, RS_Pin);
	GPIO_ResetBits(RW_Port, RW_Pin);
	Col_Data = (Row | 0xB8)&0xBF;
	GPIO_Write(Data_Port, Col_Data);
	Enable_Pulse();
}
void Glcd_GotoXY(unsigned char x, unsigned char y)
{
	Glcd_Goto_Col(x);
	Glcd_Goto_Row(y);
}
void Glcd_Write(unsigned short Data)
{
	GPIO_SetBits(RS_Port, RS_Pin);
	GPIO_ResetBits(RW_Port, RW_Pin);
	GPIO_Write(Data_Port, Data);
	Glcd_Delay(2);
	Enable_Pulse();
}
unsigned short Glcd_Read(unsigned short Col)
{
	unsigned short Read_Data = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = Data_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(Data_Port, &GPIO_InitStructure);
	GPIO_SetBits(RW_Port, RW_Pin);
	GPIO_SetBits(RS_Port, RS_Pin);
	if (Col>63)
	{
		GPIO_SetBits(CS1_Port, CS1_Pin);
		GPIO_ResetBits(CS2_Port, CS2_Pin);
	}
	else
	{
		GPIO_ResetBits(CS1_Port, CS1_Pin);
		GPIO_SetBits(CS2_Port, CS2_Pin);
	}
	Glcd_Delay(1);
	GPIO_SetBits(EN_Port, EN_Pin);
	Glcd_Delay(1);
	GPIO_ResetBits(EN_Port, EN_Pin);
	Glcd_Delay(5);
	GPIO_SetBits(EN_Port, EN_Pin);
	Glcd_Delay(1);
	Read_Data = GPIO_ReadInputData(Data_Port);
	GPIO_ResetBits(EN_Port, EN_Pin);
	Glcd_Delay(1);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(Data_Port, &GPIO_InitStructure);
	return Read_Data;
}
void Glcd_Clear_Line(unsigned short Line)
{
	unsigned char i = 0;
	Glcd_GotoXY(0, Line);
	Glcd_GotoXY(64, Line);
	GPIO_ResetBits(CS1_Port, CS1_Pin);
	for(i = 0; i< 65; i++)
		Glcd_Write(0);
}
void Glcd_Clear_Screen(void)
{
	unsigned char  i = 0;
	for( i = 0; i<8; i++)
			Glcd_Clear_Line(i);
}
void Glcd_Point(unsigned char x, unsigned char y, unsigned char color)
{
	unsigned short Col_Data;
	Glcd_GotoXY(x, (y/8));
	switch(color)
	{
		case 0: Col_Data = ~(1<<(y%8))&Glcd_Read(x); break;
		case 1: Col_Data = (1<<(y%8))&Glcd_Read(x); break;
	}
  Glcd_GotoXY(x, (y/8));
	Glcd_Write(Col_Data);
}
void Glcd_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = Data_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(Data_Port, &GPIO_InitStructure);
	GPIO_Write(Data_Port, 0xFF);
	GPIO_InitStructure.GPIO_Pin = RS_Pin;
	GPIO_Init(RS_Port, &GPIO_InitStructure);
	GPIO_SetBits(RS_Port, RS_Pin);
	
	GPIO_InitStructure.GPIO_Pin = RW_Pin;
	GPIO_Init(RW_Port, &GPIO_InitStructure);
	GPIO_SetBits(RW_Port, RW_Pin);
	
	GPIO_InitStructure.GPIO_Pin = EN_Pin;
	GPIO_Init(EN_Port, &GPIO_InitStructure);
	GPIO_SetBits(EN_Port, EN_Pin);
	
	GPIO_InitStructure.GPIO_Pin = CS1_Pin;
	GPIO_Init(CS1_Port, &GPIO_InitStructure);
	GPIO_SetBits(CS1_Port, CS1_Pin);
	
	GPIO_InitStructure.GPIO_Pin = CS2_Pin;
	GPIO_Init(CS2_Port, &GPIO_InitStructure);
	GPIO_SetBits(CS2_Port, CS2_Pin);
	
	GPIO_InitStructure.GPIO_Pin = RST_Pin;
	GPIO_Init(RST_Port, &GPIO_InitStructure);
	GPIO_SetBits(RST_Port, RST_Pin);
	
	Glcd_On();
	Glcd_Clear_Screen();
	Set_Start_Line(0);
}
