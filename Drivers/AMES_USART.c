#include <AMES_USART.h>

typedef struct Data_Frame
{
	char *ID;
	char *Data;
}AMES_Frame;

char *Command;
unsigned char *Index;
unsigned char *EndCmd;

unsigned char TSVN_USART_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = TSVN_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(TSVN_USART_PORT, &GPIO_InitStructure);  
	
  GPIO_InitStructure.GPIO_Pin = TSVN_USART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(TSVN_USART_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = TSVN_USART_BAURATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	USART_Cmd(USART1, ENABLE);
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
	Command = (char*)calloc(255, sizeof(char));
	Index = (unsigned char*)malloc(sizeof(unsigned char));
	EndCmd = (unsigned char*)malloc(sizeof(unsigned char));
	
	if (Command == NULL || Index == NULL || EndCmd == NULL)
		return ERROR;
	else
		return SUCCESS;
}
void TSVN_USART_Deinit(void)
{
	USART_Cmd(USART1, DISABLE);
	free(Command);
	free(Index);
	free(EndCmd);
}
unsigned char TSVN_USART_Create_Frame(char Rx)
{
	switch(Rx)
	{
		case '{': Command[0] = ' '; *Index = 1; *EndCmd = None; break;
		case '}': Command[*Index] = ' '; *EndCmd = End; Command[(*Index)+1] = '\0'; break;
		default: Command[(*Index)++] = Rx; break;
	}
	return *EndCmd;
}
char* TSVN_USART_Get_Frame(void)
{
	if (*EndCmd == End)
		return Command;
	else
		return "";
}
unsigned char TSVN_Find(char chrfind, char *Source, unsigned char No)
{
	unsigned char n = strlen(Source);
	unsigned char i = 0;
	unsigned char pos = 0;
	for (i = 0; i < n; i++)
	{
		if (*Source++ == chrfind)
		{
			pos++;
			if (pos == No)
				return i + 1;
		}
	}
	return 0;
}
char* TSVN_Get_Parameters(unsigned char pvParameters, char* Frame)
{
	static char *ReturnValue;
	unsigned char start = 0;
	unsigned char end, i = 0;
	start = TSVN_Find(' ', Frame, pvParameters);
	end = TSVN_Find(' ', Frame, pvParameters + 1) - 1;
	if (start >= strlen(Frame) || end > strlen(Frame))
		return OUT_OF_MEM;
	Frame += start;
	if (ReturnValue != 0)
		free(ReturnValue);
	ReturnValue = (char*)malloc(sizeof(char*));
	if (ReturnValue == 0)
		return OUT_OF_MEM;
	while (i < (end - start))
	{
		ReturnValue[i++] = *Frame++;
	}
	ReturnValue[i] = '\0';
	return ReturnValue;
}

int fputc(int ch, FILE *f)
{
  USART_SendData(USART1, (uint8_t) ch);

  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}
  return ch;
}
