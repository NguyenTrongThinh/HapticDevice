
#define Data_Pin  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
#define RS_Pin		GPIO_Pin_8
#define RW_Pin		GPIO_Pin_9
#define EN_Pin		GPIO_Pin_10
#define CS1_Pin		GPIO_Pin_13
#define CS2_Pin		GPIO_Pin_12
#define RST_Pin		GPIO_Pin_14

#define Data_Port GPIOA
#define RS_Port		GPIOA
#define RW_Port		GPIOA
#define EN_Port		GPIOA
#define CS1_Port	GPIOB
#define CS2_Port	GPIOB
#define RST_Port	GPIOB

void Glcd_GotoXY(unsigned char x, unsigned char y);
void Glcd_Init(void);
void Glcd_Point(unsigned char x, unsigned char y, unsigned char color);

