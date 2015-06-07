/*----------------------------------------------------------------------------
 * Name:    GLCD.c
 * Purpose: MCBSTM32C low level Graphic LCD (320x240 pixels) functions
 * Version: V1.00
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------
 * History:
 *          V1.00 Initial Version
 *         	V2.00 Add support ILI9341 (kamejoko80@yahoo.com) : 25/04/2010 
 *----------------------------------------------------------------------------*/

#pragma diag_suppress=550

#include <stm32f10x_cl.h>
#include <stdio.h>
#include "GLCD.h"
#include "font.h"

/* SPI_SR - bit definitions. */
#define RXNE    0x01
#define TXE     0x02
#define BSY     0x80
										 
/*********************** Hardware specific configuration **********************/

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    60

/*---------------------- Graphic LCD size definitions ------------------------*/

#define WIDTH       320                 /* Screen Width (in pixels)           */
#define HEIGHT      240                 /* Screen Hight (in pixels)           */
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

#define SPI_START   (0x70)              /* Start byte for SPI transfer        */
#define SPI_RD      (0x01)              /* WR bit 1 within start              */
#define SPI_WR      (0x00)              /* WR bit 0 within start              */
#define SPI_DATA    (0x02)              /* RS bit 1 within start byte         */
#define SPI_INDEX   (0x00)              /* RS bit 0 within start byte         */
 
/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short TextColor = White, BackColor = Black;

void GLCD_putPixel(unsigned int x, unsigned int y);
void GLCD_drawChar(unsigned int x, unsigned int y, unsigned short *c);

/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (unsigned int cnt) {

  unsigned int i,j;  

  for (i=1;i<=cnt; i++)
  	for (j=1;j<10000;j++);
}

/*******************************************************************************
* Write data (16 bit) to LCD controller                                        *
*   Parameter:    c:      data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat (unsigned short c) {

#ifdef ILI9341
    Set_Rs;
    Clr_Cs;
    Set_nRd;
    Clr_nWr;
    GPIOE->ODR = c;
    Set_nWr;
    Set_Cs;
#else
	Set_Rs;
	Set_nRd;
 	GPIOE->ODR = c;
	Clr_nWr;
	Set_nWr;
#endif

}

/*******************************************************************************
* Write command to LCD controller                                              *
*   Parameter:    c:      command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_cmd (unsigned char c) {

#ifdef ILI9341
    Clr_Rs;
    Clr_Cs;
    Set_nRd;
    Clr_nWr;
    GPIOE->ODR = c;
    Set_nWr;
    Set_Cs;
#else
	Clr_Rs;
	Set_nRd;
	GPIOE->ODR = c;
	Clr_nWr;
	Set_nWr;
#endif
}

/*******************************************************************************
* Read data from LCD controller                                                *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/

static __inline unsigned short rd_dat (void) {
  unsigned short val = 0;

#ifdef ILI9341
    GPIOE->CRH = 0x44444444; // Floating input
    GPIOE->CRL = 0x44444444; // Floating input
    Set_Rs;
	Set_nWr;
	Clr_Cs;
	Clr_nRd;
	Set_nRd;
	val = GPIOE->IDR;
    Set_Cs;
	GPIOE->CRH = 0x33333333; // output push-pull 
	GPIOE->CRL = 0x33333333; // output push-pull
#else
    GPIOE->CRH = 0x44444444; // Floating input
	GPIOE->CRL = 0x44444444; // Floating input
	Set_Rs;
	Set_nWr;
	Clr_nRd;
	val = GPIOE->IDR;
	val = GPIOE->IDR;
	GPIOE->CRH = 0x33333333; // output push-pull
	GPIOE->CRL = 0x33333333; // output push-pull
	Set_nRd;
#endif

    return val;  
}

/*******************************************************************************
* Write to LCD register                                                        *
*   Parameter:    reg:    register to be read                                  *
*                 val:    value to write to register                           *
*******************************************************************************/

#ifndef ILI9341
static __inline void wr_reg (unsigned char reg, unsigned short val) {

  	Clr_Cs;
	wr_cmd(reg);      
	wr_dat(val);    
	Set_Cs; 
}
#endif

/*******************************************************************************
* Read from LCD register                                                       *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from register                             *
*******************************************************************************/
#ifndef ILI9341
static unsigned short rd_reg (unsigned short reg) {

  	Clr_Cs;
	wr_cmd(reg);     
	reg = rd_dat();      	
	Set_Cs;
	return reg;
}
#endif

#ifdef ILI9341
/*******************************************************************************
* Read the Graphic LCD module ID code                                          *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/
static void ili9341_read_id (void) {

    unsigned short id0, id1, id2, id3;

	wr_cmd(0xd3);  // Read id_code_command
    id0 = rd_dat();      // read dummy data
	id1 = rd_dat();
	id2 = rd_dat();
	id3 = rd_dat();
	printf ("id0 = %X, id1 = %X, id2 = %X, id3 = %X\n", id0, id1, id2, id3);
}

static void ili9341_set_window (unsigned short col_l,   // 
                                unsigned short col_h,	// col_l <= col_h < 320 --> x
								unsigned short row_l,	// row_l <= row_h < 240	--> y
								unsigned short row_h) {

/*
	0 ----------> X (max 319)
	|
	|
	|
	|
	Y (max 239)	
*/

	wr_cmd(0x2A);              // Collum area set  (collum max = 320)
	
	wr_dat((col_l>>8)& 0xFF);  // Colum high byte (lower collum)
	wr_dat(col_l & 0xFF);      // Colum low byte  (lower collum)

	wr_dat((col_h>>8)& 0xFF);  // Colum high byte (upper collum)
	wr_dat(col_h & 0xFF);      // Colum low byte  (upper collum)
    
		
	wr_cmd(0x2B);              // Row area set	  (row max = 240)

	wr_dat((row_l>>8)& 0xFF);  // Row high byte   (lower row)
	wr_dat(row_l & 0xFF);      // Row low byte    (lower row)

	wr_dat((row_h>>8)& 0xFF);  // Row high byte   (upper row)
	wr_dat(row_h & 0xFF);      // Row low byte    (upper row)
		  
	wr_cmd(0x2c);

}

#if 0
static void ili9341_display_color (unsigned short data)
{
    unsigned int i, j;

    // ili9341_set_window(50, 50 + 16, 50, 50 + 24);

    for(i = 0; i < (WIDTH*HEIGHT); i++)
    {
  	  wr_dat(BackColor);
    }
}
#endif

#endif  

 
/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller                                        *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_init (void) {

#ifndef ILI9341 
  static unsigned short DriverCode;
#endif
													   
// Hardware interface
// nCS         PC6
// RS          PD13
// nWR         PD14
// nRD         PD15
// DATA[15:0]  PE[15:0]

  /* Enable clock for GPIOA,B,C,D,E AFIO and SPI3. */
  RCC->APB2ENR |= 0x0000007D;

  /* PE output set to high. */
  GPIOE->CRL = 0x33333333;	    // output push-pull
  GPIOE->CRH = 0x33333333;		// output push-pull

  GPIOC->CRL &= 0xf0ffffff;	    // PC6 output push-pull 50MHz
  GPIOC->CRL |= 0x03000000;

  GPIOD->CRH &= 0x000fffff;	    //PD13,14,15 output push-pull 50MHz
  GPIOD->CRH |= 0x33300000;

  GPIOC->BSRR = 0x00000040;		//cs = 1;
  GPIOD->BSRR = 0x0000E000;		//rs,wr,rd = 1;

  delay(10);                    /* Delay 50 ms                        */


#ifdef ILI9341
   
    ili9341_read_id();    

	wr_cmd(0x11);
	delay(20);
	wr_cmd(0xD0);
	wr_dat(0x07);
	wr_dat(0x41);
	wr_dat(0x1F);

	wr_cmd(0xD1);
	wr_dat(0x00);
	wr_dat(0x2C);
	wr_dat(0x1A);

	wr_cmd(0xD2);
	wr_dat(0x01);
	wr_dat(0x11);

	wr_cmd(0xC0);
	wr_dat(0x00);
	wr_dat(0x3B);
	wr_dat(0x00);
	wr_dat(0x02);
	wr_dat(0x11);

	wr_cmd(0xC5);
	wr_dat(0x02);

	wr_cmd(0xC8);
	wr_dat(0x07);
	wr_dat(0x07);
	wr_dat(0x07);
	wr_dat(0x50);
	wr_dat(0x08);
	wr_dat(0x00);
	wr_dat(0x07);
	wr_dat(0x07);
	wr_dat(0x07);
	wr_dat(0x05);
	wr_dat(0x02);
	wr_dat(0x06);

	wr_cmd(0x36);  // MADCTRL (Memory Access Control)
	wr_dat(0xE8);	//0x4B, 0x48

	wr_cmd(0x3A);
	wr_dat(0x55);

	wr_cmd(0xF0); 
	wr_dat(0x01); 

	wr_cmd(0xF3); 
	wr_dat(0x40); 
	wr_dat(0x0A); 

	wr_cmd(0x2A);
	wr_dat(0x00);
	wr_dat(0x00);
	wr_dat(0x01);
	wr_dat(0x3F);

	wr_cmd(0x2B);
	wr_dat(0x00);
	wr_dat(0x00);
	wr_dat(0x01);
	wr_dat(0xDF);
	delay(120);
	wr_cmd(0x29);
	wr_cmd(0x2C);
	delay(5);

#else /* not ILI9341*/

  DriverCode = rd_reg(0x00);

  printf ("GLCD driver code=%x\n", DriverCode); 

  if(DriverCode == 0x9320)
  {
	  /* Start Initial Sequence --------------------------------------------------*/
	  wr_reg(0xE5, 0x8000);                 /* Set the internal vcore voltage     */
	  wr_reg(0x00, 0x0001);                 /* Start internal OSC                 */
	  wr_reg(0x01, 0x0100);                 /* Set SS and SM bit                  */
	  wr_reg(0x02, 0x0700);                 /* Set 1 line inversion               */
	  wr_reg(0x03, 0x1030);                 /* Set GRAM write direction and BGR=1 */
	  wr_reg(0x04, 0x0000);                 /* Resize register                    */
	  wr_reg(0x08, 0x0202);                 /* 2 lines each, back and front porch */
	  wr_reg(0x09, 0x0000);                 /* Set non-disp area refresh cyc ISC  */
	  wr_reg(0x0A, 0x0000);                 /* FMARK function                     */
	  wr_reg(0x0C, 0x0000);                 /* RGB interface setting              */
	  wr_reg(0x0D, 0x0000);                 /* Frame marker Position              */
	  wr_reg(0x0F, 0x0000);                 /* RGB interface polarity             */
	
	  /* Power On sequence -------------------------------------------------------*/
	  wr_reg(0x10, 0x0000);                 /* Reset Power Control 1              */
	  wr_reg(0x11, 0x0000);                 /* Reset Power Control 2              */
	  wr_reg(0x12, 0x0000);                 /* Reset Power Control 3              */
	  wr_reg(0x13, 0x0000);                 /* Reset Power Control 4              */
	  delay(20);                            /* Discharge cap power voltage (200ms)*/
	  wr_reg(0x10, 0x17B0);                 /* SAP, BT[3:0], AP, DSTB, SLP, STB   */
	  wr_reg(0x11, 0x0137);                 /* DC1[2:0], DC0[2:0], VC[2:0]        */
	  delay(5);                             /* Delay 50 ms                        */
	  wr_reg(0x12, 0x0139);                 /* VREG1OUT voltage                   */
	  delay(5);                             /* Delay 50 ms                        */
	  wr_reg(0x13, 0x1D00);                 /* VDV[4:0] for VCOM amplitude        */
	  wr_reg(0x29, 0x0013);                 /* VCM[4:0] for VCOMH                 */
	  delay(5);                             /* Delay 50 ms                        */
	  wr_reg(0x20, 0x0000);                 /* GRAM horizontal Address            */
	  wr_reg(0x21, 0x0000);                 /* GRAM Vertical Address              */
	
	  /* Adjust the Gamma Curve --------------------------------------------------*/
	  wr_reg(0x30, 0x0006);
	  wr_reg(0x31, 0x0101);
	  wr_reg(0x32, 0x0003);
	  wr_reg(0x35, 0x0106);
	  wr_reg(0x36, 0x0B02);
	  wr_reg(0x37, 0x0302);
	  wr_reg(0x38, 0x0707);
	  wr_reg(0x39, 0x0007);
	  wr_reg(0x3C, 0x0600);
	  wr_reg(0x3D, 0x020B);
	  
	  /* Set GRAM area -----------------------------------------------------------*/
	  wr_reg(0x50, 0x0000);                 /* Horizontal GRAM Start Address      */
	  wr_reg(0x51, (HEIGHT-1));             /* Horizontal GRAM End   Address      */
	  wr_reg(0x52, 0x0000);                 /* Vertical   GRAM Start Address      */
	  wr_reg(0x53, (WIDTH-1));              /* Vertical   GRAM End   Address      */
	  wr_reg(0x60, 0x2700);                 /* Gate Scan Line                     */
	  wr_reg(0x61, 0x0001);                 /* NDL,VLE, REV                       */
	  wr_reg(0x6A, 0x0000);                 /* Set scrolling line                 */
	
	  /* Partial Display Control -------------------------------------------------*/
	  wr_reg(0x80, 0x0000);
	  wr_reg(0x81, 0x0000);
	  wr_reg(0x82, 0x0000);
	  wr_reg(0x83, 0x0000);
	  wr_reg(0x84, 0x0000);
	  wr_reg(0x85, 0x0000);
	
	  /* Panel Control -----------------------------------------------------------*/
	  wr_reg(0x90, 0x0010);
	  wr_reg(0x92, 0x0000);
	  wr_reg(0x93, 0x0003);
	  wr_reg(0x95, 0x0110);
	  wr_reg(0x97, 0x0000);
	  wr_reg(0x98, 0x0000);
	
	  /* Set GRAM write direction and BGR = 1
	     I/D=10 (Horizontal : increment, Vertical : increment)
	     AM=1 (address is updated in vertical writing direction)                  */
	  wr_reg(0x03, 0x1038);
	
	  wr_reg(0x07, 0x0173);                 /* 262K color and display ON          */
  	}
	else if(DriverCode == 0x7783)
	{
	  // Start Initial Sequence
	  	wr_reg(0x00FF,0x0001);
	  	wr_reg(0x00F3,0x0008);
		wr_reg(0x0001,0x0100);
		wr_reg(0x0002,0x0700);
		wr_reg(0x0003,0x1030);  //0x1030
		wr_reg(0x0008,0x0302);
		wr_reg(0x0008,0x0207);
		// Power On sequence 
		wr_reg(0x0009,0x0000);
		wr_reg(0x000A,0x0000);
		wr_reg(0x0010,0x0000);  //0x0790
		wr_reg(0x0011,0x0005);
		wr_reg(0x0012,0x0000);
		wr_reg(0x0013,0x0000);
		delay(100);
		wr_reg(0x0010,0x12B0);
		delay(100);;
		wr_reg(0x0011,0x0007);
		delay(100);
		wr_reg(0x0012,0x008B);
		delay(100);
		wr_reg(0x0013,0x1700);
		delay(100);
		wr_reg(0x0029,0x0022);
		
		// void Gamma_Set(void) 
		wr_reg(0x0030,0x0000);
		wr_reg(0x0031,0x0707);
		wr_reg(0x0032,0x0505);
		wr_reg(0x0035,0x0107);
		wr_reg(0x0036,0x0008);
		wr_reg(0x0037,0x0000);
		wr_reg(0x0038,0x0202);
		wr_reg(0x0039,0x0106);
		wr_reg(0x003C,0x0202);
		wr_reg(0x003D,0x0408);
		delay(100);
		
		// Set GRAM area
		wr_reg(0x0050,0x0000);		
		wr_reg(0x0051,0x00EF);		
		wr_reg(0x0052,0x0000);		
		wr_reg(0x0053,0x013F);		
		wr_reg(0x0060,0xA700);		
		wr_reg(0x0061,0x0001);
		wr_reg(0x0090,0x0033);				
		wr_reg(0x002B,0x000B);
		
		/* Set GRAM write direction and BGR = 1
	     I/D=10 (Horizontal : increment, Vertical : increment)
	     AM=1 (address is updated in vertical writing direction)                  */
	    wr_reg(0x03, 0x1038);		
		wr_reg(0x0007,0x0133);
	}
	else
	{
		if (DriverCode == 0x9325)
		{
		  printf ("Init ILI9325 LCD module\n");

        wr_reg(0x00e7,0x0010);      
        wr_reg(0x0000,0x0001);//??????
        wr_reg(0x0001,0x0100);     
        wr_reg(0x0002,0x0700);//????                    


		#define ID_AM 011

		#if   ID_AM==000       
	    	wr_reg(0x0003,0x1000); // TFM=0,TRI=0,SWAP=1,16 bits system interface  swap RGB to BRG,ORG HWM 0
		#elif ID_AM==001        
	     	wr_reg(0x0003,0x1008);      
		#elif ID_AM==010  
	     	wr_reg(0x0003,0x1010);        
		#elif ID_AM==011
	     	wr_reg(0x0003,0x1018);
		#elif ID_AM==100  
	     	wr_reg(0x0003,0x1020);      
		#elif ID_AM==101  
	     	wr_reg(0x0003,0x1028);      
		#elif ID_AM==110  
	     	wr_reg(0x0003,0x1030);      
		#elif ID_AM==111  
	     	wr_reg(0x0003,0x1038);
		#endif    
		   
        wr_reg(0x0004,0x0000);                                   
        wr_reg(0x0008,0x0207);            
        wr_reg(0x0009,0x0000);         
        wr_reg(0x000a,0x0000);//display setting         
        wr_reg(0x000c,0x0001);//display setting          
        wr_reg(0x000d,0x0000);//0f3c          
        wr_reg(0x000f,0x0000);

        wr_reg(0x0010,0x0000);   
        wr_reg(0x0011,0x0007);
        wr_reg(0x0012,0x0000);                                                                 
        wr_reg(0x0013,0x0000);                 
        delay(5); 
        wr_reg(0x0010,0x1590);   
        wr_reg(0x0011,0x0227);
        delay(5); 
        wr_reg(0x0012,0x009c);                  
        delay(5); 
        wr_reg(0x0013,0x1900);   
        wr_reg(0x0029,0x0023);
        wr_reg(0x002b,0x000e);
        delay(5); 
        wr_reg(0x0020,0x0000);                                                            
        wr_reg(0x0021,0x013f);           
        delay(5); 

        wr_reg(0x0030,0x0007); 
        wr_reg(0x0031,0x0707);   
        wr_reg(0x0032,0x0006);
        wr_reg(0x0035,0x0704);
        wr_reg(0x0036,0x1f04); 
        wr_reg(0x0037,0x0004);
        wr_reg(0x0038,0x0000);        
        wr_reg(0x0039,0x0706);     
        wr_reg(0x003c,0x0701);
        wr_reg(0x003d,0x000f);
        delay(5); 
        wr_reg(0x0050,0x0000); 
        wr_reg(0x0051,0x00ef);                   
        wr_reg(0x0052,0x0000);                    
        wr_reg(0x0053,0x013f);
        
        wr_reg(0x0060,0xa700);        
        wr_reg(0x0061,0x0001); 
        wr_reg(0x006a,0x0000);
        wr_reg(0x0080,0x0000);
        wr_reg(0x0081,0x0000);
        wr_reg(0x0082,0x0000);
        wr_reg(0x0083,0x0000);
        wr_reg(0x0084,0x0000);
        wr_reg(0x0085,0x0000);
      
        wr_reg(0x0090,0x0010);     
        wr_reg(0x0092,0x0000);  
        wr_reg(0x0093,0x0003);
        wr_reg(0x0095,0x0110);
        wr_reg(0x0097,0x0000);        
        wr_reg(0x0098,0x0000);  
   
        wr_reg(0x0007,0x0133);   
        wr_reg(0x0020,0x0000);                                                            
        wr_reg(0x0021,0x013f);
		  
		}
	}

#endif  /* ILI9341 */

}


void GLCD_SetCursor(unsigned short x, unsigned short y)
{

#ifdef ILI9341

    ili9341_set_window (x, x, y, y);

#else

	wr_reg(0x20,x); 
    wr_reg(0x21,y);

#endif
}

void GLCD_putPixel(unsigned int x, unsigned int y) {

#ifdef ILI9341
    GLCD_SetCursor(x, y);
	wr_dat(TextColor);
#else
    wr_reg(0x20, x);
    wr_reg(0x21, y);
    wr_reg(0x22,Red);
#endif
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:    color:  color for clearing display                           *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_setTextColor(unsigned short color) {
  TextColor = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:    color:  color for clearing display                           *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_setBackColor(unsigned short color) {
  BackColor = color;
}


/*******************************************************************************
* Clear display                                                                *
*   Parameter:    color:  color for clearing display                           *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_clear (unsigned short color) {
  unsigned int   i;

#ifdef ILI9341

    ili9341_set_window(0, WIDTH, 0, HEIGHT);

    for(i = 0; i < (WIDTH*HEIGHT); i++)
    {
  	  wr_dat(color);
    }

#else
  wr_reg(0x20, 0);
  wr_reg(0x21, 0);
  Clr_Cs;
  wr_cmd(0x22);
  for(i = 0; i < (WIDTH*HEIGHT); i++)
  {
  	wr_dat(color);
  }
  Set_Cs;
#endif    
}


/*******************************************************************************
* Draw character on given position (line, coloum                               *
*   Parameter:     x :        horizontal position                              *
*                  y :        vertical position                                *
*                  c*:        pointer to color value                           *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_drawChar(unsigned int x, unsigned int y, unsigned short *c) {

#ifdef ILI9341

  unsigned int index = 0;
  int  i = 0;

  ili9341_set_window (y, y + 15, x, x + 23);  

  for(index = 0; index < 24; index++)
  {
    for(i = 0; i < 16; i++)
    {
      if((c[index] & (1 << i)) == 0x00) {
         wr_dat(BackColor);
      } else {
         wr_dat(TextColor);
      }
    }
  } 	

#else /* ILI9341 */

  unsigned int index = 0;
  int  i = 0;
  unsigned int Xaddress = 0;   

  Xaddress = x;

  wr_reg(0x21, y);
  wr_reg(0x20, Xaddress);


  for(index = 0; index < 24; index++)
  {
  	Clr_Cs;
    wr_cmd(0x22);              /* Prepare to write GRAM */

    for(i = 0; i <= 15; i++)
    {
      if((c[index] & (1 << i)) == 0x00) {
         wr_dat(BackColor);
      } else {
         wr_dat(TextColor);
      }
    }

	Set_Cs;

    Xaddress++;

	wr_reg(0x21, y);
    wr_reg(0x20, Xaddress);
  }

#endif /* ILI9341 */

}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:     c :        ascii character                                  *
*                  ln:        line number                                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_displayChar(unsigned int ln, unsigned int col, unsigned char  c) {
  c -= 32;
  GLCD_drawChar(ln, col, &ASCII_Table[c * 24]);
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:     s*:        pointer to string                                *
*                  ln:        line number                                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_displayStringLn(unsigned int ln, unsigned char *s) {
  unsigned int i = 0;

#ifdef ILI9341
  unsigned int refcolumn = 0;
#else
  unsigned int refcolumn = (WIDTH/*-1*/)-16;
#endif

  while ((*s != 0) & (i < 20))                   /* write the string character by character on lCD */
  {
    GLCD_displayChar(ln, refcolumn, *s);         /* Display one character on LCD */

#ifdef ILI9341
	 refcolumn += 16; 
#else
    refcolumn -= 16;                             /* Decrement the column position by 16 */
#endif

    s++;                                         /* Point on the next character */
    i++;                                         /* Increment the character counter */
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:     ln:        line number                                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_clearLn(unsigned int ln) {
  GLCD_displayStringLn(ln, "                    ");
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, const unsigned short *bitmap) {
  unsigned int   i;
  unsigned int   len = w*h;
  const unsigned short *bitmap_ptr = (const unsigned short *)bitmap;

#ifdef ILI9341
  ili9341_set_window (x, x + w-1, y, y + h - 1);
  for (i = 0; i < len; i++) {
    wr_dat(*bitmap_ptr++);
  }
#else
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+h-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  wr_reg(0x20, y);
  wr_reg(0x21, x);

  Clr_Cs;
  wr_cmd(0x22);
  for (i = 0; i < len; i++) {
    wr_dat(*bitmap_ptr++);
  }
  Set_Cs;
#endif
}
/******************************************************************************/
