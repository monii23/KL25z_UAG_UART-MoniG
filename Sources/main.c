/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "derivative.h" /* include peripheral declarations */
#include <stdint.h>
#include "UART.h"

#define MCU_BUS_CLK	(21000000)
#define UART0_OSR	(16)
#define UART_BAUDRATE_TARGET	(9600)
#define CONSOLE_BAUDRATE	(MCU_BUS_CLK/(UART_BAUDRATE_TARGET*UART0_OSR))
#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(x) (((1)<<(x & GPIO_PIN_MASK)))
#define WAIT 0x780
//#define WAIT 0x0FFF
#define WAITI 0xAFC80

//#define WAITI 0xEA600
#define LCD_INS		0x00
#define LCD_ON		0x0F
#define LCD_DATA	0x01 
#define LCD_CLEAR	0x01
#define LCD_PRIMERRENGLON 0x80
#define LCD_SEGUNDORENGLON 0xC0
#define LCD_8B 0x38
#define TRIGGER_CHARACTER	('y')


#define GPIOD_DATA_MASK (0xFF0)
#define GPIOB_EN_MASK (1<<2)
#define GPIOB_RS_MASK (1)
#define GPIOB_RW_MASK (1<<1)
#define GPIOB_MASK 7
#define CLEAR             (0x01)
#define TURN_ON           (0X0C)
#define LINE_FEED         (0xC0)
#define SECOND_LINE_EN    (0x38)
#define INCREASE_CURSOR   (0x06)
#define DECREASE_CURSOR   (0x04)
#define BACKSPACE         (0x10)
#define DDRAM_ADDRESS_0   (0x80)
#define DDRAM_ADDRESS_1   (0x81)
#define DDRAM_ADDRESS_15  (0x8F)
#define DDRAM_ADDRESS_64  (0xC0)
#define DDRAM_ADDRESS_67  (0xC3)
#define DDRAM_ADDRESS_71  (0xC7)
#define DDRAM_ADDRESS_74  (0xCA)
#define DDRAM_ADDRESS_79  (0xCF)
#define CGRAM_ADDRESS_0   (0x40)
#define SPACE             (0x20)
#define UP                (0x80)
#define DOWN              (0xC0)
#define HOME              (0x02)
#define CURSOR_ON 		  (0xD)
#define CURSOR_OFF        (0xC)
#define INS (1)
#define CHAR (0)
#define WAIT_TIME (0xFFF)
uint8_t  gbCharacterEcho;
uint8_t  CONTLCD;
uint8_t  gbCharacterTrigger;

void vfnLCD_Init(void);
void ufnDelay(short wcounter);
void ufnPantalla(unsigned int uType, unsigned int uInstruction);
void vfnSet_RS(void);
void vfnClear_RS(void);
void vfnEnable(void);
void vfnLCD_Write(char bCommand, char bData);


int main(void)
{
	vfnLCD_Init();
/*UART
 
Initializa the UART0 */
	UART_vfnInit(UART0,CONSOLE_BAUDRATE,UART0_OSR,UART0_CLK_FLL_PLL);
	/* Set the driver to receive data */
	UART_vfnRxBuffer(UART0,&gbCharacterEcho,1);
/*UART*/
	
	CONTLCD=0;
	for(;;) 
	{	
		
		if (CONTLCD==16)
			vfnLCD_Write(INS,LCD_SEGUNDORENGLON);
		if (CONTLCD==32)
		{
			vfnLCD_Write(INS,LCD_CLEAR);
			CONTLCD=0;
		}
		if(!UART_CHECK_STATUS(UART_RX_PROGRESS))
		{
			//if(gbCharacterEcho == TRIGGER_CHARACTER)
					//				{
							
			/* send back the received data and wait for a new character */
			UART_vfnTxBuffer(UART0,&gbCharacterEcho,1);
			UART_vfnRxBuffer(UART0,&gbCharacterEcho,1);
			vfnLCD_Write(CHAR,gbCharacterEcho);
			CONTLCD++;
				//					}
			
			
			//UART_vfnRxBuffer(UART0,&gbCharacterEcho,1);
		}
	}
	
	return 0;

}



void ufnDelay (short  wtime)
{
	while(wtime >0)
	{ 	
		wtime = wtime - 1; 
	}
}
void vfnSet_RS(void)
{
	GPIOB_PSOR=GPIOB_RS_MASK; //Set RS pin 
}

/*****************************************************************/

void vfnClear_RS(void)
{
	GPIOB_PCOR=GPIOB_RS_MASK; //Clear RS pin
}

/*****************************************************************/

void vfnEnable(void)
{
	short hwDelay=WAIT_TIME;
	
	GPIOB_PSOR=GPIOB_EN_MASK;  //set enable
	while(hwDelay--);  //wait time
	GPIOB_PCOR=GPIOB_EN_MASK; //clear enable
}

/*****************************************************************/

void vfnLCD_Write(char bINS, char bData)
{
	if(bINS) //if INS
	{
		vfnClear_RS();  //turn off RS to write INS
		GPIOC_PDOR=bData<<4;//load data in output
		vfnEnable();    
		vfnSet_RS(); //turn on RS to write char
	}
	else //if char
	{
		GPIOC_PDOR=bData<<4; //load data in output
		vfnEnable();
	}
}
void vfnLCD_Init(void)
{
	// turn on clocks
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
	
	//LCD pins
	PORTB_PCR0= PORT_PCR_MUX(1);
	PORTB_PCR1= PORT_PCR_MUX(1);
	PORTB_PCR2= PORT_PCR_MUX(1);
	PORTC_PCR4= PORT_PCR_MUX(1);
	PORTC_PCR5= PORT_PCR_MUX(1);
	PORTC_PCR6= PORT_PCR_MUX(1);      //configure as GPIO
	PORTC_PCR7= PORT_PCR_MUX(1);
	PORTC_PCR8= PORT_PCR_MUX(1);
	PORTC_PCR9= PORT_PCR_MUX(1);
	PORTC_PCR10= PORT_PCR_MUX(1);
	PORTC_PCR11= PORT_PCR_MUX(1);
	
	GPIOB_PDDR=GPIOB_EN_MASK | GPIOB_RS_MASK | GPIOB_RW_MASK; //set LCD enable,RS and RW as outputs
	GPIOC_PDDR=GPIOD_DATA_MASK; //set LCD data pins as outputs
	
	vfnLCD_Write(INS,CLEAR); //clear screen
	vfnLCD_Write(INS,TURN_ON);//turn on screen
	vfnLCD_Write(INS,SECOND_LINE_EN);//enable second line
}


