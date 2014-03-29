 /*********************************************************************************************//*!
 *
 * @file UART.c
 *
 * @author Carlos Neri
 *
 * @date 4/02/2012
 *
 * @brief Brief description of the file
 *************************************************************************************************/
/*************************************************************************************************/
/*                                      Includes Section                                         */
/*************************************************************************************************/
#include <derivative.h>
#include "ProjectTypes.h"
#include "NVIC.h"
#include "GPIO.h"
#include "UART.h"
/*************************************************************************************************/
/*                                  Function Prototypes Section                                  */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                   Defines & Macros Section                                    */
/*************************************************************************************************/
#define UART_BDR_DIVIDER	(16)
#define UART_BAUD_RATE_REGISTERS	(2)
#define UART_BAUD_RATE_LOW_REGISTER_OFFSET	(0)
#define UART_BAUD_RATE_HIGH_REGISTER_OFFSET	(1)
#define UART_REGISTERS	(9)

enum eUART_Registers
{
	UART_BDH = 0,
	UART_BDL,
	UART_C1,
	UART_C2,
	UART_S1,
	UART_S2,
	UART_C3,
	UART_D,
	UART_C4
};
/*************************************************************************************************/
/*                                       Typedef Section                                         */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                   Global Variables Section                                    */
/*************************************************************************************************/
uint32_t UART_dwDriverStatus = 0;

static uint8_t * UART_gpbOutputBuffer;
static uint16_t  UART_gwDataOutCounter = 0;

static uint8_t * UART_gpbInputBuffer;
static uint16_t  UART_gwDataInCounter = 0;

/*************************************************************************************************/
/*                                   Static Variables Section                                    */
/*************************************************************************************************/
static const uint32_t UART_gadwClockGateMask[MAX_UARTS] =
{
		SIM_SCGC4_UART0_MASK,
		SIM_SCGC4_UART1_MASK,
		SIM_SCGC4_UART2_MASK
};

static volatile uint8_t * const UART_gapbRegisters[MAX_UARTS][UART_REGISTERS] =
{
		{
			&UART0_BDH,
			&UART0_BDL,
			&UART0_C1,
			&UART0_C2,
			&UART0_S1,
			&UART0_S2,
			&UART0_C3,
			&UART0_D,
			&UART0_C4,
		},
		{
			&UART1_BDH,
			&UART1_BDL,
			&UART1_C1,
			&UART1_C2,
			&UART1_S1,
			&UART1_S2,
			&UART1_C3,
			&UART1_D,
			&UART1_C4,
		},
		{
			&UART2_BDH,
			&UART2_BDL,
			&UART2_C1,
			&UART2_C2,
			&UART2_S1,
			&UART2_S2,
			&UART2_C3,
			&UART2_D,
			&UART2_C4,
		}
};
/*************************************************************************************************/
/*                                   Global Constants Section                                    */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                   Static Constants Section                                    */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                      Functions Section                                        */
/*************************************************************************************************/
/*!
      \fn  UART_vfnInit
      \param bUARTToEnable: UART to be enabled. Values available on UART.h
      \param wBaudRate: Calculated value for desired baudrate
      \param bOverSamplingUart0: Just to be used when UART0 is required. Sets the oversampling ratio used for baudarate generation
      \param bUART0ClockSource: Just to be used when UART0 is required. Sets the clock source for UART0. Values available on UART.h
      \brief  Initializes the respective UART
      \return
*/
void UART_vfnInit(uint8_t bUARTToEnable, uint16_t wBaudRate, uint8_t bOverSamplingUart0, uint8_t bUART0ClockSource)
{
	
	uint8_t * pbBaudRateLow;
	uint8_t * pbBaudRateHigh;
	
	/* Confirm the UART to be enable is within the available on the SoC */
	if(bUARTToEnable < MAX_UARTS)
	{
		
		/* Set the proper UART clock gating */ 
		SIM_SCGC4 |= UART_gadwClockGateMask[bUARTToEnable];
		
		/* Take the UARTx baudrate register addresses */
		pbBaudRateLow = (uint8_t*)UART_gapbRegisters[bUARTToEnable][UART_BDL];
		pbBaudRateHigh = (uint8_t*)UART_gapbRegisters[bUARTToEnable][UART_BDH];
		
		/* load the baud rate to the registers */
		*pbBaudRateLow = (uint8_t)wBaudRate;
		wBaudRate >>= 8;
		wBaudRate &=  UART_BDH_SBR_MASK;
		*pbBaudRateHigh = wBaudRate;
		
		
		/* configure pins, IRQ and any special register for each UART */
		if(bUARTToEnable == UART0)
		{			
			/* Enable pins UART function */
			GPIO_vfnPinMux(GPIO_PORT_A,1,GPIO_MUX_ALT_2);
			GPIO_vfnPinMux(GPIO_PORT_A,2,GPIO_MUX_ALT_2);
			
			/* Enable IRQ */
			(void)dwfnNVIC_EnableIRQ(NVIC_UART0);
			 
			/* Select the UART clock source */
			SIM_SOPT2 |= SIM_SOPT2_UART0SRC(bUART0ClockSource);
			
			/* set the over sampling ratio for the UART0 */
			UART0_C4 = UART0_C4_OSR(bOverSamplingUart0-1); 
		}
		else if(bUARTToEnable == UART1)
		{
			/* Enable pins UART function */
			GPIO_vfnPinMux(GPIO_PORT_C,3,GPIO_MUX_ALT_3);
			GPIO_vfnPinMux(GPIO_PORT_C,4,GPIO_MUX_ALT_3);
			/* Enable IRQ */
			(void)dwfnNVIC_EnableIRQ(NVIC_UART1);
		}
		else
		{
			/* Enable pins UART function */
			GPIO_vfnPinMux(GPIO_PORT_D,2,GPIO_MUX_ALT_3);
			GPIO_vfnPinMux(GPIO_PORT_D,3,GPIO_MUX_ALT_3);
			/* Enable IRQ */
			(void)dwfnNVIC_EnableIRQ(NVIC_UART2);
		}
		
	}
		
}
/*!
      \fn  UART_vfnTxBuffer
      \param bUartToUse: UART to be used. Values available on UART.h
      \param pbTxBuffer: Pointer to the buffer data
      \param wDataToSend: Amount of bytes to be sent
      \brief  Sends thru UART the bytes requested
      \return
*/
void UART_vfnTxBuffer(uint8_t bUartToUse, uint8_t * pbTxBuffer, uint16_t wDataToSend)
{
	uint8_t * pbControlRegister;
	
	/* confirm if the driver is busy */
	if(!(UART_CHECK_STATUS(UART_TX_PROGRESS)))
	{
		/* confirm the UART exists */
		if(bUartToUse < MAX_UARTS)
		{
			/* take the UART_C2 register address to enable the transmitter */
			pbControlRegister = (uint8_t*)UART_gapbRegisters[bUartToUse][UART_C2];
			
			/* copy the buffer and data size from the application to driver internal back up to be used on the ISR */
			UART_gpbOutputBuffer = pbTxBuffer;
			UART_gwDataOutCounter = wDataToSend;
			
			/* set the driver as busy */
			UART_SET_STATUS(UART_TX_PROGRESS);
			
			/* enable the transmitter and its interrupt */
			*pbControlRegister |= UART_C2_TE_MASK|UART_C2_TIE_MASK;
		}
	}
}
/*!
      \fn  UART_vfnRxBuffer
      \param bUartToUse: UART to be used. Values available on UART.h
      \param pbTxBuffer: Pointer to the buffer where the data will be stored
      \param wDataToSend: Amount of bytes to be received
      \brief  Receives thru UART the bytes requested
      \return
*/
void UART_vfnRxBuffer(uint8_t bUartToUse, uint8_t * pbRxBuffer, uint16_t wDataToReceive)
{
	uint8_t * pbControlRegister;
	/* confirm if the driver is busy */
	if(!(UART_CHECK_STATUS(UART_RX_PROGRESS)))
	{
		/* confirm the UART exists */
		if(bUartToUse < MAX_UARTS)
		{
			/* take the UART_C2 register address to enable the transmitter */
			pbControlRegister = (uint8_t*)UART_gapbRegisters[bUartToUse][UART_C2];
			
			/* copy the buffer and data size to local variables to be used on ISR */
			UART_gpbInputBuffer = pbRxBuffer;
			UART_gwDataInCounter = wDataToReceive;
			
			/* set the driver as busy */
			UART_SET_STATUS(UART_RX_PROGRESS);
			/* enable RX and its interrupt */
			*pbControlRegister |= UART_C2_RE_MASK|UART_C2_RIE_MASK;
		}
	}
}
/*!
      \fn  UART0_IRQHandler
      \param None
      \brief  UART0 ISR
      \return
*/
void UART0_IRQHandler(void)
{
	/* check if the interrupt was the transmitter */
	if(UART0_S1&UART_S1_TDRE_MASK)
	{
		/* confirm there was a TX in progress */
		if(UART_CHECK_STATUS(UART_TX_PROGRESS))
		{
			/* if there is data to be sent */
			if(UART_gwDataOutCounter--)
			{
				UART0_D = *UART_gpbOutputBuffer++;
			}
			else
			{
				/* Clear flags and disable the transmitter */
				UART0_C2 &= ~(UART_C2_TE_MASK|UART_C2_TIE_MASK);
				UART_CLEAR_STATUS(UART_TX_PROGRESS);
			}
		}
	}
	/* check if the interrupt source was teh receiver */
	if(UART0_S1&UART_S1_RDRF_MASK)
	{
		/* confirm there's an RX in progress */
		if(UART_CHECK_STATUS(UART_RX_PROGRESS))
		{
			/* Take the data from the data register and decrement the data counter */
			*UART_gpbInputBuffer++ = UART0_D;
			UART_gwDataInCounter--;
			/* if the data counter reaches zero, disable the receiver and clear flags */
			if(!UART_gwDataInCounter)
			{
				UART0_C2 &= ~(UART_C2_RE_MASK|UART_C2_RIE_MASK);
				UART_CLEAR_STATUS(UART_RX_PROGRESS);
				UART_SET_STATUS(UART_RX_DONE);
			}
		}
		else
		{
			/* do a dummy read if no RX is in progress */
			(void)UART0_D;
		}
	}
}
/*!
      \fn  UART1_IRQHandler
      \param None
      \brief  UART1 ISR
      \return
*/
void UART1_IRQHandler(void)
{
	if(UART1_S1&UART_S1_TDRE_MASK)
	{
		if(UART_gwDataOutCounter--)
		{
			UART1_D = *UART_gpbOutputBuffer++;
		}
		else
		{
			UART1_C2 &= ~UART_C2_TE_MASK;
			UART_CLEAR_STATUS(UART_TX_PROGRESS);
		}
	}
	if(UART1_S1&UART_S1_RDRF_MASK)
	{
		if(UART_gwDataInCounter--)
		{
			*UART_gpbInputBuffer++ = UART1_D;
		}
		else
		{
			UART1_C2 &= ~UART_C2_RE_MASK;
			UART_CLEAR_STATUS(UART_RX_PROGRESS);
			UART_SET_STATUS(UART_RX_DONE);
		}
	}
}
/*!
      \fn  UART2_IRQHandler
      \param None
      \brief  UART2 ISR
      \return
*/
void UART2_IRQHandler(void)
{
	if(UART2_S1&UART_S1_TDRE_MASK)
	{
		if(UART_gwDataOutCounter--)
		{
			UART2_D = *UART_gpbOutputBuffer++;
		}
		else
		{
			UART2_C2 &= ~UART_C2_TE_MASK;
			UART_CLEAR_STATUS(UART_TX_PROGRESS);
		}
	}
	if(UART2_S1&UART_S1_RDRF_MASK)
	{
		if(UART_gwDataInCounter--)
		{
			*UART_gpbInputBuffer++ = UART2_D;
		}
		else
		{
			UART2_C2 &= ~UART_C2_RE_MASK;
			UART_CLEAR_STATUS(UART_RX_PROGRESS);
			UART_SET_STATUS(UART_RX_DONE);
		}
	}
}
