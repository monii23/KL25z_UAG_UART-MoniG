 /*************************************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2010 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************//*!
 *
 * @file UART.h
 *
 * @author B22385
 *
 * @date Dec 21, 2012
 *
 * @brief 
 *************************************************************************************************/


#ifndef UART_H_
#define UART_H_

/*************************************************************************************************/
/*                                      Includes Section                                         */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                  Defines & Macros Section                                     */
/*************************************************************************************************/
enum
{
	UART_INTERRUPT_ENABLE = 0,
	UART_DMA_ENABLE,
	UART_PARITY_ENABLE,
	UART_PARITY_ODD
};

enum eUARTStatus
{
	UART_TX_PROGRESS = 0,
	UART_RX_PROGRESS,
	UART_TX_DONE,
	UART_RX_DONE
};
enum eUARTS
{
	UART0 = 0,
	UART1,
	UART2,
	MAX_UARTS
};

enum eUART0_CLK
{
	UART0_CLK_FLL_PLL = 1,
	UART0_CLK_OSC,
	UART0_CLK_IRC
};
#define UART_UART0_SRC_CLK			(10000000)

#define UART_INTERRUPT_ENABLE_MASK	(1<<UART_INTERRUPT_ENABLE)
#define UART_DMA_ENABLE_MASK		(1<<UART_DMA_ENABLE)
#define UART_PARITY_ENABLE_MASK		(1<<UART_PARITY_ENABLE)
#define	UART_PARITY_ODD_MASK		(1<<UART_PARITY_ODD)

#define UART_CHECK_STATUS(X)	(UART_dwDriverStatus&(1<<X))
#define UART_SET_STATUS(X)		(UART_dwDriverStatus |= (1<<X))
#define UART_CLEAR_STATUS(X)	(UART_dwDriverStatus &=~ (1<<X))
/*************************************************************************************************/
/*                                      Typedef Section                                          */
/*************************************************************************************************/


/*************************************************************************************************/
/*                                Function-like Macros Section                                   */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                  Extern Constants Section                                     */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                  Extern Variables Section                                     */
/*************************************************************************************************/
extern uint32_t UART_dwDriverStatus;
/*************************************************************************************************/
/*                                Function Prototypes Section                                    */
/*************************************************************************************************/
void UART_vfnInit(uint8_t bUARTToEnable, uint16_t wBaudRate, uint8_t bOverSamplingUart0, uint8_t bUART0ClockSource);
void UART_vfnTxBuffer(uint8_t bUartToUse, uint8_t * pbTxBuffer, uint16_t wDataToSend);
void UART_vfnRxBuffer(uint8_t bUartToUse, uint8_t * pbRxBuffer, uint16_t wDataToReceive);
/*************************************************************************************************/

#endif /* UART_H_ */
