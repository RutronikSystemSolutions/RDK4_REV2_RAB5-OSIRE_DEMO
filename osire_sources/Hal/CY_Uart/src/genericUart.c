/*****************************************************************************
 * Copyright 2022 by ams OSRAM AG                                            *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 */
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include <Driver/CY_Uart/inc/uartDriver.h>
//#include <Hal/Gpios/inc/pin.h>
//#include <Hal/System/inc/initSystem.h>
#include <Hal/CY_Uart/inc/genericUart.h>
#include <Hal/CY_Uart/inc/uart.h>
//#include "SDK/platform/drivers/inc/interrupt_manager.h"
//#include "Sources/SwTimer/inc/swTimer.h"

#include "sys_timer.h"

/*****************************************************************************/
/*****************************************************************************/
// ---- DEFINES: -----------------------------------
//#define UART_DRIVER_BUFFER_SIZE 264  // Buffer Size > Longest Possible Message 264

/*****************************************************************************/
/*****************************************************************************/
// ---- VARIABLES: -----------------------------------
static uartReceiveStates_t uartCurrentState = IDLE;
static uint8_t workingBuffer[UART_DRIVER_BUFFER_SIZE];
static uint32_t messageDataLength = 0;
static uint32_t rxTimeoutTimestamp = 0;

cy_stc_scb_uart_context_t KITPROG_UART_Context;

/*****************************************************************************/
/*****************************************************************************/
// ---- FUNCTIONS: -----------------------------------
// UART Receiver FSM -> Call continuously to check for incoming Bytes according to protocol
rxUartStatus_t uart_receive_fsm(void)
    {
    rxUartStatus_t retVal = IDLE;
    switch (uartCurrentState)
	{ // Idle -> Wait for Rx
    case IDLE:
	if (uart_driver_get_rx_buffer_level() > 0)
	    {
	    retVal = RX_START;
	    uartCurrentState = RX_START;
	    }
	else //nothing received
	    {
	    retVal = RX_IDLE;
	    }

	break;
	//Rx Start -> Start receiving data
    case RX_START:
    memset(workingBuffer, 0x00, sizeof(workingBuffer));
	messageDataLength = 0;
	uartCurrentState = RX_WAIT_LENGTH;
	retVal = RX_BUSY;
	break;
	// Rx Wait Length -> Wait until the header (length of the message) is received
    case RX_WAIT_LENGTH:
	if (uart_driver_get_rx_buffer_level() >= MESSAGE_HEADER)
	    {
	    uart_driver_get_rx_data(workingBuffer, MESSAGE_HEADER);
	    messageDataLength = workingBuffer[MESSAGE_DATA_LENGTH_INDEX];
	    uartCurrentState = RX_ACTIVE;
	    rxTimeoutTimestamp = get_system_time_ms() + UART_RX_TIMEOUT_MS;
	    }
	retVal = RX_BUSY;
	break;
	//Rx Active -> Wait until correct amount of bytes has arrived, or we timeout
    case RX_ACTIVE:
		if (uart_driver_get_rx_buffer_level() >= (messageDataLength + MESSAGE_OVERHEAD_CRC))
		{
			uart_driver_get_rx_data((&workingBuffer[MESSAGE_HEADER]), (messageDataLength + MESSAGE_OVERHEAD_CRC));
			uartCurrentState = RX_COMPLETE;
		}
		else if (get_system_time_ms() > rxTimeoutTimestamp)
		{
			uartCurrentState = RX_ABORT;
		}

	retVal = RX_BUSY;
	break;

	//Rx Complete -> Complete receiving data (Complete Message should have arrived) + Transfer Data into a Working Buffer
    case RX_COMPLETE:
	uart_driver_complete_receive_data_using_int();
	uartCurrentState = RX_START;
	retVal = RX_DONE;
	break;
	// Rx Abort -> Received message is shorter than expected, restart receive process.
    case RX_ABORT:
	uart_driver_complete_receive_data_using_int();
	memset(workingBuffer, 0x00, sizeof(workingBuffer));
	uartCurrentState = RX_START;
	retVal = RX_TIMEOUT_ERROR;
	break;
    default:
	break;
	}

    return (retVal);
    }

/*****************************************************************************/
/*****************************************************************************/
// Fetch Working Buffer after command has been received
uint8_t* get_working_buffer(uint16_t *p_sizeOfWorkingBuffer)
    {
    *p_sizeOfWorkingBuffer = UART_DRIVER_BUFFER_SIZE;
    return (workingBuffer);
    }

/*****************************************************************************/
/*****************************************************************************/
// Initial Receiver FSM Trigger -> Call once in startup
void start_uart_receive(void)
    {
    if (uartCurrentState == IDLE)
	{
	uartCurrentState = RX_START;
	}
    }
