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
#include <Driver/CY_Uart/inc/uartDriver.h>
#include <HAL/CY_System/inc/initSystem.h>
#include <Hal/CY_Uart/inc/uart.h>
#include "sys_timer.h"

/*****************************************************************************/
/*****************************************************************************/
// ---- DEFINES: -----------------------------------



// ---- VARIABLES: -----------------------------------
static errorUart_t errorUart = NO_ERROR_UART;
static status_t uartCYError = STATUS_SUCCESS ;

/*****************************************************************************/
/*****************************************************************************/
// ---- FUNCTIONS: -----------------------------------
// UART Initialization
errorUart_t hal_uart_init(void)
    {
    uartCYError = STATUS_SUCCESS;

    uartCYError = uart_driver_init();

    return (uartCYError);
    }

/*****************************************************************************/
/*****************************************************************************/
// UART Send Data (Blocking)
errorUart_t uart_send_data_blocking (uint8_t *p_bufferSend, uint8_t count)
{
  errorUart = NO_ERROR_UART;
  uartCYError = STATUS_SUCCESS;

  uartCYError = uart_driver_send_data (p_bufferSend, count);
  if (uartCYError == STATUS_SUCCESS)
    {
      uint32_t txTimeoutTimestamp = get_system_time_ms() + UART_TX_TIMEOUT_MS;
      // Wait until the data is sent
      while ((uart_driver_get_status () == STATUS_BUSY)
          && (txTimeoutTimestamp > get_system_time_ms()))
        {

        }
      if (txTimeoutTimestamp > get_system_time_ms())
        {
          uartCYError = STATUS_TIMEOUT;
        }
      else
        {
          uartCYError = uart_driver_get_status ();
        }
      uart_driver_complete_send_data_using_int ();
    }
  // Error Handling -> Map NXP Error to HAL Error
  switch (uartCYError)
    {
    case STATUS_SUCCESS:
      errorUart = TIMEOUT_BLOCKING_UART;
      break;
    case STATUS_BUSY:
      errorUart = BUSY_BLOCKING_UART;
      break;
    case STATUS_ERROR:
      errorUart = ERROR_BLOCKING_UART;
      break;
    default:
      break;
    }

  return (errorUart);
}

/*****************************************************************************/
/*****************************************************************************/
// UART Receive Data (Non-Blocking)
errorUart_t uart_receive_data (void)
{
  errorUart = NO_ERROR_UART;
  uartCYError = STATUS_SUCCESS;

  uartCYError = uart_driver_receive_data ();

  // Error Handling -> Map NXP Error to HAL Error
  switch (uartCYError)
    {
    case STATUS_BUSY:
      errorUart = RX_BUSY_ERROR_NONBLOCKING_UART;
      break;
    default:
      break;
    }

  return (errorUart);
}
