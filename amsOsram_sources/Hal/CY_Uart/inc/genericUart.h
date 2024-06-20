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
 *****************************************************************************/

#ifndef HAL_UART_INC_GENERICUART_H_
#define HAL_UART_INC_GENERICUART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define MESSAGE_OVERHEAD          4  // <DEVICEID><GROUP><LENGTH> [...] <CRC>
#define MESSAGE_HEADER            3  // <DEVICEID><GROUP><LENGTH>
#define MESSAGE_DATA_LENGTH_INDEX 2  // <0-DEVICEID><1-GROUP><2-LENGTH>
#define MESSAGE_OVERHEAD_CRC      1  // CRC is 1 byte long

// Receiving State Machine
typedef enum
{
  IDLE = 0x00,
  RX_START = 0x01,
  RX_ACTIVE = 0x02,
  RX_ABORT = 0x03,
  RX_WAIT_LENGTH = 0x04,
  RX_COMPLETE = 0x05
} uartReceiveStates_t;

// UART Error Handling
typedef enum
{
  NO_ERROR_UART = 0x00,
  INIT_ERROR_UART = 0x01,
  TIMEOUT_BLOCKING_UART = 0x10,
  BUSY_BLOCKING_UART = 0x11,
  ERROR_BLOCKING_UART = 0x12,
  BUSY_POLLING_UART = 0x20,
  BUSY_SEND_UART = 0x21,

  RX_TIMEOUT_BLOCKING_UART = 0x30,
  RX_BUSY_ERROR_BLOCKING_UART = 0x31,
  RX_FRAMING_ERROR_BLOCKING_UART = 0x32,
  RX_NOISE_ERROR_BLOCKING_UART = 0x33,
  RX_PARITY_ERROR_BLOCKING_UART = 0x34,
  RX_OVERRUN_ERROR_BLOCKING_UART = 0x35,
  RX_DMA_ERROR_BLOCKING_UART = 0x36,

  RX_BUSY_ERROR_POLLING_UART = 0x40,
  RX_OVERRUN_ERROR_POLLING_UART = 0x41,

  RX_BUSY_ERROR_NONBLOCKING_UART = 0x50,
  RX_ABORTED_ERROR_NONBLOCKING_UART = 0x51,
  RX_TIMEOUT_ERROR_NONBLOCKING_UART = 0x52,
  RX_OVERRUN_ERROR_NONBLOCKING_UART = 0x53,
  RX_FRAMING_ERROR_NONBLOCKING_UART = 0x54,
  RX_PARITY_ERROR_NONBLOCKING_UART = 0x55,
  RX_NOISE_ERROR_NONBLOCKING_UART = 0x56,
  RX_ERROR_NONBLOCKING_UART = 0x57
} errorUart_t;

// Rx UART Status
typedef enum
{
  RX_IDLE = 0x00, RX_BUSY = 0x01, RX_DONE = 0x02, RX_TIMEOUT_ERROR = 0x03
} rxUartStatus_t;

// ---- METHODS: -----------------------------------

// UART Receiver FSM -> Call continuously to check for incoming Bytes according to protocol
rxUartStatus_t uart_receive_fsm (void);

// Fetch Working Buffer after command has been received
uint8_t* get_working_buffer (uint16_t *p_sizeOfWorkingBuffer);

// Initial Receiver FSM Trigger -> Call once in startup
void start_uart_receive (void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_INC_GENERICUART_H_ */
