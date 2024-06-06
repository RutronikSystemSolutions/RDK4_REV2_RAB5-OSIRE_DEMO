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

#ifndef UARTPROTOCOL_INC_OSPCOMMANDS_H_
#define UARTPROTOCOL_INC_OSPCOMMANDS_H_

#include <Osp/inc/osireDevice.h>
#include <UartProtocol/inc/uartProtocolHandler.h>

#define OSP_OSIRE_READ_OTP_COMPLETE 0x80

#define LENGTH_UART_PWM_READ 10
#define LENGTH_UART_INIT 5
#define LENGTH_UART_OTP_READ 11
#define LENGTH_UART_OTP_COMPLETE 35
#define LENGTH_UART_SETUP_READ 4
#define LENGTH_UART_TEMP_STATUS 5
#define LENGTH_UART_STATUS 4
#define LENGTH_UART_TEMP 4
#define LENGHT_UART_COM_STATUS 4
#define LENGTH_UART_LED_STATUS 4

#define LENGTH_UART_OTTH_READ 6

#define LENGTH_OSP_ERROR_LENGTH_BYTE 1
#define LENGTH_OSP_ERROR 5

/**
 * @brief OSP Commands for microcontroller will be executed
 *
 * The requested OSP command will be executed. If the LED sends a response,
 * this is output via the UART interface.
 *
 * @param p_msg, Pointer to response message via UART
 * @param hdr, Passes the header information of the received UART message
 */

void osp_commands (uint8_t *p_msg, uartHeader_t hdr);

void send_uart_temp_status (uartHeader_t hdr, osireTempStatus_t tempStatus);

#endif /* UARTPROTOCOL_INC_OSPCOMMANDS_H_ */
