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

#ifndef UARTPROTOCOL_INC_UARTPROTOCOLHANDLER_H_
#define UARTPROTOCOL_INC_UARTPROTOCOLHANDLER_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <Hal/CY_Uart/inc/genericUart.h>

#define GROUP_SETUP 0x00
#define GROUP_PASSTHROUGH 0x01
#define GROUP_OSPCOMMANDS 0x02
#define GROUP_FEATURECOMMANDS 0x03
#define GROUP_SCENARIO 0x04
#define GROUP_CONFIGURATION_UC 0x10
#define GROUP_CONFIGURATION_EMULATOR 0x20
#define GROUP_OSP_ERROR 0x7E
#define GROUP_ACK 0x7F
#define GROUP_ACK_PASSTHROUGH 0xFF

#define DEVICE_ID 0x01

#define BROADCAST_ID 0x00
#define BROADCAST_GROUP 0x00
#define BROADCAST_LENGTH 1
#define BROADCAST_COMMENT 0x00

#define PASSTHROUGH_MIN_LENGTH 5
#define PASSTHROUGH_MAX_LENGTH 13

#define CONFIG_UC_MIN_LENGTH 1
#define CONFIG_UC_MAX_LENGTH 1
#define CONFIG_UC_LAST_COMMEND 0x03

#define LENGTH_ACK 6
#define LENGTH_ACK_LENGTH_BYTE 2
#define LENGTH_UART_ANS_HEADER 3
#define LENGTH_UART_ANS_CRC 1

enum UART_MSG
{
  UART_MSG_DEVICEID = 0,
  UART_MSG_GROUP = 1,
  UART_MSG_LENGTH = 2,
  UART_MSG_DATA0 = 3,
};

typedef union
{
  uint8_t buf[4];
  struct
  {
    uint8_t deviceID;
    uint8_t group;
    uint8_t length;
    uint8_t byte_3;
  } bit;
} uartHeader_t;

typedef union
{
  uint8_t status;
  struct
  {
    uint8_t crc_err :1;
    uint8_t timeout_err :1;
    uint8_t cmd_not_implemented :1;
    uint8_t incorrect_cmd :1;
    uint8_t reserv :4;
  } bit;
} uartStatus_t;

/**
 * @brief UART Receive new Message
 * This function must be called continuously and checks if new valid UART messages have arrived.
 * If there is a valid message the function uart_protocol_handler is called and the received message is forwarded to it.
 *
 */
void uart_receive_new_msg (void);

/**
 * @brief UART Protocol
 * The transferred message is checked for errors in the protocol. If errors are detected,
 * this will reported by the send_ack function.
 * If no errors are detected, an ACK without errors and the corresponding function are called.
 *
 * @param rxUartStatus, Status of the receiving message.
 * @param p_msg, Pointer to respose message via UART
 */
void uart_protocol_handler (rxUartStatus_t rxUartStatus, uint8_t *p_msg);

/**
 * @brief SEND ACK via UART
 *
 * This function generates a acknowledge message based on the given variables and
 * sends it out via the UART interface of the microcontroller.
 *
 * @param hdr, Passes the header information of the received UART message
 * @param uartStatus, Passes the status register where errors are stored
 */
void send_ack (uartHeader_t hdr, uartStatus_t uartStatus);

/**
 * @brief SEND NACK via UART
 *
 * This function generates a not acknowledge message based on the given variables and
 * sends it out via the UART interface of the �C.
 *
 * @param hdr, Passes the header information of the received UART message
 */
void send_nack (uartHeader_t hdr);

#ifdef __cplusplus
}
#endif

#endif /* UARTPROTOCOL_INC_UARTPROTOCOLHANDLER_H_ */
