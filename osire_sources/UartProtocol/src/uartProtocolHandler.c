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

#include <Crc/inc/crc.h>
#include <Hal/CY_Uart/inc/genericUart.h>
#include <string.h>
#include <UartProtocol/inc/configUc.h>
#include <UartProtocol/inc/groupSetup.h>
#include <UartProtocol/inc/ospCommands.h>
#include <UartProtocol/inc/passthrough.h>
#include <UartProtocol/inc/uartProtocolHandler.h>
#include <Hal/CY_Uart/inc/uart.h>

void uart_receive_new_msg (void)
{
  rxUartStatus_t uartStatus;

  uartStatus = uart_receive_fsm ();
  if ((uartStatus == RX_DONE) || (uartStatus == RX_TIMEOUT_ERROR))
    {
      uint16_t sizeOfWorkingBuffer;
      uint8_t *p_uartbuffer;
      p_uartbuffer = get_working_buffer (&sizeOfWorkingBuffer);
      uart_protocol_handler (uartStatus, p_uartbuffer);
    }
}

void uart_protocol_handler (rxUartStatus_t rxUartStatus, uint8_t *p_msg)
{
  uartHeader_t hdr;
  memcpy (hdr.buf, p_msg, 4);

  uartStatus_t uartStatus;
  uartStatus.status = 0;

  if (rxUartStatus == RX_TIMEOUT_ERROR)
    {
      uartStatus.bit.timeout_err = 1;
      send_ack (hdr, uartStatus);
      return;
    }
  if (crc (p_msg, (hdr.bit.length + 4)) != 0) // Check if CRC is right
    {
      uartStatus.bit.crc_err = 1;
      send_ack (hdr, uartStatus);
      return;
    }

  if (hdr.bit.deviceID == BROADCAST_ID) // Broadcast for Device_ID
    {
      if ((hdr.bit.group == BROADCAST_GROUP)
          && (hdr.bit.length == BROADCAST_LENGTH)
          && (hdr.bit.byte_3 == BROADCAST_COMMENT))
        {
          config_uc (p_msg, hdr);
        }
      else if (hdr.bit.group == GROUP_ACK)
        {
          //ignore ACK
          return;
        }
      else
        {
          uartStatus.bit.incorrect_cmd = 1;
          send_ack (hdr, uartStatus);
        }
      return;
    }

  if (hdr.bit.deviceID != DEVICE_ID) // Check if Message is for this microcontroller device
    {
      uartStatus.bit.incorrect_cmd = 1;
      send_ack (hdr, uartStatus);
      return;
    }

  switch (hdr.bit.group)
    {
    case GROUP_SETUP:
      {
        setupCommandsErrorCode_t error = change_setup (p_msg, hdr);
        if (error == CHANGE_SETUP_COMMAND_NOT_IMPLEMENTED)
          {
            uartStatus.bit.incorrect_cmd = 1;
          }
        else if (error == CHANGE_SETUP_ERROR_WRONG_LENGHT)
          {
            uartStatus.bit.incorrect_cmd = 1;
          };
        send_ack (hdr, uartStatus);
        return;
      }
    case GROUP_PASSTHROUGH:
      if ((hdr.bit.length >= PASSTHROUGH_MIN_LENGTH)
          && (hdr.bit.length <= PASSTHROUGH_MAX_LENGTH))
        {
          hdr.bit.group = GROUP_ACK_PASSTHROUGH;
          send_ack (hdr, uartStatus);
          passthrough (p_msg, hdr);
        }
      else
        {
          uartStatus.bit.incorrect_cmd = 1;
          send_ack (hdr, uartStatus);
        }
      return;

    case GROUP_OSPCOMMANDS:
      osp_commands (p_msg, hdr);
      return;

    case GROUP_FEATURECOMMANDS:
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;

    case GROUP_SCENARIO:
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;

    case GROUP_CONFIGURATION_UC:

      if ((hdr.bit.length < CONFIG_UC_MIN_LENGTH)
          || (hdr.bit.length > CONFIG_UC_MAX_LENGTH))
        {
          uartStatus.bit.incorrect_cmd = 1;
          send_ack (hdr, uartStatus);
        }
      else
        {
          config_uc (p_msg, hdr);
        }
      return;

    case GROUP_CONFIGURATION_EMULATOR:
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;

    case GROUP_OSP_ERROR: //case for status ???
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;

    case GROUP_ACK:
      // do ACK action
      return;

    default:
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;
    }
}

void send_ack (uartHeader_t hdr, uartStatus_t uartStatus)
{
  uint8_t buffer[LENGTH_ACK];
  buffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
  buffer[UART_MSG_GROUP] = GROUP_ACK;
  buffer[UART_MSG_LENGTH] = LENGTH_ACK_LENGTH_BYTE;
  if (hdr.bit.group == GROUP_ACK_PASSTHROUGH)
    {
      buffer[UART_MSG_DATA0] = GROUP_ACK_PASSTHROUGH;
    }
  else
    {
      buffer[UART_MSG_DATA0] = hdr.bit.byte_3;
    }
  buffer[UART_MSG_DATA0 + 1] = uartStatus.status;
  buffer[LENGTH_ACK - 1] = crc (buffer, LENGTH_ACK - 1);
  uart_send_data_blocking (buffer, LENGTH_ACK);
}

void send_nack (uartHeader_t hdr)
{
  uint8_t buffer[LENGTH_OSP_ERROR];
  buffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
  buffer[UART_MSG_GROUP] = GROUP_OSP_ERROR;
  buffer[UART_MSG_LENGTH] = LENGTH_OSP_ERROR_LENGTH_BYTE;
  if (hdr.bit.group == GROUP_ACK_PASSTHROUGH)
    {
      buffer[UART_MSG_DATA0] = GROUP_ACK_PASSTHROUGH;
    }
  else
    {
      buffer[UART_MSG_DATA0] = hdr.bit.byte_3;
    }
  buffer[LENGTH_OSP_ERROR - 1] = crc (buffer, LENGTH_OSP_ERROR - 1);
  uart_send_data_blocking (buffer, LENGTH_OSP_ERROR);
}
