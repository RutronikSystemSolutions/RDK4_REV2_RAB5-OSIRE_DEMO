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

#include <Common/inc/osireEvkDefines.h>
#include <Crc/inc/crc.h>
#include <Hal/CY_Uart/inc/uart.h>
#include <UartProtocol/inc/configUc.h>
#include <UartProtocol/inc/uartProtocolHandler.h>
#include <Demos/MinimalRgbStripe/inc/minimalRgbStripe.h>

#define POSITION_OFFSET_CONFIG 4

void config_uc (uint8_t *p_msg, uartHeader_t hdr)
{
  uint8_t uartBuffer[8];

  uartStatus_t uartStatus;
  uartStatus.status = 0;

  UNUSED(p_msg);

  uartBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
  uartBuffer[UART_MSG_GROUP] = hdr.bit.group;
  uartBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
  switch (hdr.bit.byte_3)
    {
    case CONFIG_DEVICE_GET_TYPE:
      send_ack (hdr, uartStatus);
      uartBuffer[UART_MSG_LENGTH] = LENGTH_GET_TYPE;
      uartBuffer[UART_MSG_DATA0 + 1] = DEVICE_ID;
      uartBuffer[UART_MSG_DATA0 + LENGTH_GET_TYPE] = crc (
          uartBuffer, UART_MSG_DATA0 + LENGTH_GET_TYPE);
      uart_send_data_blocking (uartBuffer,
                               LENGTH_GET_TYPE + POSITION_OFFSET_CONFIG);
      return;
    case CONFIG_DEVICE_GET_VERSION:
      send_ack (hdr, uartStatus);
      uartBuffer[UART_MSG_LENGTH] = LENGTH_GET_VERSION;
      uartBuffer[UART_MSG_DATA0 + 1] = CONFIG_DEVICE_VERSION_MAJOR;
      uartBuffer[UART_MSG_DATA0 + 2] = CONFIG_DEVICE_VERSION_MINOR;
      uartBuffer[UART_MSG_DATA0 + LENGTH_GET_VERSION] = crc (
          uartBuffer, UART_MSG_DATA0 + LENGTH_GET_VERSION);
      uart_send_data_blocking (uartBuffer,
                               LENGTH_GET_VERSION + POSITION_OFFSET_CONFIG);
      return;
    case CONFIG_DEVICE_GETFW_VERSION:
      send_ack (hdr, uartStatus);
      uartBuffer[UART_MSG_LENGTH] = LENGTH_GETFW_VERSION;
      uartBuffer[UART_MSG_DATA0 + 1] = FW_VERSION_MAJOR;
      uartBuffer[UART_MSG_DATA0 + 2] = FW_VERSION_MINOR;
      uartBuffer[UART_MSG_DATA0 + 3] = FW_VERSION_REVISION;
      uartBuffer[UART_MSG_DATA0 + LENGTH_GETFW_VERSION] = crc (
          uartBuffer, UART_MSG_DATA0 + LENGTH_GETFW_VERSION);
      uart_send_data_blocking (uartBuffer,
                               LENGTH_GETFW_VERSION + POSITION_OFFSET_CONFIG);
      return;
    default:
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;
    }
}
