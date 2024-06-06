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
#include <Hal/CY_Spi/inc/spiGeneral.h>
#include <Hal/CY_Uart/inc/uart.h>
#include <string.h>
#include <UartProtocol/inc/passthrough.h>
#include <UartProtocol/inc/uartProtocolHandler.h>

void passthrough (uint8_t *p_msg, uartHeader_t hdr)
{
  p_msg = p_msg + 4; //set pointer to first position of Message to LED
  if (hdr.bit.byte_3 == 0) // without answer from LED
    {
      errorSpi_t err1 = send_data_over_spi_blocking (p_msg,
                                                      hdr.bit.length - 1);
      if (err1 != NO_ERROR_SPI)
        {
          uart_send_data_blocking (p_msg, 1);
        }
      return;
    }
  else // with answer from LED
    {
      uint8_t rspBuffer[hdr.bit.byte_3];
      errorSpi_t err1 = send_and_receive_data_over_spi_blocking (
          p_msg, rspBuffer, hdr.bit.length - 1, hdr.bit.byte_3);
      if (err1 != NO_ERROR_SPI)
        {
          hdr.bit.group = GROUP_ACK_PASSTHROUGH;
          send_nack (hdr);
        }
      else
        {
          uint8_t uartBuffer[hdr.bit.byte_3];
          uartBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
          uartBuffer[UART_MSG_GROUP] = GROUP_PASSTHROUGH;
          uartBuffer[UART_MSG_LENGTH] = hdr.bit.byte_3;
          memcpy (&uartBuffer[UART_MSG_DATA0], rspBuffer, hdr.bit.byte_3);
          uartBuffer[hdr.bit.byte_3 + LENGTH_UART_ANS_HEADER] = crc (
              uartBuffer, hdr.bit.byte_3 + LENGTH_UART_ANS_HEADER);
          uart_send_data_blocking (
              uartBuffer,
              hdr.bit.byte_3 + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        }
    }
}
