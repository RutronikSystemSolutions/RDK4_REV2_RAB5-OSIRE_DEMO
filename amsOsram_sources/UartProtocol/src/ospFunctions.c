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
#include <Hal/Osire/inc/osire.h>
#include <Hal/CY_Uart/inc/uart.h>
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>
#include <UartProtocol/inc/ospCommands.h>
#include <UartProtocol/inc/ospFunctions.h>
#include <UartProtocol/inc/uartProtocolHandler.h>
#include <osp2/inc/osp2.h>
#include <SwTimer/inc/swTimer.h>       // eg delay_ms()

void osp_functions (uint8_t *p_msg, uartHeader_t hdr)
{

  p_msg = p_msg + 4;
  uint16_t deviceAddress = *p_msg << 8;
  p_msg++;
  deviceAddress = deviceAddress + *p_msg;
  uartStatus_t uartStatus;
  uartStatus.status = 0;
  uint16_t addr;
  addr =  (hdr.bit.param1 <<8) + hdr.bit.param2;

  enum OSP_ERROR_CODE ospErrorCode = OSP_NO_ERROR;
  switch (hdr.bit.byte_3)
	{

		case MULTIPLEADCREAD:
		{
			uint16_t n_samples;
			uint16_t sum;
			uint16_t avg;
			uint16_t adcdata;

			n_samples = hdr.bit.param3;
			sum = 0;
			for(int i=0; i<n_samples; i++) {
				ospErrorCode = osp2_send_ADCdataread(addr, &adcdata);
				sum += adcdata;
				Cy_SysLib_Delay(20);
			}

			avg = sum / n_samples;

			uint8_t sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
			sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
			sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
			sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_16bit;
			sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

			if (ospErrorCode != OSP_NO_ERROR)
			{
				send_nack (hdr);
				sendBuffer[UART_MSG_DATA0 + 1] =  255;
				sendBuffer[UART_MSG_DATA0 + 2] =  255;
			}
			else
			{
				send_ack (hdr, uartStatus);
				sendBuffer[UART_MSG_DATA0 + 1] =  avg >> 8;
				sendBuffer[UART_MSG_DATA0 + 2] =  ((uint8_t) avg ) & 0xFF;
			}

			sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER);
			uart_send_data_blocking (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
			return;
		}

	/*	case OSP_IDENTIFY:
		{
			uint32_t id;
			uint8_t idcast;
			ospErrorCode = osp2_send_identify(addr, &id);
			idcast = (uint8_t)id;
			uint8_t sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
			sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
			sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
			sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_ID;
			sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

			if (ospErrorCode != OSP_NO_ERROR)
			{
				send_nack (hdr);
				sendBuffer[UART_MSG_DATA0 + 1] =  255;
			}
			else
			{
				send_ack (hdr, uartStatus);
				sendBuffer[UART_MSG_DATA0 + 1] =  idcast;
			}

			sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER);
			uart_send_data_blocking (sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
			return;

		}*/
	}
}
