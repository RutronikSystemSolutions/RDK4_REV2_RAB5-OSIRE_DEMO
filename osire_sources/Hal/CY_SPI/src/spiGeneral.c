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

//#include <Common/inc/osireEvkDefines.h>
#include <Hal/CY_Spi/inc/spiGeneral.h>
#include <Hal/CY_Spi/inc/spiMaster.h>
#include <Hal/CY_Spi/inc/spiSlave.h>
#include "Manchester/inc/manchester.h"
#include "sys_timer.h"

#define TIME_OUT_MS_FOR_ANSWER 100

/*****************************************************************************/
/*****************************************************************************/
// Function for sending only to the LEDs -> Blocking!
errorSpi_t send_data_over_spi_blocking (uint8_t *p_bufferSend, uint16_t count)
{
  errorSpi_t errorCode;
  uint8_t bufferTemp[count * 2];

  manchester_encoding_buffer_swap (p_bufferSend, bufferTemp, count,sizeof(bufferTemp));

  errorCode = hal_spi_master_send_blocking (bufferTemp, sizeof(bufferTemp));
  return (errorCode);
}

/*****************************************************************************/
/*****************************************************************************/
//Function for sending and receiving of Messages to and from the LED over 2
// different SPI interfaces
errorSpi_t send_and_receive_data_over_spi_blocking(uint8_t *p_bufferSend,uint8_t *p_bufferReceive, uint16_t countSend, uint16_t countReceive)
{
	errorSpi_t errorCode = NO_ERROR_SPI; // init with no error
	uint8_t bufferTemp[countSend * 2];
	uint16_t byteCount = countSend * 2;
	uint32_t rx_read_bytes = 0;

	manchester_encoding_buffer_swap(p_bufferSend, bufferTemp, countSend,sizeof(bufferTemp));

	errorCode = hal_spi_master_send_blocking(bufferTemp, byteCount);
	if(errorCode != NO_ERROR_SPI)
	{
		return errorCode;
	}

	//***CS for Slave****
	Cy_GPIO_Write(CS_SW_OUTPUT_PORT, CS_SW_OUTPUT_PIN, 0);

	uint32_t sysTime = get_system_time_ms();
	uint32_t sysTimeOut = sysTime + TIME_OUT_MS_FOR_ANSWER;

	errorCodeSpiNewMessage_t newMessageState = SPI_NO_NEW_MESSAGE;

	while (newMessageState != SPI_NEW_MESSAGE_OK)
	{
		sysTime = get_system_time_ms();

		if (sysTime > sysTimeOut)
		{
			errorCode = SPI_ERROR_TIME_OUT;
			break;
		}

		while (rx_read_bytes < countReceive)
		{
			rx_read_bytes = Cy_SCB_SPI_GetNumInRxFifo(sSPI_HW);
			sysTime = get_system_time_ms();
			if (sysTime > sysTimeOut)
			{
				errorCode = SPI_ERROR_TIME_OUT;
				break;
			}
		}
		rx_read_bytes = Cy_SCB_SPI_ReadArray(sSPI_HW, p_bufferReceive,countReceive);

		if (rx_read_bytes != countReceive)
		{
			rx_read_bytes = Cy_SCB_SPI_GetNumInRxFifo(sSPI_HW);
			newMessageState = SPI_NO_NEW_MESSAGE;
		}
		else
		{
			newMessageState = SPI_NEW_MESSAGE_OK;
			break;
		}

		if (newMessageState == SPI_ERROR_DATA_CORRUPTION)
		{
			break;
		}
	};

	Cy_GPIO_Write(CS_SW_OUTPUT_PORT, CS_SW_OUTPUT_PIN, 1);

	return (errorCode);
}

/*****************************************************************************/
/*****************************************************************************/
// Function for sending only to the LEDs
errorSpi_t send_data_over_spi_non_blocking (uint8_t *p_bufferSend,
                                            uint16_t count, uint32_t delay,
                                            bool restart)
{
  errorSpi_t err;
  uint8_t bufferTemp[count * 2];
  uint16_t byteCount = count * 2;

  manchester_encoding_buffer_swap (p_bufferSend, bufferTemp, count,sizeof(bufferTemp));

  err = hal_spi_master_send_non_blocking (bufferTemp, byteCount, delay);

  if (err == NO_ERROR_SPI)
    {
      if (restart == true)
        {
          restart_non_blocking_spi_send ();
        }
    }

  return (err);
}

/*****************************************************************************/
/*****************************************************************************/
// This function is for SPI non-blocking sending only
errorSpi_t send_and_receive_data_over_spi_non_blocking (uint8_t *p_bufferSend, uint16_t count, bool restart)
{
  errorSpi_t err;
  uint8_t bufferTemp[count * 2 ];

  uint16_t byteCount = count * 2;

  manchester_encoding_buffer_swap (p_bufferSend, bufferTemp, count,sizeof(bufferTemp));

  err = hal_spi_master_send_non_blocking (bufferTemp, byteCount, 0);

  if (err == NO_ERROR_SPI)
    {
      if (restart == true)
        {
          restart_non_blocking_spi_send ();
        }
    }

  return (err);
}

errorSpi_t restart_non_blocking_spi_send (void)
{
    errorSpi_t err = 0;
  return (err);

}

uint8_t* get_pointer_next_message_non_blocking (errorSpi_t *p_err)
{

  errorCodeSpiNewMessage_t errorSPI = SPI_NO_NEW_MESSAGE;

  uint8_t *p_messageBuffer = NULL;
	 // hal_get_new_message (&errorSPI);

  if (errorSPI == SPI_NEW_MESSAGE_OK)
    {
      *p_err = NO_ERROR_SPI;
    }
  else
    {
      *p_err = SPI_NO_NEW_DATA_RECEIVED;
      p_messageBuffer = NULL;
    }

  return (p_messageBuffer);
}

errorSpi_t spi_receive_control (void)
{
  spiReceiveStatusSlave_t status = 0;
  errorSpi_t err = NO_ERROR_SPI;

  if (status == SPI_RECEIVE_ERROR_CORRUPT_DATA)
    {
      err = SPI_ERROR_CORRUPT_DATA;
    }

  return (err);
}

void spi_receive_reset_buffer (void)
{
}

void spi_receive_reset (void)
{
}
