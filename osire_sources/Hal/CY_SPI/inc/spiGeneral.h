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

#ifndef HAL_SPI_INC_SPI_GENERAL_H_
#define HAL_SPI_INC_SPI_GENERAL_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include "cy_pdl.h"
#include "cycfg.h"

typedef enum
{
  NO_ERROR_SPI = 0x00,
  MAX_LENGHT_TO_HIGH_SPI = 0x01,
  SPI_BUSY = 0x02,
  SPI_SEND_ERROR = 0x03,
  SPI_RECEIVE_ERROR = 0x04,
  SPI_DUMMY_RECEIVE_ERROR = 0x05,
  SPI_ERROR_TIME_OUT = 0x06,
  SPI_NO_RESET_NON_BLOCKING_SEND = 0x07,
  SPI_COULD_NOT_ADD_NEW_DATA_BUFFER_FULL = 0x08,
  SPI_NO_NEW_DATA_RECEIVED = 0x09,
  SPI_ERROR_CORRUPT_DATA = 0x0A,
  SPI_ERROR_NOT_DEF = 0xFF
} errorSpi_t;

extern errorSpi_t spiHalError;
//extern status_t spiHalNxpError;

void example_spi (void);
errorSpi_t send_data_over_spi_blocking (uint8_t *bufferSend, uint16_t count);
errorSpi_t send_and_receive_data_over_spi_blocking (uint8_t *p_bufferSend,
                                                    uint8_t *p_bufferReceive,
                                                    uint16_t countSend,
                                                    uint16_t countReceive);

errorSpi_t send_data_over_spi_non_blocking (uint8_t *p_bufferSend,
                                            uint16_t count, uint32_t delay,
                                            bool restart);
errorSpi_t send_and_receive_data_over_spi_non_blocking (uint8_t *p_bufferSend,
                                                        uint16_t count,
                                                        bool restart);
errorSpi_t restart_non_blocking_spi_send (void);
uint8_t* get_pointer_next_message_non_blocking (errorSpi_t *p_err);
errorSpi_t spi_receive_control (void);
void spi_receive_reset_buffer (void);
void spi_receive_reset (void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_INC_SPI_GENERAL_H_ */
