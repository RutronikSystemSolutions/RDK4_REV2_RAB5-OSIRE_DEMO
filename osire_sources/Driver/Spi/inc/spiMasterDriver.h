/*****************************************************************************
 * Copyright 2022 by ams OSRAM AG                                                  *
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

#ifndef DRIVER_SPI_INC_SPI_MASTER_DRIVER_H_
#define DRIVER_SPI_INC_SPI_MASTER_DRIVER_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <Driver/SPI/inc/spiGeneric.h>
#include <stdbool.h>


#define MAX_COUNT_OF_FIFO_WORD 4

typedef enum
{
  SPI_MASTER_DRIVER_NO_ERROR, SPI_MASTER_DRIVER_NULL_POINTER
} spiMasterDriverError_t;

typedef struct
{
  uint8_t countMessageStored;
  uint8_t MaxLengthBuffer;
  uint8_t *p_Buffer;
  uint8_t *p_BufferFilledLength;
  uint8_t *p_MessageLengthBuffer;
  uint32_t *p_delayBuffer;
  bool *p_answerBuffer;
} spiBuffer_t;

void dri_spi_m_init (void);
spiMasterDriverError_t setPointer_SPI_Driver (spiBuffer_t *p_buffer0,
                                              spiBuffer_t *p_buffer1,
                                              spiBuffer_t *p_buffer2);


spiCommandError_t dri_spi_m_restart_with_timer (void);
spiStatusDriver_t dri_spi_m_get_status_master (void);
void dri_spi_m_send_isr (void);
uint8_t dri_spi_m_get_counter_marker_interrupt_change (void);
uint8_t dri_spi_m_get_send_count_spi_driver (void);
spiCommandError_t dri_spi_m_restart_data_send (void);
uint8_t dri_spi_m_get_active_buffer_nbr (void);


#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SPI_INC_SPI_MASTER_DRIVER_H_ */
