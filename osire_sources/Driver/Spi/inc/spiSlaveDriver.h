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

#ifndef DRIVER_SPI_INC_SPI_MASTER_DRIVER_H_
#define DRIVER_SPI_INC_SPI_MASTER_DRIVER_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <Driver/Spi/inc/spiGeneric.h>

#define MAX_COUNT_OF_FIFO_WORD 4
#define SIZE_OF_SPI_RECEIVE_BUFFER 33

void dri_spi_s_set_tcr (tcr_t *p_tcrMaster);
void dri_spi_s_set_interrupt (spiInterruptRegister_t *p_intMaster);
void dri_spi_s_set_configuration_register_1 (spiConfig1Register_t *p_configReg1);
void dri_spi_s_init (void);
void dri_spi_s_receive_isr (void);
spiStatusDriver_t dri_spi_s_get_status_slave (void);
uint16_t dri_spi_s_get_receive_count (void);
uint16_t dri_spi_s_get_remaining_count (void);
int8_t dri_spi_s_get_hw_fifo_count (void);

uint8_t* get_buffer (void);
uint32_t* get_buffer_time_stamp (void);
void reset_buffer (void);
void reset_buffer2 (void);

uint8_t get_buffer_position (void);
uint8_t get_max_size_buffer (void);
void dri_spi_s_reset_buffer (void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SPI_INC_SPI_MASTER_DRIVER_H_ */
