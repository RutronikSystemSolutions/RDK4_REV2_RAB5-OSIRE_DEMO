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

#ifndef HAL_TIMER_INC_TIMER_H_
#define HAL_TIMER_INC_TIMER_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include "../../CY_System/inc/initSystem.h"
#include <stdint.h>
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"


typedef void (*p_fkt) (void);
//void hal_init_timer_SPI_send (const isr_t newHandler);
//void hal_init_timer_SW_timer (const isr_t newHandler);
void hal_clearISR_timer_SPI_send (void);
void hal_clearISR_SW_timer (void);
void hal_set_new_timer_val_spi_tick (uint16_t tick);
void hal_start_spi_timer (void);
void hal_stop_spi_timer (void);

void CY_PDL_Timer_SPIsend_init(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_TIMER_INC_TIMER_H_ */
