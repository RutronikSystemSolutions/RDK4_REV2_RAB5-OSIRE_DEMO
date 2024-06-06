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

/*
 * timer.c
 *
 *  @date 06.07.2023
 *  @auhtor A.Heder
 *
 *  SPI TIMER:
 *  we need here an Timer which has an resolution of 1us and a dynamic change. --> So we use pdl_timer_counter --> configured in device configurator
 *
 *  Base time clk: 1MHz Freq
 *  Period: 1 = 1us ... 1000 = 1000us
 *
 */

#include <stdbool.h>
#include <sys/_stdint.h>
#include "../../CY_Timer/inc/timer.h"

//static ftm_state_t ftmStateStruct_SW_timer;

void CY_PDL_Timer_SPIsend_init(void)
    {
//TODO: init timer function
    }

//void hal_init_timer_SPI_send (const isr_t newHandler)
//{
//  LPTMR_DRV_Init (INST_TIMER_SPI_SEND, &timer_spi_send_config0, false);
//  INT_SYS_InstallHandler (LPTMR0_IRQn, newHandler, (isr_t*) 0);
//  INT_SYS_EnableIRQ (LPTMR0_IRQn);
//}



void hal_clearISR_timer_SPI_send (void)
{
  //LPTMR_DRV_ClearCompareFlag (INST_TIMER_SPI_SEND);
}

//void hal_clearISR_SW_timer (void)
//{
//  FTM_DRV_ClearStatusFlags (INST_SW_TIMER, (uint32_t) FTM_TIME_OVER_FLOW_FLAG);
//}

void hal_set_new_timer_val_spi_tick (uint16_t tick)
{
//  LPTMR_DRV_SetCompareValueByCount (INST_TIMER_SPI_SEND, tick);

}

void hal_start_spi_timer (void)
{
//  LPTMR_DRV_StartCounter (INST_TIMER_SPI_SEND);
}

void hal_stop_spi_timer (void)
{
//  LPTMR_DRV_StopCounter (INST_TIMER_SPI_SEND);
}

