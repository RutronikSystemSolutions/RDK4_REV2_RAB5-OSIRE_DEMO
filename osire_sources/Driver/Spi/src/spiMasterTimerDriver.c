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

#include <CY_Timer/inc/timer.h>
#include <Driver/SPI/inc/spiMasterTimerDriver.h>
#include <Driver/BufferControl/inc/bufferControl.h>
#include "Driver/SPI/inc/spiMasterDriver.h"

#define SYSTEM_EXTRA_TIME_SPI 8
#define SYSTEM_EXTRA_TIME_TIMER 8
#define SYSTEM_EXTRA_TIME_WORKAROUND 3

static uint8_t flagTimeDelay;
static uint32_t timeComplete;
static uint32_t timeLeft;
static uint32_t timeActual;
static uint16_t timeTick;

void init_spi_timer (void)
{
  hal_set_new_timer_val_spi_tick (1000);
}

void lptmr_SPI_Send_ISR (void)
{
  hal_clearISR_timer_SPI_send ();

  timeActual += timeTick;

  if (timeActual >= timeComplete)
    {
      hal_stop_spi_timer ();
      flagTimeDelay = 0;
      dri_spi_m_restart_with_timer ();
      timeTick = 0;
    }
  else
    {
      if (timeLeft > 65500)
        {
          timeTick = 65500; //131ms
          timeLeft = timeLeft - timeTick;
        }
      else
        {
          timeTick = timeLeft;

          hal_stop_spi_timer ();
          hal_set_new_timer_val_spi_tick (timeTick);
          hal_start_spi_timer ();
        }
    }
}

uint8_t check_flag_time_delay (void)
{
  return (flagTimeDelay);
}

p_fkt get_spi_timer_intHandler (void)
{
  return (lptmr_SPI_Send_ISR);
}

void start_delay_timer (uint16_t time) //delay in 100�s
{
  flagTimeDelay = 1;

  timeActual = 0;
  timeComplete = ((uint32_t) time) * 50; //500 tick per ms! //50 tick per 100�s

  if (timeComplete > 65500)
    {
      timeTick = 65500; //131ms
      timeLeft = timeComplete - timeTick;
    }
  else
    {
      timeTick = timeComplete;
    }

  hal_set_new_timer_val_spi_tick (timeTick);
  hal_start_spi_timer ();
}
