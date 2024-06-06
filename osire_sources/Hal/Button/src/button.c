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

#include <Hal/Button/inc/button.h>
#include <Hal/CY_Gpios/inc/pin.h>
#include "capsense_buttons.h"
#include "button_timer.h"

/*****************************************************************************/
/*****************************************************************************/
static volatile uint8_t sw1;
static volatile uint8_t sw3;

static volatile uint8_t sw1Old;
static volatile uint8_t sw3Old;

static volatile uint8_t buttonSw3Toggle;
static volatile uint8_t buttonSw1Toggle;

static volatile uint8_t oneTimePressedSw3;
static volatile uint8_t oneTimePressedSw1;

/*****************************************************************************/
/*****************************************************************************/
void button_polling (void)
{
  /* Process touch input */
  process_touch();

  sw3 = cbuttons.csb3_status;
  sw1 = cbuttons.csb1_status;

  if ((sw3 == 0x01) && (sw3Old == 0x00))
    {
      oneTimePressedSw3 = 1;
      if (buttonSw3Toggle == 0)
        {
          buttonSw3Toggle = 1;
        }
      else
        {
          buttonSw3Toggle = 0;
        }
    }

  if ((sw1 == 0x01) && (sw1Old == 0x00))
    {
      oneTimePressedSw1 = 1;
      if (buttonSw1Toggle == 0)
        {
          buttonSw1Toggle = 1;
        }
      else
        {
          buttonSw1Toggle = 0;
        }
    }
  /*---------------------set button values for check-------------------------*/
  sw1Old = sw1;
  sw3Old = sw3;
}

/*****************************************************************************/

/*****************************************************************************/
void reset_button_one_time_pressed (void)
{
  oneTimePressedSw1 = 0;
  oneTimePressedSw3 = 0;
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t get_button_toggle_val_sw3 (void)
{
  return (buttonSw3Toggle);
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t get_button_toggle_val_sw1 (void)
{
  return (buttonSw1Toggle);
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t one_time_button_pressed_sw3 (void)
{
  uint8_t val = oneTimePressedSw3;
  oneTimePressedSw3 = 0;

  if(val)
  {
	  if(button_ind_complete)
	  {
		    /*Turn ON LEDs here*/
		    set_led_red (1);
		    set_led_green (1);
		    set_led_blue (1);
		    button_ind_timer_start (BUTTON_IND_HOLD_MS);
	  }
  }

  return (val);
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t one_time_button_pressed_sw1 (void)
{
  uint8_t val = oneTimePressedSw1;
  oneTimePressedSw1 = 0;

  if(val)
  {
	  if(button_ind_complete)
	  {
		    /*Turn ON LEDs here*/
		    set_led_red (1);
		    set_led_green (1);
		    set_led_blue (1);
		    button_ind_timer_start (BUTTON_IND_HOLD_MS);
	  }
  }

  return (val);
}
