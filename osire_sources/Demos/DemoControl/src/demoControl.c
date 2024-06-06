/*
 * demoControl.c
 *
 *  Created on: 07.06.2023
 *      Author: EDE
 */

#include "demoControl.h"
#include <Hal/Button/inc/button.h>
#include <CY_Gpios/inc/pin.h>
#include <Demos/ColorCorrectionStripe/inc/colorCorrectionStripe.h>
#include <Demos/DemoControl/inc/demoControl.h>
#include <Demos/MinimalRgbStripe/inc/minimalRgbStripe.h>
#include <Demos/RunningLight/inc/runningLight.h>
#include <Feature/ColorCorrection/inc/ledPwmCalc.h>
#include <StoreOtp/inc/storeXyzInFlash.h>
#include <SwTimer/inc/swTimer.h>
#include <Hal/CY_Spi/inc/spiGeneral.h>

/*****************************************************************************/
/*****************************************************************************/
static volatile runningLightVersion_t runningLightVersion = 0;
static demoState_t state = MINIMAL_RGBI;
static uint8_t colorCorrectionModeWithTemp = 1;
/*****************************************************************************/
/*****************************************************************************/
void reset_modi (void)
{
  reset_button_one_time_pressed ();
  reset_ledPwmCalcArray ();
  reset_minimal_rgbi_control ();
  reset_color_correction ();
  reset_store_xyz_to_flash ();
  reset_running_light_control ();
  set_led_green (0);
  set_led_red (0);
  set_led_blue (0);
  runningLightVersion = 0;
  spi_receive_reset_buffer ();
}

/*****************************************************************************/
/*****************************************************************************/
void start_uart_mode (void)
{
  reset_modi ();
  state = UART_MODE;
}

/*****************************************************************************/
/*****************************************************************************/
void start_minimal_rbgi_mode (void)
{
  reset_modi ();
  state = MINIMAL_RGBI;
}

/*****************************************************************************/
/*****************************************************************************/
void start_colorCorrection_mode (void)
{
  reset_modi ();
  colorCorrectionModeWithTemp = 1;
  state = COLOR_CORRECTION;
}
/*****************************************************************************/
/*****************************************************************************/
bool start_runningLight_mode (uint8_t version)
{
  bool returnVal = false;

  if (((runningLightVersion_t) version) < RUNNING_LIGHT_VERSION_END_MARKER)
    {
      reset_modi ();
      state = RUNNING_LIGHT;
      runningLightVersion = (runningLightVersion_t) version;
      running_light_set_mode_via_uart_helper (runningLightVersion); //we make a "reset" so we need to set this to the "new" version
    }
  else
    {
      returnVal = true;
    }

  return (returnVal);
}

/*****************************************************************************/
/*****************************************************************************/
void demo_control (void)
{
  /*-----------------------------check buttons-----------------------------*/

  if (one_time_button_pressed_sw1 () == 1)
    {
      reset_modi ();

      state++;
      if (state > UPDATE_RATE_EXAMPLE)
        {
          state = MINIMAL_RGBI;
        }
    }

  /*------------------------end button check---------------------------------*/

  switch (state)
    {
    case MINIMAL_RGBI:
      if (check_flag_100ms ())
        {
          reset_flag_100ms ();
          /*-----------------------------Minimal RGBi demo-------------------*/
          minimal_rgb_stripe_control ();
        }
      break;
    case COLOR_CORRECTION:
      /*-----------------------------Color correction demo-------------------*/
      if (one_time_button_pressed_sw3 () == true)
        {
          if (colorCorrectionModeWithTemp)
            {
              colorCorrectionModeWithTemp = 0;
            }
          else
            {
              colorCorrectionModeWithTemp = 1;
            }
        }
      color_Correction_Stripe_Control (colorCorrectionModeWithTemp);
      break;
    case XYZ_TO_FLASH:
      /*------------------------Store xyz of OTP to Flash--------------------*/
      if (check_flag_100ms ())
        {
          reset_flag_100ms ();
          save_xyz_in_flash (one_time_button_pressed_sw3 ());
        }
      break;
    case UART_MODE:
      if (check_flag_100ms ())
        {
          reset_flag_100ms ();
          set_led_green (2); //Toggle green LED
        }
      break;
    case RUNNING_LIGHT:
      if (one_time_button_pressed_sw3 () == true)
        {
          runningLightVersion++;
          if (runningLightVersion >= RUNNING_LIGHT_VERSION_END_MARKER)
            {
              runningLightVersion = 0;
            }
        }
      running_light_control (runningLightVersion);

      break;
    default:
      state = MINIMAL_RGBI;
      break;
    }

}
