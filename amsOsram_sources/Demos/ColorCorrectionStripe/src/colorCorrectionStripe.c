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

#include <amsOsram_sources/Demos/ColorCorrectionStripe/inc/colorCorrectionStripe.h>
#include <amsOsram_sources/Feature/ColorCorrection/inc/colorCorrection.h>
#include <amsOsram_sources/Feature/ColorCorrection/inc/colorCorrectionTest.h>
#include <amsOsram_sources/Feature/Init/inc/initFeature.h>
#include <amsOsram_sources/Hal/CY_Gpios/inc/pin.h>
#include <amsOsram_sources/Hal/Osire/inc/osire.h>
#include <amsOsram_sources/Osp/inc/genericDevice.h>
#include <amsOsram_sources/Osp/inc/osireDevice.h>
#include <amsOsram_sources/SwTimer/inc/swTimer.h>
#include <string.h>

/*-----------------------------Colour definitions-----------------------------*/
#define COLOR_CX 0.3221f
#define COLOR_CY 0.3316f
#define COLOR_MCD 1500.0f
#define DEFAULT_PWM_VALUE 0x7fff
static bool dayMode = true;

/*--------------------------Temperature definitions--------------------------*/
#define OFFSET_TEMP_LED 113
#define TEMP_WITHOUT_CORRECTION 25

/*-----------------------------------Defines for maximum---------------------*/
#define MAX_COUNT_LED_FOR_DEMO 40

/*--------------------------Global variables in File-------------------------*/
static colorCorrectionStripeState_t state = INIT;
static enum OSP_ERROR_CODE errorCodeLed;
static osirePwmData_t dataPwm;
static uint8_t temperature = 0;
static float tempFloat = 0;
static uint16_t countLed = 0;
static uint16_t ledActive = 0;
static uint8_t initColor = 0;
static pwm_t pwm;

static ledPwmCalc_t *p_ledPwmCalcArray;
static XYZ_t tXYZ; // target color in X, Y, Z coordinates

static osireTempStatus_t tempStatus;

void reset_color_correction (void)
{
  state = INIT;
  initColor = 0;
  set_led_blue (0);
  set_led_red (0);
}

/*--------------------Color Correction Control function----------------------*/
void color_Correction_Stripe_Control (uint8_t withCorrection)
{
  switch (state)
    {
    case INIT:
      /*-----------------------set default values----------------------------*/

      p_ledPwmCalcArray = get_addr_ledPwmCalcArray ();

      dataPwm.data.bit.blue_curr = 0;
      dataPwm.data.bit.green_curr = 0;
      dataPwm.data.bit.red_curr = 0;
      dataPwm.data.bit.blue_pwm = DEFAULT_PWM_VALUE;
      dataPwm.data.bit.green_pwm = DEFAULT_PWM_VALUE;
      dataPwm.data.bit.red_pwm = DEFAULT_PWM_VALUE;

      initFeatureError_t init;
      init_led_feature_blocking (true, true, &countLed, &init);

      if (init == INIT_FEATURE_ERROR)
        {
          state = ERROR;
        }
      else
        {
          init_feature_init_led_status_xyz_pointers (countLed, p_ledPwmCalcArray);
          state = TARGET_COLOR_DEFINITION;
        }

      break;

    case TARGET_COLOR_DEFINITION:
      /*-----------------------calc color (X,Y,Z)----------------------------*/
      cxyY2XYZ (COLOR_CX, COLOR_CY, COLOR_MCD, &tXYZ);

      /*-----------------------next case-------------------------------------*/
      ledActive = 0; //start with the first led!
      state = GET_TEMP;
      break;

    case GET_TEMP:
      /*-----------------------read LED temp---------------------------------*/
      errorCodeLed = osp_osire_read_tempstatus (ledActive + 1, &tempStatus);

      if (errorCodeLed != OSP_NO_ERROR)
        {
    	  /*Check once more*/
    	  errorCodeLed = osp_osire_read_tempstatus (ledActive + 1, &tempStatus);
    	  if(errorCodeLed != OSP_NO_ERROR)
    	  {
    		  state = ERROR;
    	  }
        }
      else
        {
          state = CALC_COLOR;

          osireStatus_t status;
          status.data.status = tempStatus.data.byte.Status;

          if (status.data.bit.state != 2)
            {
              set_led_blue (1);
            }

          /*-----------------------calc temp---------------------------------*/
          if (withCorrection != 0)
            {
              osireTemp_t temp;
              temp.data.temp_value = tempStatus.data.byte.Temp;
              temperature = temp.data.temp_value;
            }
          else
            {
              temperature = OFFSET_TEMP_LED + TEMP_WITHOUT_CORRECTION;
            }

          tempFloat = (float) (temperature - OFFSET_TEMP_LED);
        }

      break;
    case CALC_COLOR:
      {
        /*----------------------calc color based on temp---------------------*/
        CCError_t rc = XYZ2pwm (&p_ledPwmCalcArray[ledActive], &tXYZ, tempFloat, &pwm, dayMode);

        if (rc != NO_ERROR)
          {
            state = ERROR;
          }
        else
          {
            /*-------------update buffer with new color values---------------*/
            state = SET_COLOR;
            dataPwm.data.bit.blue_pwm = pwm.B;
            dataPwm.data.bit.green_pwm = pwm.G;
            dataPwm.data.bit.red_pwm = pwm.R;

            dataPwm.data.bit.blue_curr = p_ledPwmCalcArray[ledActive].regPwm.data.bit.blue_curr;
            dataPwm.data.bit.red_curr = p_ledPwmCalcArray[ledActive].regPwm.data.bit.red_curr;
            dataPwm.data.bit.green_curr = p_ledPwmCalcArray[ledActive].regPwm.data.bit.green_curr;
          }

        break;
      }
    case SET_COLOR:
      {
        uint8_t check = 0;
        for (uint8_t i = 0; i < 6; i++)
          {
            if (dataPwm.data.pwmData[i] != 0)
              {
                check++;
              }
          }

        if (check == 0)
          {
            set_led_blue (1);
          }

        /*-----------------------set new color---------------------------------*/
        if (initColor == 0)
          {
            /*-------if color was not set yet once: set color for all----------*/
            errorCodeLed = osp_osire_set_pwm (0, dataPwm); //broadcast
            initColor = 1;
          }
        else
          {
            /*-----------------------set new color for single LED--------------*/
            errorCodeLed = osp_osire_set_pwm (ledActive + 1, dataPwm);
          }

        if (errorCodeLed != OSP_NO_ERROR)
          {
            state = ERROR;
          }
        else
          {
            state = GET_TEMP;
          }

        ledActive++; //go to the next LED

        /*-----------------------check if last LED-----------------------------*/
        if (ledActive >= countLed)
          {
            ledActive = 0;
          }

        break;
      }
    case ERROR:
      /*-----------------------Error-----------------------------------------*/
      if (check_flag_100ms ())
        {
    	  //reset_flag_100ms ();
    	  //set_led_red (2); //toggle LED
          //state = TARGET_COLOR_DEFINITION;
    	  state = GET_TEMP;
        }
      break;
    default:
      state = ERROR;
      break;

    }
}
