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

#include <CY_Gpios/inc/pin.h>
#include <Demos/MinimalRgbStripe/inc/minimalRgbStripe.h>
#include <Feature/Init/inc/initFeature.h>
#include <Hal/Osire/inc/osire.h>
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>

#define COLOR_TIME_START_VAL 5
#define COLOR_COUNT_MAX 7

/*-------------------------------variables-----------------------------------*/
static minRgbState_t state = MIN_RGB_INIT;
static osirePwmData_t dataPwm;
//static osirePwmData_t dataPwmReturn;
static uint16_t countLed;

/*-----------------------------Color definitions-----------------------------*/
// color for one LED consists of 3 values (red,green and blue)
// the following definitions result in the colors:
// color[0] = RED
// color[1] = GREEN
// color[2] = BLUE
// color[3] = PURPLE
// color[4] = CYAN
// color[5] = ORANGE
// color[6] = WHITE
static uint16_t colorRed[COLOR_COUNT_MAX] =
  { 0x0fff, 0x0000, 0x0000, 0x0fff, 0x0000, 0x0fff, 0x0fff };
static uint16_t colorGreen[COLOR_COUNT_MAX] =
  { 0x0000, 0x0fff, 0x0000, 0x0000, 0x0fff, 0x0fff, 0x0fff };
static uint16_t colorBlue[COLOR_COUNT_MAX] =
  { 0x0000, 0x0000, 0x0fff, 0x0fff, 0x0fff, 0x0000, 0x0fff };

static uint8_t colorNbr = 0;
static uint8_t colorTime = COLOR_TIME_START_VAL; //5 for first start

void reset_minimal_rgbi_control (void)
{
  state = MIN_RGB_INIT;
  colorNbr = 0;
  colorTime = COLOR_TIME_START_VAL;
  set_led_red (0);
  set_led_green (0);
}

/*-----------------------------minimal RGBi demo-----------------------------*/
void minimal_rgb_stripe_control (void)
{
  /*-----------------------------init-----------------------------*/

  dataPwm.data.bit.blue_curr = 0;
  dataPwm.data.bit.green_curr = 0;
  dataPwm.data.bit.red_curr = 0;

  enum OSP_ERROR_CODE errorCodeLed;
  switch (state)
    {
    case MIN_RGB_INIT:

      /*--------------------------Color definitions--------------------------*/
      set_led_green (1);

      initFeatureError_t init = 0;
      countLed = 0;
      init_led_feature_blocking (true, false, &countLed, &init); //init-BiDir

      if (init != INIT_FEATURE_NO_ERROR)
        {
          state = MIN_RGB_ERROR;
          break;
        }
      else
        {
          state = MIN_RGB_LOOP;
        }

      set_led_green (0);
      break;
    case MIN_RGB_ERROR:
      /*-----------------------Error-----------------------------------------*/
      set_led_red (2); //toggle LED
      break;
    case MIN_RGB_LOOP:
      /*-----------------set different PWM values----------------------------*/
      if (colorTime >= COLOR_TIME_START_VAL)
        {
          colorTime = 0;
          dataPwm.data.bit.blue_pwm = colorBlue[colorNbr];
          dataPwm.data.bit.green_pwm = colorGreen[colorNbr];
          dataPwm.data.bit.red_pwm = colorRed[colorNbr];

          colorNbr++;
          if (colorNbr >= COLOR_COUNT_MAX)
            colorNbr = 0;

          errorCodeLed = osp_osire_set_pwm (0, dataPwm);
          if (errorCodeLed != OSP_NO_ERROR)
            {
              state = MIN_RGB_ERROR;
              break;
            }
        }

      colorTime++;

      break;
    default:
      state = MIN_RGB_ERROR;
      break;
    }

}
