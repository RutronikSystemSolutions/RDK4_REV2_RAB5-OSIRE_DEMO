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

#include <Feature/ColorCorrection/inc/ledPwmCalc.h>
#include <Feature/Init/inc/initFeature.h>
#include <Hal/CY_Flash_EEPROM/inc/flash.h>
#include <Hal/CY_Gpios/inc/pin.h>
#include <Hal/Osire/inc/osire.h>
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>
#include <StoreOtp/inc/storeXyzInFlash.h>

#define COLOR_TIME_MAX  5

static uint8_t colorTime = COLOR_TIME_MAX;
static uint16_t countLed = 0;
static store_XYZ_state_t state = STORE_XYZ_INIT;

void reset_store_xyz_to_flash (void)
{
  colorTime = COLOR_TIME_MAX;
  set_led_red (0);
  set_led_green (0);
  set_led_blue (0);
  state = STORE_XYZ_INIT;
  countLed = 0;
}

void save_xyz_in_flash (uint8_t button)
{

  enum OSP_ERROR_CODE errorCodeLed;

  switch (state)
    {
    case STORE_XYZ_INIT:
      {
        //------------LED RESET+ INIT-----//
        initFeatureError_t init = 0;
        init_led_feature_blocking (false, false, &countLed, &init);

        if (init != INIT_FEATURE_NO_ERROR)
          {
            state = STORE_XYZ_ERROR;
            break;
          }
        else
          {
            state = STORE_XYZ_WAIT_FOR_SW;
          }
        break;
      }
    case STORE_XYZ_WAIT_FOR_SW:

      if (colorTime >= COLOR_TIME_MAX)
        {
          colorTime = 0;
          set_led_red (2);
          set_led_green (2);
        }
      colorTime++;
      if (button == 1)
        {
          state = STORE_XYZ_SW_PUSHED;
        }
      break;

    case STORE_XYZ_SW_PUSHED:
      set_led_red (0);
      set_led_green (0);
      set_led_blue (1);

      //------------Store XYZ values to Flash------------//
      errorCodeLed = init_feature_otp_to_flash (countLed);
      if (errorCodeLed != OSP_NO_ERROR)
        {
          state = STORE_XYZ_ERROR;
        }
      else
        {
          state = STORE_XYZ_FINISH;
        }
      set_led_blue (0);
      break;

    case STORE_XYZ_FINISH:
      if (colorTime >= COLOR_TIME_MAX)
        {
          colorTime = 0;
          set_led_green (2);
        }
      colorTime++;
      break;

    case STORE_XYZ_ERROR:
      set_led_red (2);
      break;
    }

}
