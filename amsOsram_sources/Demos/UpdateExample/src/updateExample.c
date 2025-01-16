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

#include <Demos/UpdateExample/inc/updateExample.h>

#include <Feature/ColorCorrection/inc/colorCorrection.h>
#include <Feature/Init/inc/initFeature.h>
#include <Hal/CY_Gpios/inc/pin.h>
#include <Hal/Osire/inc/osire.h>
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>
#include <string.h>
#include <SwTimer/inc/swTimer.h>
#include <Crc/inc/crc.h>
#include <rdk4_system/sys_timer.h>

#include <Hal/CY_Spi/inc/spiGeneral.h>
#include <Driver/BufferControl/inc/bufferControl.h>
#include <Osp/inc/ospCmdBuffer.h>
#include <rdk4_system/nonBlock_spi_timer.h>

#define UPDATE_TIME_SLOT_MS 10
#define MAX_USED_LEDS 100

#define OFFSET_TEMP_LED 113
#define TEMP_INIT 25
#define TIME_OUT_READ_TEMPSTATUS_MS 5

#define ERROR_BLINKING_MS 200

#define DEFAULT_COLOR_CX 0.312f
#define DEFAULT_COLOR_CY 0.329f
#define DEFAULT_BRIGHTNESS_MCD 500.0f

#define MAX_RETRY_CORRUPT_DATA 3

static updateExampleStates_t updateExampleState = UE_INIT;
static uint16_t countLed = 0;
static initFeatureError_t intError = INIT_FEATURE_NO_ERROR;
volatile static uint32_t *p_globalTime = 0; //do only read this value!
static uint32_t time = 0;
static int8_t temp[MAX_USED_LEDS + 1];
static ledPwmCalc_t *p_ledPwmCalcArray;
static enum OSP_ERROR_CODE ospErrorCode;
static int16_t ledActive = 0;
static int16_t ledNbrTempRead = 1;
static bool timerOverrun = false;
static osireTempStatus_t tempStatusData;
static uint32_t errorTimer = 0;
static uint32_t timeOutRead = 0;

/*-----------------------------Color definitions-----------------------------*/
static bool dayMode = false; //night mode is used
static float colorCx = DEFAULT_COLOR_CX;
static float colorCy = DEFAULT_COLOR_CY;
static float colorMcd = 500.0f;
static XYZ_t xyz; // target color in X, Y, Z coordinates

void reset_update_example (void)
{
  update_example_init ();
  set_led_red (0);
  colorCx = DEFAULT_COLOR_CX;
  colorCy = DEFAULT_COLOR_CY;
  colorMcd = DEFAULT_BRIGHTNESS_MCD;
}

void update_example_init (void)
{
  updateExampleState = UE_INIT;
  countLed = 0;
  intError = INIT_FEATURE_NO_ERROR;
  p_globalTime = get_system_time_pointer_ms();
  p_ledPwmCalcArray = get_addr_ledPwmCalcArray ();

  time = 0;
  ledActive = 0;
  ledNbrTempRead = 1;
  timerOverrun = false;

  for (uint8_t i = 0; i < MAX_USED_LEDS; i++)
    {
      temp[i] = TEMP_INIT;
    }
}

void change_color_update_example (float cx, float cy, float mcd, bool dayModeSetting)
{
  colorCx = cx;
  colorCy = cy;
  if(dayModeSetting)
  {
	  colorMcd = mcd * 12;
  }
  else
  {
	  colorMcd = mcd;
  }
  dayMode = dayModeSetting;
}

enum OSP_ERROR_CODE osp_osire_set_pwm_non_blocking (uint16_t deviceAddress,
                                                    osirePwmData_t data,
                                                    bool restart,
                                                    uint16_t delay)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_OSIRE_SET_PWM;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.pwmData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);

  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_data_over_spi_non_blocking (ospCmd.p_outCmdBuffer,
                                              ospCmd.outCmdBufferLength, delay,
                                              restart);

  spiError = NO_ERROR_SPI;
  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  return OSP_NO_ERROR;
}

enum OSP_ERROR_CODE osp_osire_set_pwm_and_sr_non_blocking (
    uint16_t deviceAddress, osirePwmData_t data, osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_SET_PWM_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.pwmData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_non_blocking (
      ospCmd.p_outCmdBuffer, ospCmd.outCmdBufferLength, 1);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  return OSP_NO_ERROR;
}

enum OSP_ERROR_CODE get_temp_status (osireTempStatus_t *p_rsp,
                                     uint8_t *p_rspBuffer)
{
  if (crc (p_rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = p_rspBuffer[4 - i];
    }

  p_rsp->address = ((p_rspBuffer[0] & 0x0F) << 6)
      | ((p_rspBuffer[1] >> 2) & 0x3F);

  return OSP_NO_ERROR;
}

static volatile bool errorInit = false;
static volatile uint8_t errorCounterCorruptedData = 0;

void update_example_control (void)
{

  switch (updateExampleState)
    {
    case UE_INIT:

      update_example_init ();

      init_led_feature_blocking (true, true, &countLed, &intError);
      errorInit = init_feature_init_led_status_xyz_pointers (countLed,p_ledPwmCalcArray);

      if (intError != INIT_FEATURE_NO_ERROR)
        {
          updateExampleState = UE_ERROR;
          break;
        }
      else
        {
          time = *p_globalTime + UPDATE_TIME_SLOT_MS;

          if (time < (*p_globalTime))
            {
              timerOverrun = true;
            }

          updateExampleState = UE_CALC;
        }

      osp_go_active(0);
      break;
    case UE_CALC:

      errorCounterCorruptedData = 0; //we start a new 10ms frame

      /*----------------------calc color based on temp---------------------*/
      cxyY2XYZ (colorCx, colorCy, colorMcd, &xyz);
      pwm_t pwmTemp;
      CCError_t rc;

      for (uint8_t i = 0; i <= countLed; i++)
        {

          rc = XYZ2pwm (p_ledPwmCalcArray[i].XYZTyp, &xyz, ((float) temp[i]), &pwmTemp, dayMode);

          if (i > countLed)
            {
              rc = NO_ERROR; //if we use less than MAX_USED_LEDS than we are getting wrong calcs here...
              //but we still need the calcs for the timing demo
            }

          if (rc != NO_ERROR) //first approach: hard break!
            {
              updateExampleState = UE_ERROR;
              break;
            }

          p_ledPwmCalcArray[i].regPwm.data.bit.blue_pwm = pwmTemp.B;
          p_ledPwmCalcArray[i].regPwm.data.bit.green_pwm = pwmTemp.G;
          p_ledPwmCalcArray[i].regPwm.data.bit.red_pwm = pwmTemp.R;

          if (dayMode == false)
            {
              p_ledPwmCalcArray[i].regPwm.data.bit.blue_curr = 0;
              p_ledPwmCalcArray[i].regPwm.data.bit.green_curr = 0;
              p_ledPwmCalcArray[i].regPwm.data.bit.red_curr = 0;
            }
          else
            {
              p_ledPwmCalcArray[i].regPwm.data.bit.blue_curr = 1;
              p_ledPwmCalcArray[i].regPwm.data.bit.green_curr = 1;
              p_ledPwmCalcArray[i].regPwm.data.bit.red_curr = 1;
            }
        }

      if (updateExampleState != UE_ERROR)
        {
          updateExampleState = UE_SEND_DATA;
          ledActive = countLed;
        }

      break;

    case UE_SEND_DATA:
      {
        //Send all data out! (LED Address which is not used will also be send out but has no influence on the LED Stripe in BiDir)
        for (; ledActive > 0; ledActive--)
          {
        	//TODO: implement non-bloking transmit and receive state machine
            //ospErrorCode = osp_osire_set_pwm_non_blocking (ledActive, p_ledPwmCalcArray[ledActive - 1].regPwm, 0, 0);
            ospErrorCode =  osp_osire_set_pwm (ledActive, p_ledPwmCalcArray[ledActive - 1].regPwm);
          }

        if (ledActive <= 1)
          {
            updateExampleState = UE_READ_OUT; //we have everything in the buffer...
          }

        break;
      }
    case UE_READ_OUT:
      {
      	if(transfer_delay_complete)
      	{
      		ospErrorCode = osp_osire_read_tempstatus (ledNbrTempRead, &tempStatusData);

      		if (ospErrorCode != OSP_NO_ERROR)
      		{
      			/*Check once more*/
              	ospErrorCode = osp_osire_read_tempstatus (ledNbrTempRead, &tempStatusData);
              	if (ospErrorCode != OSP_NO_ERROR)
              	{
              		updateExampleState = UE_ERROR;
              		set_led_blue (2);
              		break;
              	}
      		}

              temp[ledNbrTempRead - 1] = (int8_t) ((int16_t) (tempStatusData.data.byte.Temp - (int16_t) OFFSET_TEMP_LED));
              //temp starts at 0,  ledNbrTempRead starts with 1 -> -1 because of offset

              ledNbrTempRead++;

              if ((ledNbrTempRead > countLed) || (ledNbrTempRead > MAX_USED_LEDS))
                {
                  ledNbrTempRead = 1;
                }

              if (ospErrorCode == OSP_NO_ERROR)
                {
              	updateExampleState = UE_WAIT;
                  timeOutRead = (*p_globalTime);
                }
      	}
        break;
      }
    case UE_WAIT_FOR_DATA:
      {
        break;
      }
    case UE_WAIT:

      if (timerOverrun == true)
        {
          if (time >= (*p_globalTime))
            {
              updateExampleState = UE_CALC;

              time = *p_globalTime + UPDATE_TIME_SLOT_MS;

              if (time < (*p_globalTime))
                {
                  timerOverrun = true;
                }
              else
                {
                  timerOverrun = false;
                }
            }
        }
      else
        {
          if (time <= (*p_globalTime))
            {
              updateExampleState = UE_CALC;

              time = *p_globalTime + UPDATE_TIME_SLOT_MS;

              if (time < (*p_globalTime))
                {
                  timerOverrun = true;
                }
              else
                {
                  timerOverrun = false;
                }
            }
        }

      break;
    case UE_ERROR:
      /*-----------------------Error-----------------------------------------*/
      if (errorTimer < (*p_globalTime))
        {
          set_led_red (2); //toggle LED
          errorTimer = *p_globalTime + ERROR_BLINKING_MS;
        }

      break;
    default:
      updateExampleState = UE_ERROR;
      break;
    }

}
