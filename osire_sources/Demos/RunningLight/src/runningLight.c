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

#include <Feature/ColorCorrection/inc/colorCorrection.h>
#include <Feature/ColorCorrection/inc/colorCorrectionTest.h>
#include <Demos/ColorCorrectionStripe/inc/colorCorrectionStripe.h>
#include <Demos/RunningLight/inc/runningLight.h>
#include <Feature/Init/inc/initFeature.h>
#include <Hal/CY_Gpios/inc/pin.h>
#include <Hal/Osire/inc/osire.h>
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>

#include <Hal/CY_Spi/inc/spiGeneral.h>
#include <Driver/BufferControl/inc/bufferControl.h>
#include <Osp/inc/ospCmdBuffer.h>

#include "nonBlock_spi_timer.h"

#define SIZE_OF_PWM_DATA_ARRAY 6
#define DELAY_MS_10	 10
#define DELAY_MS_100 100
#define DELAY_MS_708 708
#define START_COLOR_TIME 5

/*-------------------------------variables-----------------------------------*/
static runningLightState_t state = RUNNING_LIGHT_INIT;
static runningLightVersion_t colorVersionOld;
static osirePwmData_t dataPwm;
static osirePwmData_t dataPwmReturn;
static uint16_t countLed = 0;
static uint16_t countUp = 1;
static uint16_t countDown = 0;
static bool upWhite = true;
static bool dayMode = false;

static ledPwmCalc_t *p_ledPwmCalc;
static XYZ_t tXYZ; // target color in X, Y, Z coordinates
static pwm_t pwm;

/*-----------------------------Color definitions-----------------------------*/
// color for one LED consists of 3 values (red,green and blue)
// the following definitions result in the colors:
// color[0] = RED
// color[1] = GREEN
// color[2] = BLUE
// color[3] = PURPLE
// color[4] = CYAN
// color[5] = ORANGE
static uint16_t colorRed[7] =
{ 0x1fff, 0x0000, 0x0000, 0x1fff, 0x0000, 0x1fff };
static uint16_t colorGreen[7] =
{ 0x0000, 0x1fff, 0x0000, 0x0000, 0x1fff, 0x1fff };
static uint16_t colorBlue[7] =
{ 0x0000, 0x0000, 0x1fff, 0x1fff, 0x1fff, 0x0000 };

static float cx = 0.33f;
static float cy = 0.33f;
static float brightness = 0; //[mcd]

#define STEPSIZE_NIGHT 4 //[mcd]
#define STEPSIZE_DAY 7 //[mcd]
#define DAY_MIN 500 //[mcd]
#define DAY_MAX 1501 //[mcd]
#define NIGHT_MIN 0 //[mcd]
#define NIGHT_MAX 400 //[mcd]

static uint8_t colorNbr = 0;
static uint8_t colorTime = START_COLOR_TIME; //5 for first start

void set_color(uint8_t color)
{
	if (color == 0)
	{
		dataPwm.data.bit.blue_pwm = colorBlue[colorNbr];
		dataPwm.data.bit.green_pwm = colorGreen[colorNbr];
		dataPwm.data.bit.red_pwm = colorRed[colorNbr];
	}
	else
	{
		cxyY2XYZ(cx, cy, brightness, &tXYZ);
		CCError_t rc = XYZ2pwm(p_ledPwmCalc, &tXYZ, 25, &pwm, dayMode); //138 => 25�C

		if (rc != NO_ERROR)
		{
			//state = ERROR;
			dataPwm.data.bit.blue_pwm = 0;
			dataPwm.data.bit.green_pwm = 0;
			dataPwm.data.bit.red_pwm = 0;
		}
		else
		{
			/*-------------update buffer with new color values---------------*/
			dataPwm.data.bit.blue_pwm = pwm.B;
			dataPwm.data.bit.green_pwm = pwm.G;
			dataPwm.data.bit.red_pwm = pwm.R;

			if (dayMode == true)
			{
				dataPwm.data.bit.blue_curr = 1;
				dataPwm.data.bit.red_curr = 1;
				dataPwm.data.bit.green_curr = 1;
			}
			else
			{
				dataPwm.data.bit.blue_curr = 0;
				dataPwm.data.bit.red_curr = 0;
				dataPwm.data.bit.green_curr = 0;
			}
		}
	}
}

void set_nextColor(runningLightVersion_t color)
{
	uint16_t step = 0, min = 0, max = 0;

	switch (color)
	{
	case RUNNING_LIGHT_COLOR:
		colorNbr++;
		if (colorNbr >= SIZE_OF_PWM_DATA_ARRAY)
		{
			colorNbr = 0;
		}
		break;
	case RUNNING_LIGHT_DIMM_WHITE_FAST_NIGHT:
	case RUNNING_LIGHT_DIMM_WHITE_SLOW_NIGHT:
		step = STEPSIZE_NIGHT;
		min = NIGHT_MIN;
		max = NIGHT_MAX;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_FAST_DAY:
	case RUNNING_LIGHT_DIMM_WHITE_SLOW_DAY:
		step = STEPSIZE_DAY;
		min = DAY_MIN;
		max = DAY_MAX;
		break;
	default:
		break;
	}

//Dimming:
	if (upWhite == true)
	{
		brightness = brightness + step;
	}
	else
	{
		brightness = brightness - step;
	}

	if (brightness >= max)
	{
		upWhite = false;
	}

	if (brightness <= min)
	{
		upWhite = true;
	}

}

void set_version_parameter(runningLightVersion_t colorVersion, uint16_t *p_delayMS, uint16_t *p_delayAtEnd)
{
	switch (colorVersion)
	{
	case RUNNING_LIGHT_COLOR:
		*p_delayMS = DELAY_MS_10;
		*p_delayAtEnd = DELAY_MS_10;
		dayMode = false;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_FAST_NIGHT:
		*p_delayMS = 0;
		*p_delayAtEnd = 0;
		dayMode = false;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_SLOW_NIGHT:
		*p_delayMS = 0;
		*p_delayAtEnd = DELAY_MS_708;
		dayMode = false;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_FAST_DAY:
		*p_delayMS = 0;
		*p_delayAtEnd = 0;
		dayMode = true;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_SLOW_DAY:
		*p_delayMS = 0;
		*p_delayAtEnd = DELAY_MS_708;
		dayMode = true;
		break;
	default:
		break;
	}
}

void reset_running_light_control(void)
{
	state = RUNNING_LIGHT_INIT;
	colorNbr = 0;
	colorTime = START_COLOR_TIME;
	set_led_red(0);
	set_led_green(0);
	countUp = 1;
	countDown = 0;
	upWhite = true;

	brightness = 0;

	for (uint8_t i = 0; i < SIZE_OF_PWM_DATA_ARRAY; i++)
	{
		dataPwm.data.pwmData[i] = 0;
	}
}

void running_light_set_mode_via_uart_helper(runningLightVersion_t newVal)
{
	colorVersionOld = newVal;
	switch (colorVersionOld)
	{
	case RUNNING_LIGHT_COLOR:
		colorNbr = 0;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_FAST_NIGHT:
	case RUNNING_LIGHT_DIMM_WHITE_SLOW_NIGHT:
		brightness = NIGHT_MIN;
		break;
	case RUNNING_LIGHT_DIMM_WHITE_FAST_DAY:
	case RUNNING_LIGHT_DIMM_WHITE_SLOW_DAY:
		brightness = DAY_MIN;
		break;
	default:
		break;
	}
}

enum OSP_ERROR_CODE osp_osire_set_pwm_running_light(uint16_t deviceAddress, osirePwmData_t data, bool restart, uint16_t delay)
{
	ospCmdBuffer_t ospCmd;
	enum OSP_ERROR_CODE ospErrorCode;
	errorSpi_t spiError;

	ospCmd.inCmdId = OSP_OSIRE_SET_PWM;
	ospCmd.inDeviceAddress = deviceAddress;
	ospCmd.p_inParameter = &data.data.pwmData;

	ospErrorCode = osp_cmd_buffer(&ospCmd);

	if (ospErrorCode != OSP_NO_ERROR)
	{
		return ospErrorCode;
	}

	spiError = send_data_over_spi_non_blocking(ospCmd.p_outCmdBuffer, ospCmd.outCmdBufferLength, delay, restart);

	if (spiError != NO_ERROR_SPI)
	{
		return OSP_ERROR_SPI;
	}

	return OSP_NO_ERROR;
}

void running_light_control(runningLightVersion_t colorVersion)
{
	/*-----------------------------init-----------------------------*/
	uint16_t delayMS = 0;
	uint16_t delayAtEnd = 0;

	set_version_parameter(colorVersion, &delayMS, &delayAtEnd);

	if (colorVersionOld != colorVersion)
	{
		for (uint8_t i = 0; i < SIZE_OF_PWM_DATA_ARRAY; i++)
		{
			dataPwm.data.pwmData[i] = 0;
		}

		osp_osire_set_pwm_running_light(0, dataPwmReturn, true, delayMS);
		colorVersionOld = colorVersion;

		switch (colorVersion)
		{
		case RUNNING_LIGHT_COLOR:
			colorNbr = 0;
			break;
		case RUNNING_LIGHT_DIMM_WHITE_FAST_NIGHT:
		case RUNNING_LIGHT_DIMM_WHITE_SLOW_NIGHT:
			brightness = NIGHT_MIN;
			break;
		case RUNNING_LIGHT_DIMM_WHITE_FAST_DAY:
		case RUNNING_LIGHT_DIMM_WHITE_SLOW_DAY:
			brightness = DAY_MIN;
			break;
		default:
			break;
		}

		state = RUNNING_LIGHT_CALCULATE_COLOR;
	}

	enum OSP_ERROR_CODE errorCodeLed;
	switch (state)
	{
	case RUNNING_LIGHT_INIT:

		p_ledPwmCalc = get_addr_ledPwmCalcArray();
		dataPwmReturn.data.bit.blue_pwm = 0;
		dataPwmReturn.data.bit.green_pwm = 0;
		dataPwmReturn.data.bit.red_pwm = 0;

		/*--------------------------Color definitions--------------------------*/
		set_led_green(1);

		initFeatureError_t init = 0;

		init_led_feature_blocking(true, true, &countLed, &init);
		init_feature_init_led_status_xyz_pointers(1, p_ledPwmCalc);

		if (init != INIT_FEATURE_NO_ERROR)
		{
			state = RUNNING_LIGHT_ERROR;
			break;
		}
		else
		{
			state = RUNNING_LIGHT_CALCULATE_COLOR;
		}
		break;

	case RUNNING_LIGHT_CALCULATE_COLOR:
		/*-----------------------calc color (X,Y,Z)----------------------------*/
		// convert (cx, cy, Y) -> (X,Y,Z)
		cxyY2XYZ(cx, cy, brightness, &tXYZ);

		/*-----------------------next case-------------------------------------*/
		set_color(colorVersion);
		osp_osire_set_pwm_running_light(0, dataPwmReturn, true, delayMS);
		set_led_green(0);
		state = RUNNING_LIGHT_LOOP;
		break;

	case RUNNING_LIGHT_ERROR:
		/*-----------------------Error-----------------------------------------*/
		set_led_red(2); //toggle LED
		break;
	case RUNNING_LIGHT_LOOP:
	{
		/*-----------------set different PWM values----------------------------*/
		errorCodeLed = OSP_NO_ERROR;

		if (transfer_delay_complete)
		{
			set_color(colorVersion);

			if (countUp < (countLed + 1))
			{
				while (countUp < (countLed + 1))
				{
					if (countUp == countLed)
					{
						delayMS = delayAtEnd;
					}
					if (transfer_delay_complete)
					{
						errorCodeLed = osp_osire_set_pwm_running_light(countUp, dataPwm, true, delayMS);
						countUp++;
					}
				}

				if (countUp == (countLed + 1))
				{
					set_nextColor(colorVersion);
					countDown = countUp - 1;
					countUp = countLed + 1 + 1;
				}
			}
			else if (countDown >= 1)
			{
				while ((countDown >= 1))
				{
					if (countDown == 1)
					{
						delayMS = delayAtEnd;
					}

					if (transfer_delay_complete)
					{
						errorCodeLed = osp_osire_set_pwm_running_light(countDown, dataPwm, true, delayMS);
						countDown--;
					}
				}

				if (countDown == 0)
				{
					set_nextColor(colorVersion);
					countUp = 1;
					countDown = 0;
				}
			}

		}

		if (errorCodeLed != OSP_NO_ERROR)
		{
			state = RUNNING_LIGHT_ERROR;
			break;
		}

		break;
	}
	default:
		state = RUNNING_LIGHT_ERROR;
		break;
	}

}
