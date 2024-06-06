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

#include <stdio.h>
#include <stdlib.h>
#include "cy_pdl.h"
#include "cybsp.h"
#include <Feature/Init/inc/initFeature.h>
#include <Osp/inc/osireDevice.h>
#include <Hal/Osire/inc/osire.h>
#include <CY_Flash_EEPROM/inc/flash.h>

typedef enum
    {
    LED_INIT_INIT,
    LED_INIT_RESET,
    LED_INIT_BIDIR,
    LED_INIT_LOOPBACK,
    LED_INIT_OTP_TO_FLASH,
    LED_INIT_SET_ACTIVE,
    LED_INIT_SET_SETUP,
    LED_INIT_ERROR
    } ledInitState_t;

static ledInitState_t stateLedInit = LED_INIT_INIT;
static enum OSP_ERROR_CODE errorCodeLed;
static bool loopBack = false;

DN_RGB_XYZ_t *xyzStorage = NULL;

void init_led_feature_control(void)
    {
    bool setActive = true;
    bool storeOtpToFlash = true;
    uint16_t countLed = 0;
    initFeatureError_t result;
    loopBack = false;
    init_led_feature_blocking(setActive, storeOtpToFlash, &countLed, &result);
    }

void init_led_feature_blocking(bool setActive, bool storeOtpToFlash, uint16_t *p_countLed, initFeatureError_t *p_result)
    {
    bool running = true;
    ospInitRsp_t rsp;

    while (running)
	{
	switch (stateLedInit)
	    {
	case LED_INIT_INIT:
	    errorCodeLed = 0;
	    loopBack = false;
	    stateLedInit = LED_INIT_RESET;
	    break;
	case LED_INIT_RESET:
	    /*-----------------------LED reset---------------------------------*/
	    hal_reset_osire_start();
	    errorCodeLed = osp_reset(0);
	    hal_reset_osire_end();

	    if (errorCodeLed != OSP_NO_ERROR)
		{
		stateLedInit = LED_INIT_ERROR;
		}
	    else
		{
		if (loopBack == false)
		    {
		    stateLedInit = LED_INIT_BIDIR;
		    }
		else
		    {
		    stateLedInit = LED_INIT_LOOPBACK;
		    }
		}
	    break;
	case LED_INIT_BIDIR:
	    /*-----------------------set LED MODE------------------------------*/
	    errorCodeLed = osp_init_bidir(1, &rsp);
	    if (errorCodeLed != OSP_NO_ERROR)
		{
		*p_countLed = 0;
		stateLedInit = LED_INIT_ERROR;
		}
	    else
		{
		/*-----------------------set last LED address------------------*/
		*p_countLed = rsp.data.bit.address;

		if (storeOtpToFlash == true)
		    {
		    stateLedInit = LED_INIT_OTP_TO_FLASH;
		    }
		else if (setActive == true)
		    {
		    stateLedInit = LED_INIT_SET_ACTIVE;
		    }
		else
		    {
		    stateLedInit = LED_INIT_SET_SETUP;
		    }
		}

	    break;

	case LED_INIT_LOOPBACK:
	    /*-----------------------set LED MODE------------------------------*/
	    loopBack = true;
	    errorCodeLed = osp_init_loop(1, &rsp);
	    if (errorCodeLed != OSP_NO_ERROR)
		{
		stateLedInit = LED_INIT_RESET;
		}
	    else
		{
		/*-----------------------set last LED address------------------*/
		*p_countLed = rsp.data.bit.address;

		if (storeOtpToFlash == true)
		    {
		    stateLedInit = LED_INIT_OTP_TO_FLASH;
		    }
		else if (setActive == true)
		    {
		    stateLedInit = LED_INIT_SET_ACTIVE;
		    }
		else
		    {
		    stateLedInit = LED_INIT_SET_SETUP;
		    }
		}

	    break;
	case LED_INIT_OTP_TO_FLASH:
	    /*----------------------- OTP to FLASH -------------------------------*/
	    errorCodeLed = init_feature_otp_to_flash(*p_countLed);

	    if (errorCodeLed != OSP_NO_ERROR)
		{
		stateLedInit = LED_INIT_ERROR;
		}
	    else if (setActive == true)
		{
		stateLedInit = LED_INIT_SET_ACTIVE;
		}
	    else
		{
		stateLedInit = LED_INIT_SET_SETUP;
		}
	    break;
	case LED_INIT_SET_ACTIVE:
	    /*-----------------------set LED active------------------------------*/
	    errorCodeLed = osp_go_active(0);
	    if (errorCodeLed != OSP_NO_ERROR)
		{
		stateLedInit = LED_INIT_ERROR;
		}
	    else
		{
		stateLedInit = LED_INIT_SET_SETUP;
		}
	    break;
	case LED_INIT_SET_SETUP:
	    {
	    /*-----------------------set LED Setup Register--------------------*/

	    osireSetSetupData_t setupData;
	    osireSetSetupData_t setupDataNeu;
	    setupData.data.setupData = 0x32; // Default?
	    errorCodeLed = osp_osire_set_setup(0, setupData);

	    if (errorCodeLed != OSP_NO_ERROR)
		{
		stateLedInit = LED_INIT_ERROR;
		}
	    else
		{
		stateLedInit = LED_INIT_INIT;

		errorCodeLed = osp_osire_read_setup(*p_countLed, &setupDataNeu);

		if (setupDataNeu.data.setupData == setupData.data.setupData)
		    {
		    *p_result = INIT_FEATURE_NO_ERROR;

		    }
		else
		    {
		    *p_result = INIT_FEATURE_ERROR;
		    }
		running = false;
		}

	    break;
	    }
	case LED_INIT_ERROR:
	    /*-----------------------Error-------------------------------------*/
	    running = false;
	    *p_result = INIT_FEATURE_ERROR;
	    break;
	default:
	    stateLedInit = LED_INIT_ERROR;
	    break;
	    }
	}

    stateLedInit = LED_INIT_INIT; //reset to Init for next call
    }

enum OSP_ERROR_CODE init_feature_otp_to_flash(uint16_t countLed)
    {
    enum OSP_ERROR_CODE errorCodeLed = OSP_NO_ERROR;
    osireOtpDataComplete_t otpMemory;
    DN_RGB_XYZ_t calibrationValueContainer;
    DN_RGB_XYZ_t calibrationValueContainerTest;
    uint32_t address = 0;
    int cmp_res = 0;

    // Clean the place and store data for all led's
    hal_erase_led_xyz_data_from_flash();
    for (uint16_t i = 0; i < countLed; i++)
	{
	// Read LED otp data
	errorCodeLed = osp_osire_read_otp_complete(i + 1, &otpMemory);
	if (errorCodeLed == OSP_NO_ERROR)
	    {
	    // Calculate xyz values using otp data
	    OTP_to_XYZ(&otpMemory, &calibrationValueContainer);

	    // Store calculated data to flash
	    address = i * sizeof(DN_RGB_XYZ_t);
	    Cy_Em_EEPROM_Write(address, (void *)&calibrationValueContainer, sizeof(calibrationValueContainer), &em_eeprom_context);
	    Cy_Em_EEPROM_Read (address,(void*)&calibrationValueContainerTest, sizeof(calibrationValueContainerTest), &em_eeprom_context);

	    /*Check if data is the same*/
	    cmp_res = memcmp((void *)&calibrationValueContainer, (void*)&calibrationValueContainerTest, sizeof(calibrationValueContainer));
	    if(cmp_res != 0)
	    {
	    	errorCodeLed = OSP_ADDRESS_ERROR;
	    	return errorCodeLed;
	    }

	    }
	else
	    {
	    // In case of error, exit the loop and return the error code.
	    break;
	    }
	}
    return errorCodeLed;
    }

bool init_feature_init_led_status_xyz_pointers(uint16_t countLed,ledPwmCalc_t *p_ledStatusArray)
{
	  uint32_t xyzStructAddr = em_eeprom_config.userFlashStartAddr;

	  for (uint16_t i = 0; i < countLed; i++)
	    {
	      if (xyzStructAddr != 0)
	        {
	          p_ledStatusArray[i].XYZTyp = (DN_RGB_XYZ_t*) xyzStructAddr;

	          /*Next address*/
	          xyzStructAddr = xyzStructAddr + sizeof(DN_RGB_XYZ_t);
	        }
	      else
	        {
	          // In case of error, break the loop and return false.
	          return false;
	        }
	    }

	  return true;
}
