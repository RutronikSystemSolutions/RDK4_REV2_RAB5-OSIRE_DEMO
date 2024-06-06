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
 */

#ifndef FEATURE_COLORCORRECTION_INC_LEDPWMCALC_H_
#define FEATURE_COLORCORRECTION_INC_LEDPWMCALC_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <Feature/ColorCorrection/inc/colorDefinition.h>
#include <Feature/ColorCorrection/inc/colorCorrectionCoefficients.h>
#include <osire_sources/Osp/inc/osireDevice.h>

#define MAX_COUNT_LED_SUPPORTED 1000

/**
 * \brief quantization value for luminous intensity (Night mode)
 * 
 */
#define LSB_IV_NIGHT 0.44f  // [mcd]

/**
 * \brief quantization value for luminous intensity (Day mode)
 * 
 */
#define LSB_IV_DAY   1.25f // [mcd]

/**
 * \brief quantization value for CIE 1976 u'v' coordinates (Night mode)
 * 
 */
#define LSB_UV_NIGHT 0.0025f

/**
 * \brief quantization value for CIE 1976 u'v' coordinates (Day mode)
 * 
 */
#define LSB_UV_DAY   0.0025f

/**
 * \def LED_OTP_SIZE
 * \brief size of LED OTP memory in bytes 
 * 
 */
#define LED_OTP_SIZE 32

/**
 * \brief defines buffer for OTP memory (32 bytes)
 * 
 */
typedef uint8_t otpMemory_t[LED_OTP_SIZE];

// Device Parameter
/*!
 * \enum PWMFrequency
 *
 * \brief Defines PWM frequency and PWM resolution
 *
 * \var PWMFrequency PWMFrequency::PWM_FAST_MODE
 * \brief PWM frequency at least 1 kHz; PWM resolution 14 bit
 *
 * \var PWMFrequency PWMFrequency::PWM_SLOW_MODE
 * \brief PWM frequency at least 500 Hz; PWM resolution 15 bit
 */
enum PWMFrequency
{
  PWM_FAST_MODE,   //< 1kHz, 14bit resolution
  PWM_SLOW_MODE    //< 500Hz, 15bit resolution
};
typedef enum PWMFrequency pwmFrequency_t;

/*!
 * \enum LEDCurrent
 *
 * \brief Defines LED current
 *
 * \var PWMFrequency LEDCurrent::Night
 * \brief Night mode LED current 10 mA +/- 3%
 *
 * \var LEDCurrent LEDCurrent::Day
 * \brief Day mode LED current 50 mA +/- 3%
 */
enum LEDCurrent
{
  Night,  // 10 mA
  Day     // 50 mA
};
typedef enum LEDCurrent ledCurrent_t;

typedef struct
{
  osirePwmData_t regPwm;
  // typical values (calibration data)
  DN_RGB_XYZ_t *XYZTyp;
} ledPwmCalc_t;

/**
 * \brief initialize typical XYZ values
 * \param otpMemory otpMemory structure
 * \param xyzTyp  pointer on container for xyz values
 */
void OTP_to_XYZ (osireOtpDataComplete_t *otpMemory, DN_RGB_XYZ_t *p_xyzTyp);

/**
 * \brief returns pointer to ledPwmCalcArray
 * \return pointer to ledPwmCalcArray
 */
ledPwmCalc_t* get_addr_ledPwmCalcArray (void);

/**
 * \brief sets ledPwmCalcArray values to 0
 *
 */
void reset_ledPwmCalcArray (void);

#define LED_STATUS_LED_SYSTEM_STATUS_H_

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_COLORCORRECTION_INC_LEDPWMCALC_H_ */
