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

#ifndef DEMOS_COLORCORRECTION_INC_COLORCORRECTIONTEST_H_
#define DEMOS_COLORCORRECTION_INC_COLORCORRECTIONTEST_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <Feature/ColorCorrection/inc/colorCorrection.h>
#include <Feature/ColorCorrection/inc/ledPwmCalc.h>

#define TEST_COLOR_CORRECTION 1 // define this to compile test functions for color correction

#ifdef TEST_COLOR_CORRECTION
#define NUMBER_OF_DEVICES 100

/**
 * \brief helper function for test
 *
 */
void test_cc_sweep (XYZ_t *p_XYZ);

/*!
 * \brief calculate temperature compensated PWM values from tristimulus values X, Y, Z
 *
 * \param p_led pointer to RGBi device status
 * \param p_T pointer to target color (tristimulus values X, Y, Z)
 * \param temp temperature in °C
 * \param p_pwm pointer to pwm result vector
 * \param dayMode true = day Mode, false = night Mode
 * \return CCError_t pwm calculation result status
 */
CCError_t XYZ2pwm (ledPwmCalc_t *p_led, XYZ_t *p_T, float temp, pwm_t *p_pwm,
bool dayMode); // calculate pwm for target p_T for led at temperature temp

/**
 * \brief calculate temperature compensated PWM values from cx, cy & Y
 *
 * \param p_led pointer to RGBi device status
 * \param p_T pointer to target color (CIE cx, cy & Y)
 * \param temp temperature in °C
 * \param p_pwm pointer to pwm result vector
 * \param dayMode true = day Mode, false = night Mode
 * \return CCError_t pwm calculation result status
 */
CCError_t xyY2pwm (ledPwmCalc_t *p_led, xyY_t *p_T, float temp, pwm_t *p_pwm,
bool dayMode);

/**
 * \brief calculate temperature compensated PWM values from u', v' & Y
 *
 * \param p_led pointer to RGBi device status
 * \param p_T pointer to target color (CIE 1976 UCS u', v' & Y)
 * \param temp temperature in °C
 * \param p_pwm pointer to pwm result vector
 * \param dayMode true = day Mode, false = night Mode
 * \return CCError_t pwm calculation result status
 */
CCError_t upvpY2pwm (ledPwmCalc_t *p_led, upvpY_t *p_T, float temp,
                     pwm_t *p_pwm, bool dayMode);

#endif // TEST_COLOR_CORRECTION

#ifdef __cplusplus
}
#endif

#endif /* DEMOS_COLORCORRECTION_INC_COLORCORRECTIONTEST_H_ */
