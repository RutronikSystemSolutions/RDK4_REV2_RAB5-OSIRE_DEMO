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

#ifndef COLORCORRECTION_H_
#define COLORCORRECTION_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "colorDefinition.h"

/**
 * \struct pwm_t
 * \brief PWM values for red, green & blue led chip
 *
 * resolution: 14/15 bit, depending on selected PWM frequency mode
 *
 * \var pwm_t PWM_t::R
 * \brief pwm value for red chip
 * \var pwm_t PWM_t::G
 * \brief pwm value for green chip
 * \var pwm_t PWM_t::B
 * \brief pwm value for blue chip
 *
 */
typedef struct
{
  uint16_t R;
  uint16_t G;
  uint16_t B;
} pwm_t;

/*!
 * \enum CCError_t
 * \brief Color correction error codes
 *
 * \var CCError_t CCError_t::NO_ERROR
 * \brief no error occured
 *
 * \var CCError_t CCError_t::TARGET_COLOR_UNREACHABLE
 * \brief target color out of gamut or luminance specification.
 *
 *
 */
typedef enum
{
  NO_ERROR, TARGET_COLOR_UNREACHABLE
} CCError_t;

/*!
 * \brief calculate temperature compensated PWM values from tristimulus values X, Y, Z
 *
 * \param p_XYZTyp pointer to Tristimulus values for all LED chips (red, green, blue) and current configurations (day, night)
 * \param p_T pointer to target color (tristimulus values X, Y, Z)
 * \param temp temperature in °C
 * \param p_pwm pointer to pwm result vector
 * \param dayMode true = day Mode, false = night Mode
 * \return CCError_t pwm calculation result status
 */
CCError_t XYZ2pwm (DN_RGB_XYZ_t *p_XYZTyp, XYZ_t *p_T, float temp, pwm_t *p_pwm,
                   bool dayMode); // calculate pwm for target p_T for led at temperature temp

/**
 * \brief calculate temperature compensated PWM values from cx, cy & Y
 *
 * \param p_XYZTyp pointer to Tristimulus values for all LED chips (red, green, blue) and current configurations (day, night)
 * \param p_T pointer to target color (CIE cx, cy & Y)
 * \param temp temperature in °C
 * \param p_pwm pointer to pwm result vector
 * \param dayMode true = day Mode, false = night Mode
 * \return CCError_t pwm calculation result status
 */
CCError_t xyY2pwm (DN_RGB_XYZ_t *p_XYZTyp, xyY_t *p_T, float temp, pwm_t *p_pwm,
                   bool dayMode);

/**
 * \brief calculate temperature compensated PWM values from u', v' & Y
 *
 * \param p_XYZTyp pointer to Tristimulus values for all LED chips (red, green, blue) and current configurations (day, night)
 * \param p_T pointer to target color (CIE 1976 UCS u', v' & Y)
 * \param temp temperature in °C
 * \param p_pwm pointer to pwm result vector
 * \param dayMode true = day Mode, false = night Mode
 * \return CCError_t pwm calculation result status
 */
CCError_t upvpY2pwm (DN_RGB_XYZ_t *p_XYZTyp, upvpY_t *p_T, float temp,
                     pwm_t *p_pwm, bool dayMode);

#ifdef __cplusplus
}
#endif

#endif /* COLORCORRECTION_H_ */
