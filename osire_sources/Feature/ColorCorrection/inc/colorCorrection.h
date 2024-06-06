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

#ifndef DEMOS_COLORCORRECTION_INC_COLORCORRECTION_H_
#define DEMOS_COLORCORRECTION_INC_COLORCORRECTION_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <Feature/ColorCorrection/inc/colorDefinition.h>
#include <Feature/ColorCorrection/inc/colorCorrectionCoefficients.h>
#include <Feature/ColorCorrection/inc/ledPwmCalc.h>

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

// Vector & matrix operations
/*!
 * \brief  3-dim float
 */
typedef float vector3[3];   // 3-dim row vector
/*!
 * \brief  3x3 float matrix
 */
typedef vector3 matrix3[3]; // 3x3 matrix

/*!
 * \brief multiply matrix3 * vector3: w = M*v
 *
 * \param M 3x3 float matrix
 * \param v 3-dim input vector
 * \param w 3-dim output vector
 */
void multiply_vec (matrix3 M, vector3 v, vector3 w); // multiply matrix * vector: w = M*v

/*!
 * \brief multiply matrix3 * matrix3: C = A*B
 *
 * \param A 3x3 float input matrix
 * \param B 3x3 float input matrix
 * \param C 3x3 float output matrix
 */
void multiply_mat (matrix3 A, matrix3 B, matrix3 C); // multiply matrix * matrix: C = A*B

/*!
 * \brief calculate determinant of matrix A
 *
 * \param A 3x3 float input matrix
 * \return determinant of 3x3 matrix A
 */
float det (matrix3 A);                              // determinant of matrix A

// Color correction & PWM calculation

/**
 * \brief temperature compensation function for LED (red, green & blue chip)
 * 
 * \param p_led pointer to LED status structure
 * \param T temperature in [°C]
 * \param p_RGB pointer to temperature compensated tristimulus values for red, green & blue chip
 * \param dayMode true = day Mode, false = night Mode
 */
void color_compensation_RGB (ledPwmCalc_t *p_led, float T, RGB_XYZ_t *p_RGB,
bool dayMode);

/**
 * \brief temperature compensation function for tristimulus values X, Y & Z
 * 
 * \param p_XYZ_typ pointer to typical tristimulus values X, Y, Z 
 * \param p_Coef pointer to coefficients for temperature compensation functions
 * \param dT temperature difference dT = T - T_cal (calibration temperature)
 * \param p_XYZ pointer to temperature compensated tristimulus values X, Y, Z
 */
void color_compensation_XYZ (XYZ_t *p_XYZTyp, corrFunCoef_t *p_coef, float dT,
                             XYZ_t *p_XYZ); // temperature compensated tristimulus values

#ifdef __cplusplus
}
#endif

#endif /* DEMOS_COLORCORRECTION_INC_COLORCORRECTION_H_ */
