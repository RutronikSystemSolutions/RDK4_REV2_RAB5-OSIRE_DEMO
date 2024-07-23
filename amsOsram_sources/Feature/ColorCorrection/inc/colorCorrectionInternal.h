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

#ifndef COLORCORRECTIONINTERNAL_H_
#define COLORCORRECTIONINTERNAL_H_

#include "colorCorrectionCoefficients.h"

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
static void multiply_vec (matrix3 M, vector3 v, vector3 w); // multiply matrix * vector: w = M*v

/*!
 * \brief calculate determinant of matrix A
 *
 * \param A 3x3 float input matrix
 * \return determinant of 3x3 matrix A
 */
static float det (matrix3 A);                              // determinant of matrix A

// Color correction & PWM calculation

/**
 * \brief temperature compensation function for LED (red, green & blue chip)
 *
 * \param p_led pointer to LED status structure
 * \param T temperature in [°C]
 * \param p_RGB pointer to temperature compensated tristimulus values for red, green & blue chip
 * \param dayMode true = day Mode, false = night Mode
 */
static void color_compensation_RGB (DN_RGB_XYZ_t* XYZTyp, float T, RGB_XYZ_t *p_RGB, bool dayMode);

/**
 * \brief temperature compensation function for tristimulus values X, Y & Z
 *
 * \param p_XYZ_typ pointer to typical tristimulus values X, Y, Z
 * \param p_Coef pointer to coefficients for temperature compensation functions
 * \param dT temperature difference dT = T - T_cal (calibration temperature)
 * \param p_XYZ pointer to temperature compensated tristimulus values X, Y, Z
 */
static void color_compensation_XYZ (XYZ_t *p_XYZTyp, const corrFunCoef_t *p_coef, float dT,
                             XYZ_t *p_XYZ); // temperature compensated tristimulus values




#endif /* COLORCORRECTIONINTERNAL_H_ */
