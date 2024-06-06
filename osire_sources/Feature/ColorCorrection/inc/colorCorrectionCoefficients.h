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

#ifndef DEMOS_COLORCORRECTION_INC_COLORCORRECTIONCOEFFICIENTS_H_
#define DEMOS_COLORCORRECTION_INC_COLORCORRECTIONCOEFFICIENTS_H_

#ifdef __cplusplus
extern "C"
  {
#endif

/*!
 *  \struct InterpolFunCoef_t
 *  \brief coefficients for interpolating functions: f(T) = a*(T-T_cal)^2 + b*(T-T_cal) + 1
 *
 *  T: Temperature in °C
 *
 *  T_cal: calibration temperature (typ. 25°C)
 *
 *  \var InterpolFunCoef_t InterpolFunCoef_t::a
 *  \brief coefficient a in f(T)
 *  \var InterpolFunCoef_t InterpolFunCoef_t::b
 *  \brief coefficient b in f(T)
 */
typedef struct
{
  float a;
  float b;
} InterpolFunCoef_t;

/*!
 *  \struct CorrFunCoef_t
 *  \brief coefficients for interpolating functions of relative tristimulus values (X, Y, Z) over temperature
 *  \var CorrFunCoef_t CorrFunCoef_t::XRel
 *  \brief coefficients for interpolated relative tristimulus value XRel(T)
 *  \var CorrFunCoef_t CorrFunCoef_t::YRel
 *  \brief coefficients for interpolated relative tristimulus value YRel(T)
 *  \var CorrFunCoef_t CorrFunCoef_t::ZRel
 *  \brief coefficients for interpolated relative tristimulus value ZRel(T)
 */
typedef struct
{
  InterpolFunCoef_t XRel;
  InterpolFunCoef_t YRel;
  InterpolFunCoef_t ZRel;
} corrFunCoef_t;

/*!
 *  \struct ColorCorrFunCoef_t
 *  \brief coefficients for color corrections functions (red, green, blue)
 *  \var ColorCorrFunCoef_t ColorCorrFunCoef_t::R
 *  \brief coefficients to interpolate tristimulus values X, Y, Z for red chip
 *  \var ColorCorrFunCoef_t ColorCorrFunCoef_t::G
 *  \brief coefficients to interpolate tristimulus values X, Y, Z for green chip
 *  \var ColorCorrFunCoef_t ColorCorrFunCoef_t::B
 *  \brief coefficients to interpolate tristimulus values X, Y, Z for blue chip
 */
typedef struct
{
  corrFunCoef_t R;
  corrFunCoef_t G;
  corrFunCoef_t B;
} colorCorrFunCoef_t;

#ifdef __cplusplus
}
#endif

#endif /* DEMOS_COLORCORRECTION_INC_COLORCORRECTIONCOEFFICIENTS_H_ */
