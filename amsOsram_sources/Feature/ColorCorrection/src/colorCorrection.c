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

#include "colorCorrection.h"
#include "colorCorrectionCoefficients.h"
#include "colorCorrectionInternal.h"

static const float T_cal = 25;  // calibration function

static void multiply_vec (matrix3 M, vector3 v, vector3 u) // multiply matrix * vector: u = M*v
{
  for (uint8_t i = 0; i < 3; ++i)
    {
      u[i] = 0;
      for (uint8_t j = 0; j < 3; ++j)
        {
          u[i] += M[i][j] * v[j];
        }
    }
}

static float det (matrix3 A) // determinant of matrix A
{
  float d = A[0][0] * A[1][1] * A[2][2] - A[0][0] * A[1][2] * A[2][1]
          - A[0][1] * A[1][0] * A[2][2] + A[0][1] * A[1][2] * A[2][0]
          + A[0][2] * A[1][0] * A[2][1] - A[0][2] * A[1][1] * A[2][0];
  return d;
}

// temperature compensation
static void color_compensation_RGB (DN_RGB_XYZ_t* XYZTyp, float T, RGB_XYZ_t* p_RGB, bool dayMode)
{
  // calculate tristimulus values XYZ at temperature T for each color
  float dT = T - T_cal;

  if (dayMode == true)
    {
      // red
      color_compensation_XYZ (&XYZTyp->day.R, &ColorCorrDayCoef.R, dT, &p_RGB->R);
      // green
      color_compensation_XYZ (&XYZTyp->day.G, &ColorCorrDayCoef.G, dT, &p_RGB->G);
      // blue
      color_compensation_XYZ (&XYZTyp->day.B, &ColorCorrDayCoef.B, dT, &p_RGB->B);
    }
  else
    {
      // red
      color_compensation_XYZ (&XYZTyp->night.R, &ColorCorrNightCoef.R, dT, &p_RGB->R);
      // green
      color_compensation_XYZ (&XYZTyp->night.G, &ColorCorrNightCoef.G, dT, &p_RGB->G);
      // blue
      color_compensation_XYZ (&XYZTyp->night.B, &ColorCorrNightCoef.B, dT, &p_RGB->B);
    }
}

// temperature compensated tristimulus values, dT = T-T_cal
static void color_compensation_XYZ (XYZ_t *p_XYZTyp, const corrFunCoef_t *p_coef, float dT, XYZ_t *p_XYZ)
{
  // evaluate color compensation for tristimulus values using Horner's scheme:
  p_XYZ->X = (((p_coef->XRel.a * dT) + p_coef->XRel.b) * dT + 1.0f)
      * p_XYZTyp->X;
  p_XYZ->Y = (((p_coef->YRel.a * dT) + p_coef->YRel.b) * dT + 1.0f)
      * p_XYZTyp->Y;
  p_XYZ->Z = (((p_coef->ZRel.a * dT) + p_coef->ZRel.b) * dT + 1.0f)
      * p_XYZTyp->Z;
}

// calculate pwm for target p_T for led at temperature temp
CCError_t XYZ2pwm (DN_RGB_XYZ_t *XYZTyp, XYZ_t *p_T, float temp, pwm_t *p_pwm, bool dayMode)
{
  // set up the tristimulus matrix of LED (red, green, blue), incl.
  // temperature correction

  // temperature compensation
  RGB_XYZ_t rgb;  // color compensated tristimulus values
  color_compensation_RGB (XYZTyp, temp, &rgb, dayMode);

  matrix3 A =
    {
      { rgb.R.X, rgb.G.X, rgb.B.X },
      { rgb.R.Y, rgb.G.Y, rgb.B.Y },
      { rgb.R.Z, rgb.G.Z, rgb.B.Z }, };

  // Matrix inversion
  matrix3 Atilde =
    {  // A~ := det(A) * A^-1 (=> only multiplications, no divisions)
        { A[1][1] * A[2][2] - A[1][2] * A[2][1], A[0][2] * A[2][1]
            - A[0][1] * A[2][2], A[0][1] * A[1][2] - A[0][2] * A[1][1] },
        { A[1][2] * A[2][0] - A[1][0] * A[2][2], A[0][0] * A[2][2]
            - A[0][2] * A[2][0], A[0][2] * A[1][0] - A[0][0] * A[1][2] },
        { A[1][0] * A[2][1] - A[1][1] * A[2][0], A[0][1] * A[2][0]
            - A[0][0] * A[2][1], A[0][0] * A[1][1] - A[0][1] * A[1][0] }, };

  // target vector
  vector3 T =
    { p_T->X, p_T->Y, p_T->Z };

  // pwm values
  vector3 pwm;
  multiply_vec (Atilde, T, pwm);

  // correction factor (leftover from matrix inversion)
  CCError_t rc = NO_ERROR;

  float kA = 1.0f / det (A);
  for (int i = 0; i < 3; ++i)
    {
      pwm[i] *= kA;
      // review pwm: limit pwm values to [0.0 .. 1.0]
      if (pwm[i] < 0)
        {
          pwm[i] = 0;
          rc = TARGET_COLOR_UNREACHABLE;
        }
      else if (pwm[i] > 1.0f)
        {
          pwm[i] = 1.0f;
          rc = TARGET_COLOR_UNREACHABLE;
        }
    }

  // convert to integer
  float k = 32767.0f; // factor both valid for fast (15 bit resolution) and slow-mode (14 bit resolution)

  // map to bits [14:0] (fast) or [14:1] (slow mode: skip bit 0)
  p_pwm->R = (uint16_t) (pwm[0] * k);
  p_pwm->G = (uint16_t) (pwm[1] * k);
  p_pwm->B = (uint16_t) (pwm[2] * k);

  return rc;
}

CCError_t xyY2pwm (DN_RGB_XYZ_t *XYZTyp, xyY_t *p_T, float temp, pwm_t *p_pwm, bool dayMode)
{
  XYZ_t XYZ;
  cxyY2XYZ (p_T->cx, p_T->cy, p_T->Y, &XYZ);
  return XYZ2pwm (XYZTyp, &XYZ, temp, p_pwm, dayMode);
}

CCError_t upvpY2pwm (DN_RGB_XYZ_t *XYZTyp, upvpY_t *p_T, float temp,
                     pwm_t *p_pwm, bool dayMode)
{
  XYZ_t XYZ;
  upvpY2XYZ (p_T->up, p_T->vp, p_T->Y, &XYZ);
  return XYZ2pwm (XYZTyp, &XYZ, temp, p_pwm, dayMode);
}
