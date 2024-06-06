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

#include <Feature/ColorCorrection/inc/colorDefinition.h>

void cxy2upvp (float cx, float cy, float *p_up, float *p_vp) // convert CIE cxy -> u'v'
{
  float d = 1 / (12 * cy - 2 * cx + 3);
  *p_up = 4 * cx * d;
  *p_vp = 9 * cy * d;
}

void upvp2cxy (float up, float vp, float *p_cx, float *p_cy) // convert u'v' -> CIE cxy
{
  float d = 1 / (6 * up - 16 * vp + 12);
  *p_cx = 9 * up * d;
  *p_cy = 4 * vp * d;
}

void cxyY2XZ (float cx, float cy, float Y, float *p_X, float *p_Z) // convert cxyY -> tristimulus X,Y,Z
{
  float f = Y / cy;
  *p_X = cx * f;
  *p_Z = (1 - cx - cy) * f;
  // no need to calculate Y, it it's the same as the input
}

void upvpY2XZ (float up, float vp, float Y, float *p_X, float *p_Z) // convert u'v'Y -> tristimulus X,Y,Z
{
  float f = Y / (4 * vp);
  *p_X = 9 * up * f;
  *p_Z = (12 - 3 * up - 20 * vp) * f;
  // no need to calculate Y, it it's the same as the input
}

void cxyY2XYZ (float cx, float cy, float Y, XYZ_t *p_XYZ) // convert cxyY -> tristimulus X,Y,Z
{
  cxyY2XZ (cx, cy, Y, &p_XYZ->X, &p_XYZ->Z);
  p_XYZ->Y = Y;  // just copy tristimulus Y
}

void upvpY2XYZ (float up, float vp, float Y, XYZ_t *p_XYZ) // convert u'v'Y0 -> tristimulus X,Y,Z
{
  upvpY2XZ (up, vp, Y, &p_XYZ->X, &p_XYZ->Z);
  p_XYZ->Y = Y;  // just copy tristimulus Y
}

void XYZ2cxy (float X, float Y, float Z, float *p_cx, float *p_cy) // convert tristimulus X,Y,Z -> CIE cxy
{
  float d = 1 / (X + Y + Z);
  *p_cx = X * d;
  *p_cy = Y * d;
}

void XYZ2upvp (float X, float Y, float Z, float *p_up, float *p_vp) // convert tristimulus X,Y,Z -> CIE LUV u'v'
{
  float d = 1 / (X + 15 * Y + 3 * Z);
  *p_up = 4 * X * d;
  *p_vp = 9 * Y * d;
}
