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

#include <Feature/ColorCorrection/inc/colorDefinition.h>
#include <Feature/ColorCorrection/inc/ledPwmCalc.h>

static ledPwmCalc_t ledPwmCalcArray[MAX_COUNT_LED_SUPPORTED];

ledPwmCalc_t* get_addr_ledPwmCalcArray (void)
{
  return (ledPwmCalcArray);
}

void reset_ledPwmCalcArray (void)
{
  for (uint16_t i = 0; i < MAX_COUNT_LED_SUPPORTED; i++)
    {
      ledPwmCalcArray[i].regPwm.address = 0;
      ledPwmCalcArray[i].regPwm.data.pwmData[0] = 0;
      ledPwmCalcArray[i].regPwm.data.pwmData[1] = 0;
      ledPwmCalcArray[i].regPwm.data.pwmData[2] = 0;
      ledPwmCalcArray[i].regPwm.data.pwmData[3] = 0;
      ledPwmCalcArray[i].regPwm.data.pwmData[4] = 0;
      ledPwmCalcArray[i].regPwm.data.pwmData[5] = 0;
      ledPwmCalcArray[i].XYZTyp = 0;
    }
}

void OTP_to_XYZ (osireOtpDataComplete_t *otpMemory, DN_RGB_XYZ_t *p_xyzTyp)
{
  // Get values from OPT memory and set up LED status
  uint8_t *p_otp = otpMemory->data.byte;  // shortcut

  float up, vp, Y;  // color definition values
  // red (Night: 10mA)
  up = LSB_UV_NIGHT * (float) p_otp[0x0A];
  vp = LSB_UV_NIGHT * (float) p_otp[0x0B];
  Y = LSB_IV_NIGHT
      * (float) ((uint16_t) p_otp[0x0C]
          | (((uint16_t) (p_otp[0x0D] & 0x0f)) << 8));
  upvpY2XYZ (up, vp, Y, &p_xyzTyp->night.R);

  // red (Day: 50mA)
  up = LSB_UV_DAY * (float) p_otp[0x0E];
  vp = LSB_UV_DAY * (float) p_otp[0x0F];
  Y = LSB_IV_DAY
      * (float) ((uint16_t) p_otp[0x10] | ((uint16_t) (p_otp[0x0D] >> 4) << 8));
  upvpY2XYZ (up, vp, Y, &p_xyzTyp->day.R);

  // green (Night: 10mA)
  up = LSB_UV_NIGHT * (float) p_otp[0x18];
  vp = LSB_UV_NIGHT * (float) p_otp[0x19];
  Y = LSB_IV_NIGHT
      * (float) ((uint16_t) p_otp[0x1A]
          | (((uint16_t) (p_otp[0x1B] & 0x0f)) << 8));
  upvpY2XYZ (up, vp, Y, &p_xyzTyp->night.G);

  // green (Day: 50mA)
  up = LSB_UV_DAY * (float) p_otp[0x1C];
  vp = LSB_UV_DAY * (float) p_otp[0x1D];
  Y = LSB_IV_DAY
      * (float) ((uint16_t) p_otp[0x1E] | ((uint16_t) (p_otp[0x1B] >> 4) << 8));
  upvpY2XYZ (up, vp, Y, &p_xyzTyp->day.G);

  // blue (Night: 10mA)
  up = LSB_UV_NIGHT * (float) p_otp[0x11];
  vp = LSB_UV_NIGHT * (float) p_otp[0x12];
  Y = LSB_IV_NIGHT
      * (float) ((uint16_t) p_otp[0x13]
          | (((uint16_t) (p_otp[0x14] & 0x0f)) << 8));
  upvpY2XYZ (up, vp, Y, &p_xyzTyp->night.B);

  // blue (Day: 50mA)
  up = LSB_UV_DAY * (float) p_otp[0x15];
  vp = LSB_UV_DAY * (float) p_otp[0x16];
  Y = LSB_IV_DAY
      * (float) ((uint16_t) p_otp[0x17] | ((uint16_t) (p_otp[0x14] >> 4) << 8));
  upvpY2XYZ (up, vp, Y, &p_xyzTyp->day.B);
}
