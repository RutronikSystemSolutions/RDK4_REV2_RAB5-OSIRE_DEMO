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

#include <Feature/ColorCorrection/inc/colorCorrectionTest.h>
#include <stddef.h>

#ifdef CPU_S32K144
#include <Hal/Gpios/inc/pin.h>
#else
// define stub functions (for test)
void set_debug_1(uint8_t val);
void set_debug_2(uint8_t val);
void set_led_red(uint8_t val);
void set_led_green(uint8_t val);
void set_led_blue(uint8_t val);
#endif

#ifdef TEST_COLOR_CORRECTION

// Global LED Status memory
extern ledPwmCalc_t LEDSystemStatus[NUMBER_OF_DEVICES];
XYZ_t T_XYZ; // target color

#define NUMBER_OF_TEMPERATURES 11
float TEMP_SWEEP[NUMBER_OF_TEMPERATURES] =
  { -40, -20, -10, 0, 10, 25, 50, 85, 100, 110, 125 };

void test_cc_sweep (XYZ_t *p_XYZ)
{

  // loop over temperatures
  for (size_t i = 0; i < NUMBER_OF_TEMPERATURES; ++i)
    {
      float temp = TEMP_SWEEP[i];
      // loop over devices
      set_debug_2 (1);
      for (size_t k = 0; k < NUMBER_OF_DEVICES; ++k)
        {
          set_debug_1 (1);
          ledPwmCalc_t *p_led = &LEDSystemStatus[k];
          pwm_t pwm;
          CCError_t rc = XYZ2pwm (p_led, p_XYZ, temp, &pwm, 0);
          set_debug_1 (0);
          if (rc == NO_ERROR)
            {
              // green led
              set_led_red (0);
              set_led_green (1);
            }
          else
            {
              // red led
              set_led_green (0);
              set_led_red (1);

            }
        }
      set_debug_2 (0);
    }
}

#endif // TEST_COLOR_CORRECTION
