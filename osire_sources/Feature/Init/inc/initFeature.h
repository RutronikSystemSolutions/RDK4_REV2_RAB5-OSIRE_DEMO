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

#ifndef FEATURE_INIT_INC_INITFEATURE_H_
#define FEATURE_INIT_INC_INITFEATURE_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <Feature/ColorCorrection/inc/ledPwmCalc.h>
#include <Osp/inc/genericDevice.h>

typedef enum
{
  INIT_FEATURE_NO_ERROR, INIT_FEATURE_ERROR
} initFeatureError_t;

void init_led_feature_control (void);
void init_led_feature_blocking (bool setActive, bool storeOtpToFlash,
                                uint16_t *p_countLed,
                                initFeatureError_t *p_result);
enum OSP_ERROR_CODE init_feature_otp_to_flash (uint16_t p_countLed);
bool init_feature_init_led_status_xyz_pointers (uint16_t p_countLed,
                                                ledPwmCalc_t *p_ledStatusArray);

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_INIT_INC_INITFEATURE_H_ */
