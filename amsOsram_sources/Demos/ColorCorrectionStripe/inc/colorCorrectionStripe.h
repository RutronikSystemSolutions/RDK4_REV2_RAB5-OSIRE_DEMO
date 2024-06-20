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

#ifndef DEMOS_COLORCORRECTIONSTRIPE_INC_COLORCORRECTIONSTRIPE_H_
#define DEMOS_COLORCORRECTIONSTRIPE_INC_COLORCORRECTIONSTRIPE_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>

typedef enum
{
  INIT = 0, TARGET_COLOR_DEFINITION, GET_TEMP, CALC_COLOR, SET_COLOR, ERROR
} colorCorrectionStripeState_t;

void color_Correction_Stripe_Control (uint8_t withCorrection);
void reset_color_correction (void);

#ifdef __cplusplus
}
#endif

#endif /* DEMOS_COLORCORRECTIONSTRIPE_INC_COLORCORRECTIONSTRIPE_H_ */
