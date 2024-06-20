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

#ifndef DEMOS_RUNNINGLIGHT_INC_RUNNINGLIGHT_H_
#define DEMOS_RUNNINGLIGHT_INC_RUNNINGLIGHT_H_


typedef enum
{
  RUNNING_LIGHT_INIT,
  RUNNING_LIGHT_CALCULATE_COLOR,
  RUNNING_LIGHT_ERROR,
  RUNNING_LIGHT_LOOP
} runningLightState_t;

typedef enum
{
  RUNNING_LIGHT_COLOR,
  RUNNING_LIGHT_DIMM_WHITE_FAST_NIGHT,
  RUNNING_LIGHT_DIMM_WHITE_SLOW_NIGHT,
  RUNNING_LIGHT_VERSION_END_MARKER,
  RUNNING_LIGHT_DIMM_WHITE_FAST_DAY,
  RUNNING_LIGHT_DIMM_WHITE_SLOW_DAY,
  //RUNNING_LIGHT_VERSION_END_MARKER
  RUNNING_LIGHT_VERSION_UART_END_MARKER
} runningLightVersion_t;


void reset_running_light_control (void);
void running_light_set_mode_via_uart_helper (runningLightVersion_t newVal);
void running_light_control (runningLightVersion_t colorVersion);


#endif /* DEMOS_RUNNINGLIGHT_INC_RUNNINGLIGHT_H_ */
