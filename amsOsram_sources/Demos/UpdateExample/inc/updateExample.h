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

#ifdef __cplusplus
extern "C"
  {
#endif

#ifndef DEMOS_UPDATEEXAMPLE_INC_UPDATEEXAMPLE_H_
#define DEMOS_UPDATEEXAMPLE_INC_UPDATEEXAMPLE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  UE_INIT,
  UE_CALC,
  UE_SEND_DATA,
  UE_READ_OUT,
  UE_WAIT_FOR_DATA,
  UE_WAIT,
  UE_ERROR
} updateExampleStates_t;

typedef enum
{
  UE_START_WITH_DEFAULT_VALUES = 0,
  UE_START_WITH_COLOR_NIGHT_MODE,
  UE_CHANGE_COLOR_NIGHT_MODE,
  UE_CHANGE_COLOR_DAY_MODE
} updateExampleModeUart_t;

void reset_update_example (void);
void update_example_init (void);
void update_example_control (void);
void change_color_update_example (float cx, float cy, float mcd,
bool dayModeSetting);

#endif /* DEMOS_UPDATEEXAMPLE_INC_UPDATEEXAMPLE_H_ */

#ifdef __cplusplus
}
#endif
