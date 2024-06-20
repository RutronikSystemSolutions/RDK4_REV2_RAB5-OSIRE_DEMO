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

#ifndef HAL_GPIOS_INC_PIN_H_
#define HAL_GPIOS_INC_PIN_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
/**
 *
 */
void hal_init_pin (void);
/**
 *
 * @param val
 */
void set_led_CS_Slave (uint8_t val); //0 = low, 1 = high
uint8_t hal_check_SW2 (void);
uint8_t hal_check_SW3 (void);
/**
 *
 * @param val
 */
void set_led_red (uint8_t val); 	//0 = OFF, 1 = ON, 2 = Toggle
/**
 *
 * @param val
 */
void set_led_green (uint8_t val); 	//0 = OFF, 1 = ON, 2 = Toggle
/**
 *
 * @param val
 */
void set_led_blue (uint8_t val);	//0 = OFF, 1 = ON, 2 = Toggle

void set_ext_pull_up_invalid (void);
void set_ext_pull_up_valid (void);
/**
 *
 * @param val
 */
void set_debug_1 (uint8_t val); //0 = OFF, 1 = ON, 2 = Toggle
/**
 *
 * @param val
 */
void set_debug_2 (uint8_t val); //0 = OFF, 1 = ON, 2 = Toggle
/**
 *
 * @param val
 */
void set_debug_3 (uint8_t val); //0 = OFF, 1 = ON, 2 = Toggle
/**
 *
 * @param val
 */

void set_debug_4 (uint8_t val); //0 = OFF, 1 = ON, 2 = Toggle

/**
 *
 * @param val
 */

void set_debug_5 (uint8_t val); //0 = OFF, 1 = ON, 2 = Toggle

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIOS_INC_PIN_H_ */
