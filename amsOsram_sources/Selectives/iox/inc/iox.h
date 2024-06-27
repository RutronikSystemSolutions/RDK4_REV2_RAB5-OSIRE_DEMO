// iox.h - driver for NXP PCA6408ABSHP IO expander, currently assuming 4 LEDs and 4 buttons are connected
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
#ifndef IOX_H_
#define IOX_H_

#ifdef __cplusplus
extern "C"
  {
#endif


#include <osp2/inc/osp2.h>


// ==========================================================================
// Signaling LED control by IOX


// Masks for iox_led_on/off/set to tell which LED to switch (can be or'ed)
#define IOX_LED0    0x02
#define IOX_LED1    0x08
#define IOX_LED2    0x20
#define IOX_LED3    0x80
#define IOX_LED(n)  (1<<((n)*2+1)) // n=0..3
#define IOX_LEDALL  (IOX_LED0|IOX_LED1|IOX_LED2|IOX_LED3)
#define IOX_LEDNONE 0x00


osp2_error_t iox_led_on ( uint8_t leds); // The bits set in `leds` indicate which LED to turn on
osp2_error_t iox_led_off( uint8_t leds); // The bits set in `leds` indicate which LED to turn off
osp2_error_t iox_led_set( uint8_t leds); // The bits set in `leds` indicate which LED to turn on, the clear bits, which to turn off


// ==========================================================================
// Button control by IOX


// Masks for iox_but_wentdown to tell which buttons where pressed (can be or'ed)
#define IOX_BUT0   0x01
#define IOX_BUT1   0x04
#define IOX_BUT2   0x10
#define IOX_BUT3   0x40
#define IOX_BUTALL (IOX_BUT0|IOX_BUT1|IOX_BUT2|IOX_BUT3)


osp2_error_t iox_but_poll( );                 // Poll to update button state
uint8_t      iox_but_wentdown( uint8_t but ); // Must first do iox_but_poll() before testing one or more iox_but_wentdown()


// ==========================================================================
// Main IOX


osp2_error_t iox_present(uint16_t addr ); // Checks if an IOX is connected to OSP node with address `addr`. Returns OSP2_ERROR_NONE when device is there, OSP2_ERROR_MISSI2CDEV when not, anything else for telegram errors.
osp2_error_t iox_init(uint16_t addr);     // Initializes iox driver
osp2_error_t iox_reset(uint16_t addr);    // Resets iox driver (all pins to input)


#ifdef __cplusplus
}
#endif

#endif  // IOX_H_
