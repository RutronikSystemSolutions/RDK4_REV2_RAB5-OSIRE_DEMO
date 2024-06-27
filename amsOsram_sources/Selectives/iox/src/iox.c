// iox.c - driver for NXP PCA6408ABSHP IO expander, currently assuming 4 LEDs and 4 buttons are connected
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


#include <osp2/inc/osp2.h>
#include <iox/inc/iox.h>


// ==========================================================================
// This driver assumes an NXP PCA6408ABSHP IO expander is connected to a SAID I2C bus.
// Furthermore, it assume to port 1, 3, 5, and 7 have a button connected to it.
// The buttons are low active (0 when pushed).
// It also assumes that port 2, 4, 6, and 8 have a signaling LED connected to it.
// The signaling LEDs are high active (1 switches them on).
// This driver is not multi-instance (can not be used with two SAIDs at the same time)


// I2C address of the IO Expander
#define IOX_DADDR7         0x20


// Registers of the IO Expander
#define IOX_REGINVAL       0x00 // Input port register reflects the incoming logic levels of all pins (read)
#define IOX_REGOUTVAL      0x01 // The Output port register contains the outgoing logic levels of the pins defined as outputs (read/write)
#define IOX_REGINPINV      0x02 // The Polarity inversion register allows polarity inversion of pins defined as inputs (read/write)
#define IOX_REGCFGINP      0x03 // The Configuration register configures the direction of the I/O pins. If a bit is 1, the pin is input (read/write)


// Global vars
static uint16_t iox_saidaddr;   // address of the SAID with the buttons and LEDs


// ==========================================================================
// Signaling LED control by IOX
static uint8_t  iox_led_states; // Current state of the leds


// Masks for iox_led_on/off/set to tell which LED to switch (can be or'ed)
#define IOX_LED0    0x02
#define IOX_LED1    0x08
#define IOX_LED2    0x20
#define IOX_LED3    0x80
#define IOX_LED(n)  (1<<((n)*2+1)) // n=0..3
#define IOX_LEDALL  (IOX_LED0|IOX_LED1|IOX_LED2|IOX_LED3)
#define IOX_LEDNONE 0x00


// The bits set in `leds` indicate which LED to turn on
osp2_error_t iox_led_on( uint8_t leds) {
  iox_led_states |= leds;
  return osp2_exec_i2cwrite8(iox_saidaddr, IOX_DADDR7, IOX_REGOUTVAL, &iox_led_states, 1);
}


// The bits set in `leds` indicate which LED to turn off
osp2_error_t iox_led_off( uint8_t leds) {
  iox_led_states &= ~leds;
  return osp2_exec_i2cwrite8(iox_saidaddr, IOX_DADDR7, IOX_REGOUTVAL, &iox_led_states, 1);
}


// The bits set in `leds` indicate which LED to turn on, the clear bits, which to turn off
osp2_error_t iox_led_set( uint8_t leds) {
  iox_led_states = leds;
  return osp2_exec_i2cwrite8(iox_saidaddr, IOX_DADDR7, IOX_REGOUTVAL, &iox_led_states, 1);
}


// ==========================================================================
// Button control by IOX
static uint8_t  iox_but_prvstates; // Previous state of the buttons
static uint8_t  iox_but_curstates; // Current state of the buttons


// Masks for iox_but_wentdown to tell which buttons where pressed (can be or'ed)
#define IOX_BUT0   0x01
#define IOX_BUT1   0x04
#define IOX_BUT2   0x10
#define IOX_BUT3   0x40
#define IOX_BUTALL (IOX_BUT0|IOX_BUT1|IOX_BUT2|IOX_BUT3)


// Poll to update button state
osp2_error_t iox_but_poll( ) {
  iox_but_prvstates= iox_but_curstates;
  return osp2_exec_i2cread8(iox_saidaddr, IOX_DADDR7, IOX_REGINVAL, &iox_but_curstates, 1);
}


// Must first do iox_but_poll() before testing one or more iox_but_wentdown()
uint8_t iox_but_wentdown( uint8_t but ) {
  return ~iox_but_curstates & iox_but_prvstates & but;
}


// ==========================================================================
// Main IOX


// Checks if an IOX is connected to OSP node with address `addr`.
// Returns OSP2_ERROR_NONE when device is there, OSP2_ERROR_MISSI2CDEV when not, anything else for telegram errors.
// USE   if( iox_present(addr)==OSP2_ERROR_NONE ) {...}
osp2_error_t iox_present(uint16_t addr ) {
  uint8_t buf;
  osp2_error_t err;
  uint8_t daddr7= IOX_DADDR7;
  err = osp2_exec_i2cread8(addr, daddr7, 0x00, &buf, 1);
  if( err!=OSP2_ERROR_I2CNACK && err!=OSP2_ERROR_I2CTIMEOUT && err!=OSP2_ERROR_NONE ) return err;
  if( err==OSP2_ERROR_I2CNACK || err==OSP2_ERROR_I2CTIMEOUT ) return OSP2_ERROR_MISSI2CDEV;
  return OSP2_ERROR_NONE;
}


// Initializes iox driver
osp2_error_t iox_init(uint16_t addr) {
  osp2_error_t err;

  // record address of the SAID with I2C bridge
  iox_saidaddr= addr;

  // default I2C bus speed of 100kHz is ok
  // err= osp2_send_seti2ccfg(addr, OSP2_I2CCFG_FLAGS_DEFAULT, OSP2_I2CCFG_SPEED_DEFAULT);
  // if( err!=OSP2_ERROR_NONE ) return err;

  // switch signaling LEDs off
  err= iox_led_set(IOX_LEDNONE);
  if( err!=OSP2_ERROR_NONE ) return err;

  // configure button pins as input
  uint8_t cfg = IOX_BUTALL;
  err = osp2_exec_i2cwrite8(addr, IOX_DADDR7, IOX_REGCFGINP, &cfg, 1);
  if( err!=OSP2_ERROR_NONE ) return err;

  // determine "prev" state of buttons
  return iox_but_poll();
}


// Resets iox driver (all pins to input)
osp2_error_t iox_reset(uint16_t addr) {
  uint8_t cfg = 0xff;
  return osp2_exec_i2cwrite8(addr, IOX_DADDR7, IOX_REGCFGINP, &cfg, 1);
}
