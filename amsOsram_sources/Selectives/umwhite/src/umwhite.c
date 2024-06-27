// umwhite.c - User Mode controlling a LED string in shades of white, sw toggles MSB dithering
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

/*
DESCRIPTION
- The LEDs are in a dimming cycle (dim up, then dim down, then up again, etc).
- All LEDs dim synchronously and at the same level (so RGB look white).
- When SW2 is pressed for the first time, dimming cycle stops (forever).
- To restart the dimming cycle, restart umwhite (reset, sw3).
- Every next time SW2 is pressed, dithering is toggled.

GOAL
- To show the effect of dithering
- View the LEDs with a mobile phone camera in video mode to see flickering starts when dithering is disabled,
  or use LED Light Flicker Meter (https://play.google.com/store/apps/details?id=com.contechity.flicker_meter).
*/

#include <stdio.h>
#include <string.h>
#include <SwTimer/inc/swTimer.h>
#include <Hal/Button/inc/button.h>     // button 3
#include <osp2/inc/osp2.h>
#include <topo/inc/topo.h>
#include <umwhite/inc/umwhite.h>
#include "sys_timer.h"


// ==========================================================================
// Main state machine


// Within the RUN state, there is a sub state machine; it has its own start and (micro) step function.
static osp2_error_t umwhite_run_start();
static osp2_error_t umwhite_run_step();


// State tells what is to do next
typedef enum umwhite_state_e {
  UMWHITE_STATE_INIT,  // Send RESET and INIT (BIDIR/LOOP) telegrams, then scan topology
  UMWHITE_STATE_SETUP, // Setup individual nodes for the demo
  UMWHITE_STATE_RUN,   // Demo running step by step
  UMWHITE_STATE_ERROR, // Terminal state when an error is detected (that error is recorded in umwhite_error)
} umwhite_state_t;


static osp2_error_t    umwhite_error;    // last error
static umwhite_state_t umwhite_state;    // current state


// Resets internal state of user mode led animation
void umwhite_start() {
  umwhite_error= OSP2_ERROR_NONE;
  umwhite_state= UMWHITE_STATE_INIT;
}


// Returns 1 on error
int umwhite_step() {
  switch( umwhite_state ) {

    case UMWHITE_STATE_INIT :
      umwhite_error= topo_stdinit();
      if( umwhite_error!=OSP2_ERROR_NONE ) { umwhite_state=UMWHITE_STATE_ERROR; return 1; }
      printf("umwhite: nodes %u triplets %u i2cbridges %u loop %d\n",topo_numnodes(), topo_numtriplets(), topo_numi2cbridges(), topo_loop() );
      umwhite_state=UMWHITE_STATE_SETUP;
      break;

    case UMWHITE_STATE_SETUP:
      umwhite_error= topo_stdsetup();
      if( umwhite_error!=OSP2_ERROR_NONE ) { umwhite_state=UMWHITE_STATE_ERROR; return 1; }
      umwhite_error= umwhite_run_start();
      if( umwhite_error!=OSP2_ERROR_NONE ) { umwhite_state=UMWHITE_STATE_ERROR; return 1; }
      umwhite_state=UMWHITE_STATE_RUN;
      break;

    case UMWHITE_STATE_RUN :
      umwhite_error= umwhite_run_step();
      if( umwhite_error!=OSP2_ERROR_NONE ) { umwhite_state=UMWHITE_STATE_ERROR; return 1; }
      break;

    case UMWHITE_STATE_ERROR :
      // stay in error state
      printf("umwhite: ERROR %d %s\n",umwhite_error,osp2_error_str(umwhite_error));
      break;
  }
  return umwhite_state==UMWHITE_STATE_ERROR;
}


void umwhite_stop() {
  // nothing to shutdown
}


// ==========================================================================
// Within the RUN state, there is a sub state machine;
// it has its own reset and (micro) step function.


// For all SAIDs, set the dithering flag of its three channels
static osp2_error_t umwhite_set_dither( int dither) {
  uint8_t flags = dither ? OSP2_CURCHN_FLAGS_DITHER : 0;
  osp2_error_t err;
  for( uint16_t addr=1; addr<=topo_numnodes(); addr++ ) {
    if( topo_node_id(addr) == TOPO_ID_SAID ) {
      err= osp2_send_setcurchn(addr, 0, flags, 3, 3, 3); // 3x 24mA for channel 0 (is higher powered)
      if( err!=OSP2_ERROR_NONE ) return err;
      err=osp2_send_setcurchn(addr, 1, flags, 4, 4, 4); // 3x 24mA for channel 1
      if( err!=OSP2_ERROR_NONE ) return err;
      if( topo_node_numtriplets(addr)==3 ) {
        err=osp2_send_setcurchn(addr, 2, flags, 4, 4, 4); // 3x 24mA for channel 2
        if( err!=OSP2_ERROR_NONE ) return err;
      }
    }
  }
  return OSP2_ERROR_NONE;
}


// For all triplets, r, g, and b will be set to `brightness`
static osp2_error_t umwhite_set_brightness( int brightness) {
  topo_rgb_t rgb = { brightness, brightness, brightness };

  for( uint16_t tix=0; tix<topo_numtriplets(); tix++ ) {
    osp2_error_t err= topo_settriplet(tix, &rgb);
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  return OSP2_ERROR_NONE;
}


#define UMWHITE_STEP_MS         10     // ms per step frame
#define UMWHITE_REPAIR_MS       200    // ms per repair
#define UMWHITE_STEP_SIZE       4      // 0xCCC/4*10ms = 8.2s
#define UMWHITE_BRIGHTNESS_MAX  0x0CCC // TOPO_BRIGHTNESS_MAX is too high for USB power


static int       umwhite_run_dir;        // -1=dimdown, 0=stopped, +1=dimup
static uint32_t  umwhite_run_lastms;     // last time stamp (in ms) a micro step was taken
static uint32_t  umwhite_run_repairms;   // last time stamp (in ms) a repair telegram was sent
static int       umwhite_run_brightness; // brightness level
static int       umwhite_run_dither;     // dither state


static osp2_error_t umwhite_run_start() {
  osp2_error_t err;
  // Select state
  umwhite_run_dir= +1;
  umwhite_run_lastms= get_system_time_ms();
  umwhite_run_repairms= get_system_time_ms();
  umwhite_run_brightness= 0;
  umwhite_run_dither= 1;
  // Effectuate state
  err= umwhite_set_dither(umwhite_run_dither);
  if( err!=OSP2_ERROR_NONE ) return err;
  err= umwhite_set_brightness(umwhite_run_brightness); // umwhite_run_brightness is actual brightness, we pre-increment
  if( err!=OSP2_ERROR_NONE ) return err;
  return OSP2_ERROR_NONE;
}


static osp2_error_t umwhite_run_step() {
  osp2_error_t err;
  uint32_t now= get_system_time_ms();
  
  if( umwhite_run_dir==0 ) {
    // stopped mode, toggle dithering when SW2 is pressed
    if( one_time_button_pressed_sw3() ) {
      umwhite_run_dither = ! umwhite_run_dither;
      printf("umwhite: dither %d\n",umwhite_run_dither);
      err= umwhite_set_dither(umwhite_run_dither);
      if( err!=OSP2_ERROR_NONE ) return err; 
    }
  } else { 
    // dimup or dim down mode - is stopped when SW2 is pressed
    if( one_time_button_pressed_sw3() ) {
      umwhite_run_dir= 0; // go to stopped state
      printf("umwhite: stopped\n");
    } else if( now - umwhite_run_lastms > UMWHITE_STEP_MS ) {
      // time to make a dim step
      umwhite_run_lastms = now;
      // pre-increment umwhite_run_brightness
      int new_brightness = umwhite_run_brightness + umwhite_run_dir * UMWHITE_STEP_SIZE;
      if( 0<=new_brightness && new_brightness<=UMWHITE_BRIGHTNESS_MAX ) {
        umwhite_run_brightness = new_brightness;
      } else {
        umwhite_run_dir= - umwhite_run_dir;
        umwhite_run_brightness= umwhite_run_brightness + umwhite_run_dir * UMWHITE_STEP_SIZE;
      }
      // Effectuate the brightness
      err= umwhite_set_brightness(umwhite_run_brightness); // umwhite_run_brightness is actual brightness, we pre-increment
      if( err!=OSP2_ERROR_NONE ) return err;
    }
  }
  
  if( now - umwhite_run_repairms > UMWHITE_REPAIR_MS ) {
    // time to make a repair
    umwhite_run_repairms = now;
    // repair - Just in case there was and error (under voltage error) in some node broadcast clear all and broadcast switch back on
    err= osp2_send_clrerror(0);
    if( err!=OSP2_ERROR_NONE ) return err;
    err=osp2_send_goactive(0);
    if( err!=OSP2_ERROR_NONE ) return err;
   }  
  
  return OSP2_ERROR_NONE;
}

