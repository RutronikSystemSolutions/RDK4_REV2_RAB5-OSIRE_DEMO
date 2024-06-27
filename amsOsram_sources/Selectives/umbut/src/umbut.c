// umbut.c - User Mode controlling changing LEDs when pressing local buttons
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
- Shows one (static) flag at a time
- Tries to find a SAID with an I2C bridge with an IO-expander (with four buttons and four signaling LEDs)
- If there is no IO expander, shows four static flags switching on a time basis
- If there is an IO expander the four buttons, select which flag to show
- The signaling LED shows indicates which button/flag was selected
- If there are multiple IO-expanders the first one is taken
- Does not use SW2

NOTES
- When the app quits, the signaling LED switches off

GOAL
- To show a "sensor" (button) being access by the root MCU
*/

#include <stdio.h>
#include <string.h>
#include <SwTimer/inc/swTimer.h>
#include <osp2/inc/osp2.h>
#include <topo/inc/topo.h>
#include <flag/inc/flag.h>
#include <iox/inc/iox.h>
#include <umbut/inc/umbut.h>
#include "sys_timer.h"


// ==========================================================================
// Main state machine


// Within the RUN state, there is a sub state machine; it has its own start and (micro) step function.
static osp2_error_t umbut_run_start();
static osp2_error_t umbut_run_step();


// State tells what is to do next
typedef enum umbut_state_e {
  UMBUT_STATE_INIT,  // Send RESET and INIT (BIDIR/LOOP) telegrams, then scan topology
  UMBUT_STATE_SETUP, // Setup individual nodes for the demo
  UMBUT_STATE_RUN,   // Demo running step by step
  UMBUT_STATE_ERROR, // Terminal state when an error is detected (that error is recorded in umbut_error)
} umbut_state_t;


static osp2_error_t  umbut_error;    // last error
static umbut_state_t umbut_state;    // current state


// Resets internal state of user mode button/flag animation
void umbut_start() {
  umbut_error= OSP2_ERROR_NONE;
  umbut_state= UMBUT_STATE_INIT;
}


// Returns 1 on error
int umbut_step() {
  switch( umbut_state ) {

    case UMBUT_STATE_INIT :
      umbut_error= topo_stdinit();
      if( umbut_error!=OSP2_ERROR_NONE ) { umbut_state=UMBUT_STATE_ERROR; return 1; }
      printf("umbut: nodes %u triplets %u i2cbridges %u loop %d\n",topo_numnodes(), topo_numtriplets(), topo_numi2cbridges(), topo_loop() );
      umbut_state=UMBUT_STATE_SETUP;
      break;

    case UMBUT_STATE_SETUP:
      umbut_error= topo_stdsetup();
      if( umbut_error!=OSP2_ERROR_NONE ) { umbut_state=UMBUT_STATE_ERROR; return 1; }
      umbut_error= umbut_run_start();
      if( umbut_error!=OSP2_ERROR_NONE ) { umbut_state=UMBUT_STATE_ERROR; return 1; }
      umbut_state=UMBUT_STATE_RUN;
      break;

    case UMBUT_STATE_RUN :
      umbut_error= umbut_run_step();
      if( umbut_error!=OSP2_ERROR_NONE ) { umbut_state=UMBUT_STATE_ERROR; return 1; }
      break;

    case UMBUT_STATE_ERROR :
      // stay in error state
      printf("umbut: ERROR %d %s\n",umbut_error,osp2_error_str(umbut_error));
      break;
  }
  return umbut_state==UMBUT_STATE_ERROR;
}


void umbut_stop() {
  // Shut down signaling LEDs
  iox_led_set( IOX_LEDNONE );
}


// ==========================================================================
// Within the RUN state, there is a sub state machine;
// it has its own reset and (micro) step function.


#define UMBUT_STEP_MS 2000


// We have a list of flag painters
typedef osp2_error_t (*umbut_painter_t)();
const umbut_painter_t umbut_flags[] = { &flag_dutch, &flag_mali, &flag_europe, &flag_china };
#define UMBUT_NUMFLAGS ( sizeof(umbut_flags)/sizeof(umbut_painter_t) )

static int      umbut_run_flagix; // index of flag being shown
static int      umbut_ioxpresent; // the IOExpander is present (if not, flags auto change every UMBUT_STEP_MS)
static uint32_t umbut_run_lastms; // last time stamp (in ms) a flag was shown (for auto change)


static osp2_error_t umbut_run_start() {
  osp2_error_t err;

  umbut_run_flagix= 0; // pick some flag
  umbut_ioxpresent = ( topo_numi2cbridges() > 0 ) && ( iox_present(topo_i2cbridge_addr(0)) == OSP2_ERROR_NONE ); // todo: not first but search
  printf("umbut: %s\n", umbut_ioxpresent ? "iox found" : "no iox, auto change");

  if( umbut_ioxpresent ) {
    err= iox_init( topo_i2cbridge_addr(0) ); // take first SAID as the one with the IOX
    if( err!=OSP2_ERROR_NONE ) return err;
    iox_led_set( IOX_LED(umbut_run_flagix) ); // highlight the associated signaling LED
  }

  err= umbut_flags[umbut_run_flagix](); // draw the flag
  if( err!=OSP2_ERROR_NONE ) return err;

  umbut_run_lastms= get_system_time_ms();
  return OSP2_ERROR_NONE;
}


static osp2_error_t umbut_run_step() {
  osp2_error_t err;

  // Check if a button is pressed
  int prev_painterix= umbut_run_flagix;

  if( umbut_ioxpresent ) {
    // check if a button on the IO expander is pressed
    err= iox_but_poll();
    if( err!=OSP2_ERROR_NONE ) return err;
    if( iox_but_wentdown(IOX_BUT0) ) umbut_run_flagix= 0;
    if( iox_but_wentdown(IOX_BUT1) ) umbut_run_flagix= 1;
    if( iox_but_wentdown(IOX_BUT2) ) umbut_run_flagix= 2;
    if( iox_but_wentdown(IOX_BUT3) ) umbut_run_flagix= 3;
  } else {
    // auto change
    uint32_t now= get_system_time_ms();
    if( now - umbut_run_lastms >= UMBUT_STEP_MS ) {
      umbut_run_flagix = (umbut_run_flagix+1) % UMBUT_NUMFLAGS;
      umbut_run_lastms = now;
    }
  }

  // If a new flag selected, paint it
  if( umbut_run_flagix!=prev_painterix ) {
    err= umbut_flags[umbut_run_flagix](); // draw new flag
    if( err!=OSP2_ERROR_NONE ) return err;
    if( umbut_ioxpresent ) {
      err= iox_led_set( IOX_LED(umbut_run_flagix) ); // highlight new signaling LED
      if( err!=OSP2_ERROR_NONE ) return err;
    }
    prev_painterix = umbut_run_flagix;
  }

  // Just in case there was and error (under voltage error) in some node broadcast clear all and broadcast switch back on
  err= osp2_send_clrerror(0);
  if( err!=OSP2_ERROR_NONE ) return err;
  err=osp2_send_goactive(0);
  if( err!=OSP2_ERROR_NONE ) return err;

  return OSP2_ERROR_NONE;
}



