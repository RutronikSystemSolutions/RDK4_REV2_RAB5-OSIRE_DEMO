// umled.c - User Mode controlling a LED string (OSIRE and SAID) of arbitrary length
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
- Has a "virtual cursor" that runs from begin of chain to end of chain and then back
- Every 25ms the cursor advances one LED and paints that in the current color
- Every time the cursor hits the begin or end of the chain, it steps color 
- Color palette: red, yellow, green, cyan, blue, magenta, white
- Does not use SW2

Goal
- To show that various OSP nodes can be mixed and have color/brightness matched
*/


#include <stdio.h>
#include <string.h>
#include <SwTimer/inc/swTimer.h>
#include <osp2/inc/osp2.h>
#include <topo/inc/topo.h>
#include <umled/inc/umled.h>
#include "sys_timer.h"


// ==========================================================================
// Main state machine


// Within the RUN state, there is a sub state machine; it has its own start and (micro) step function.
static osp2_error_t umled_run_start();
static osp2_error_t umled_run_step();


// State tells what is to do next
typedef enum umled_state_e {
  UMLED_STATE_INIT,  // Send RESET and INIT (BIDIR/LOOP) telegrams, then scan topology
  UMLED_STATE_SETUP, // Setup individual nodes for the demo
  UMLED_STATE_RUN,   // Demo running step by step
  UMLED_STATE_ERROR, // Terminal state when an error is detected (that error is recorded in umled_error)
} umled_state_t;


static osp2_error_t  umled_error;    // last error
static umled_state_t umled_state;    // current state


// Resets internal state of user mode led animation
void umled_start() {
  umled_error= OSP2_ERROR_NONE;
  umled_state= UMLED_STATE_INIT;
}


// Returns 1 on error
int umled_step() {
  switch( umled_state ) {

    case UMLED_STATE_INIT :
      umled_error= topo_stdinit();
      if( umled_error!=OSP2_ERROR_NONE ) { umled_state=UMLED_STATE_ERROR; return 1; }
      printf("umled: nodes %u triplets %u i2cbridges %u loop %d\n",topo_numnodes(), topo_numtriplets(), topo_numi2cbridges(), topo_loop() );
      umled_state=UMLED_STATE_SETUP;
      break;

    case UMLED_STATE_SETUP:
      umled_error= topo_stdsetup();
      if( umled_error!=OSP2_ERROR_NONE ) { umled_state=UMLED_STATE_ERROR; return 1; }
      umled_error= umled_run_start();
      if( umled_error!=OSP2_ERROR_NONE ) { umled_state=UMLED_STATE_ERROR; return 1; }
      umled_state=UMLED_STATE_RUN;
      break;

    case UMLED_STATE_RUN :
      umled_error= umled_run_step();
      if( umled_error!=OSP2_ERROR_NONE ) { umled_state=UMLED_STATE_ERROR; return 1; }
      break;

    case UMLED_STATE_ERROR :
      // stay in error state
      printf("umled: ERROR %d %s\n",umled_error,osp2_error_str(umled_error));
      break;
  }
  return umled_state==UMLED_STATE_ERROR;
}


void umled_stop() {
  // nothing to shutdown
}

// ==========================================================================
// Within the RUN state, there is a sub state machine;
// it has its own reset and (micro) step function.


#define UMLED_STEP_MS 25


// The colors in the loop
static const topo_rgb_t  * const umled_rgbs[] = { &topo_red, &topo_orange, &topo_green, &topo_cyan, &topo_blue, &topo_magenta, &topo_white };
//#define UMLED_NUMRGBS ( sizeof(umled_rgbs)/sizeof(topo_rgb_t) )
#define UMLED_NUMRGBS 7


static uint32_t      umled_run_lastms; // last time stamp (in ms) a micro step was taken
static int           umled_run_tix;    // triplet index
static int           umled_run_cix;    // color index
static int           umled_run_dir;    // direction -1 or +1


static osp2_error_t umled_run_start() {
  umled_run_lastms= get_system_time_ms()-UMLED_STEP_MS; // falsify last step.run time-stamp to get first micro step immediately
  umled_run_cix= 0;
  umled_run_tix= 0;
  umled_run_dir= +1;
  return OSP2_ERROR_NONE;
}

static osp2_error_t umled_run_step() {
  uint32_t now= get_system_time_ms();
  if( now - umled_run_lastms < UMLED_STEP_MS ) return OSP2_ERROR_NONE;
  umled_run_lastms = now;

  // Set triplet tix to color cix
  osp2_error_t err;
  err= topo_settriplet(umled_run_tix, umled_rgbs[umled_run_cix] );
  if( err!=OSP2_ERROR_NONE ) return err;

  // Go to next triplet
  int new_tix = umled_run_tix + umled_run_dir;
  if( 0<=new_tix && new_tix<topo_numtriplets() ) {
    umled_run_tix= new_tix;
  } else  { // hit either end
    // reverse direction and step color
    umled_run_dir = -umled_run_dir;
    umled_run_cix += 1;
    if( umled_run_cix==UMLED_NUMRGBS ) {
      umled_run_cix= 0;
    }
    // Just in case there was and error (under voltage error) in some node broadcast clear all and broadcast switch back on
    err= osp2_send_clrerror(0);
    if( err!=OSP2_ERROR_NONE ) return err;
    err=osp2_send_goactive(0);
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  return OSP2_ERROR_NONE;
}

