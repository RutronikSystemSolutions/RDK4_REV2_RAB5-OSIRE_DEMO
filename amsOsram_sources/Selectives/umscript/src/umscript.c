// umscript.c - User Mode controlling a LED string (OSIRE and SAID) via an EEPROM script
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
- Plays a light shows as defined by an animation script
- Tries to find a SAID with an I2C bridge with an EEPROM
- If there is an external EEPROM (I2C address 0x51) it favors that over an internal EEPROM (address 0x50)
- If there are multiple (of the same kind, external or internal) the first one is taken
- If no EEPROM is found, uses the heartbeat script included in the firmware
- If an EEPROM is found, loads the script from the EEPROM and plays that
- The internal EEPROM (on the SAID basic board) contains the rainbow script
- External EEPROMs are flashed with bouncing-block and color-mix
- Does not use SW2

NOTES
- Ensure the I2C EEPROM stick faces "chip up" otherwise there is a short circuit (see PCB labels)
- Safest is to only swap an EEPROM when USB power is removed
- On your own risk when swapping life
- The script loading takes place when starting the app, so either (1) power up, (2) reset, (3) SW3.

GOAL
- Show that the root MCU can access I2C devices (EEPROM) e.g. for calibration values
*/

#include <stdio.h>
#include <string.h>
#include <SwTimer/inc/swTimer.h>
#include <osp2/inc/osp2.h>
#include <topo/inc/topo.h>
#include <eeprom/inc/eeprom.h>
#include <tscript/inc/tscript.h>
#include <umscript/inc/umscript.h>
#include "sys_timer.h"


// ==========================================================================
// Main state machine


// Within the RUN state, there is a sub state machine; it has its own start and (micro) step function.
static osp2_error_t umscript_run_start();
static osp2_error_t umscript_run_step();


// State tells what is to do next
typedef enum umscript_state_e {
  UMSCRIPT_STATE_INIT,  // Send RESET and INIT (BIDIR/LOOP) telegrams, then scan topology
  UMSCRIPT_STATE_SETUP, // Setup individual nodes for the demo
  UMSCRIPT_STATE_RUN,   // Demo running step by step
  UMSCRIPT_STATE_ERROR, // Terminal state when an error is detected (that error is recorded in umscript_error)
} umscript_state_t;


static osp2_error_t  umscript_error;    // last error
static umscript_state_t umscript_state;    // current state


// Resets internal state of user mode button/flag animation
void umscript_start() {
  umscript_error= OSP2_ERROR_NONE;
  umscript_state= UMSCRIPT_STATE_INIT;
}


// Returns 1 on error
int umscript_step() {
  switch( umscript_state ) {

    case UMSCRIPT_STATE_INIT :
      umscript_error= topo_stdinit();
      if( umscript_error!=OSP2_ERROR_NONE ) { umscript_state=UMSCRIPT_STATE_ERROR; return 1; }
      printf("umscript: nodes %u triplets %u i2cbridges %u loop %d\n",topo_numnodes(), topo_numtriplets(), topo_numi2cbridges(), topo_loop() );
      umscript_state=UMSCRIPT_STATE_SETUP;
      break;

    case UMSCRIPT_STATE_SETUP:
      umscript_error= topo_stdsetup();
      if( umscript_error!=OSP2_ERROR_NONE ) { umscript_state=UMSCRIPT_STATE_ERROR; return 1; }
      umscript_error= umscript_run_start();
      if( umscript_error!=OSP2_ERROR_NONE ) { umscript_state=UMSCRIPT_STATE_ERROR; return 1; }
      umscript_state=UMSCRIPT_STATE_RUN;
      break;

    case UMSCRIPT_STATE_RUN :
      umscript_error= umscript_run_step();
      if( umscript_error!=OSP2_ERROR_NONE ) { umscript_state=UMSCRIPT_STATE_ERROR; return 1; }
      break;

    case UMSCRIPT_STATE_ERROR :
      // stay in error state
      printf("umscript: ERROR %d %s\n",umscript_error,osp2_error_str(umscript_error));
      break;
  }
  return umscript_state==UMSCRIPT_STATE_ERROR;
}


void umscript_stop() {
  // nothing to shut down
}


// ==========================================================================
// Load instructions from EEPROM


#define UMSCRIP_ANIM_MAXNUMINST 128 // maximum number of instructions in an animation script
static uint16_t umscript_insts[UMSCRIP_ANIM_MAXNUMINST]; // List of instructions ("the script")


// Load script from EEPROM
static osp2_error_t umscript_insts_load() {
  osp2_error_t err;

  // Find an EEPROM
  uint16_t saidaddr; // OSP address of SAID with EEPROM
  uint8_t  eepromaddr; // 7-bit I2C address of the EEPROM
  saidaddr= topo_findi2cdev(EEPROM_EXTERNAL);
  if( saidaddr!=0 ) {
    eepromaddr= EEPROM_EXTERNAL;
  } else {
    saidaddr= topo_findi2cdev(EEPROM_INTERNAL);
    if( saidaddr!=0 ) {
      eepromaddr= EEPROM_INTERNAL;
    } else {
      eepromaddr = 0; // No SAID, so no EEPROM present
    }
  }

  // Now load the script from EEPROM
  uint16_t numtriplets = topo_numtriplets();
  if( eepromaddr==0 ) {
    // no EEPROM found, use built-in script
    printf("umscript: no EEPROM, playing from rom 'heartbeat' animation\n");
    tscript_install( tscript_heartbeat(), numtriplets);
  } else {
    if( sizeof(uint8_t[4]) != sizeof(uint16_t[2]) ) return OSP2_ERROR_DATA; // Hack: use array of sized n of uint16_t as array sized 2n of uint8_t (compiler might pad) (endianess ignored since we read and write with same processor)
    err= eeprom_read(saidaddr, eepromaddr, 0, (uint8_t*)umscript_insts, UMSCRIP_ANIM_MAXNUMINST*2 );
    if( err!=OSP2_ERROR_NONE ) return err;
    // Pass EEPROM content to tiny script
    printf("umscript: playing from SAID %04x EEPROM %02x\n",saidaddr, eepromaddr);
    tscript_install( umscript_insts, numtriplets);
  }

  return OSP2_ERROR_NONE;
}


// ==========================================================================
// Within the RUN state, there is a sub state machine;
// it has its own reset and (micro) step function.


#define UMSCRIPT_STEP_MS 100


static uint32_t umscript_run_lastms; // last time stamp (in ms) a micro step was taken


static osp2_error_t umscript_run_start() {
  osp2_error_t err;

  err= umscript_insts_load();
  if( err!=OSP2_ERROR_NONE ) return err;

  umscript_run_lastms= get_system_time_ms()-UMSCRIPT_STEP_MS; // falsify last step.run time-stamp to get first micro step immediately
  return OSP2_ERROR_NONE;
}


static osp2_error_t umscript_run_step() {
  osp2_error_t err;

  uint32_t now= get_system_time_ms();
  if( now - umscript_run_lastms < UMSCRIPT_STEP_MS ) return OSP2_ERROR_NONE;
  umscript_run_lastms = now;

  // Create next frame
  err= tscript_playframe();
  if( err!=OSP2_ERROR_NONE ) return err;

  // Just in case there was and error (under voltage error) in some node broadcast clear all and broadcast switch back on
  err= osp2_send_clrerror(0);
  if( err!=OSP2_ERROR_NONE ) return err;
  err=osp2_send_goactive(0);
  if( err!=OSP2_ERROR_NONE ) return err;

  return OSP2_ERROR_NONE;
}


