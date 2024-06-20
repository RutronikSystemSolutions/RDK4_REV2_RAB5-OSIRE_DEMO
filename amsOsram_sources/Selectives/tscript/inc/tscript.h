// tscript.h - tiny script to animate a series of rgb triplets using (tiny) instruction
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
#ifndef TSCRIPT_H_
#define TSCRIPT_H_

#ifdef __cplusplus
extern "C"
  {
#endif


#include <stdint.h>
#include <topo/inc/topo.h>



// ==========================================================================
// One instruction; it sets a consecutive series of triplets region to one color.
typedef struct tscript_inst_s {
  int         cursor;   // index into script
  uint16_t    code;     // original code at cursor
  bool        atend;    // end-marker
  bool        withprev; // this instruction should be combined with previous (or starts a new frame)
  uint16_t    tix0;     // start of the region (inclusive)
  uint16_t    tix1;     // end of the region (exclusive)
  topo_rgb_t  rgb;      // color for the region ("demo brightness range" 0..0x7FFF)
} tscript_inst_t;


// ==========================================================================
// The core functions: an iterator on a series of int16_t (the instructions)

bool                   tscript_atend();     // Internal cursor is at the instruction that marks the end
void                   tscript_gotofirst(); // Move internal cursor to first instruction
void                   tscript_gotonext();  // Move internal cursor to next instruction (except when atend, the cursor does not move)
const tscript_inst_t * tscript_get();       // gets instruction (all fields decoded)
void                   tscript_install(const uint16_t *insts, uint16_t numtriplets); // Installs a new script; numtriplets is needed to scale the region indices.


// ==========================================================================
// High level helpers
osp2_error_t tscript_playinst();  // Plays instruction the internal cursor points at; precondition: ! atend() - does not gotonext()
osp2_error_t tscript_playframe(); // Plays instruction the internal cursor points at; if next is tagged as "with prev" also executes, and so on. Does gotonext(); if there is the end marker, wraps around


// ==========================================================================
// Stock animation scripts
const uint16_t * tscript_rainbow();
int              tscript_rainbow_bytes();
const uint16_t * tscript_bouncingblock();
int              tscript_bouncingblock_bytes();
const uint16_t * tscript_colormix();
int              tscript_colormix_bytes();
const uint16_t * tscript_heartbeat();
int              tscript_heartbeat_bytes();


#ifdef __cplusplus
}
#endif

#endif  // TSCRIPT_H_





