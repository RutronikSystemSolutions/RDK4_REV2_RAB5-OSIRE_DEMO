// tscript.c - tiny script to animate a series of rgb triplets using (tiny) instruction
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
#include <tscript/inc/tscript.h>


/*
A script consist of a number of instructions. One instruction sets a region,
a consecutive series of RGB triplets, to one color. For example, an instruction
could set triplets 1, 2, 3 to red. Another instruction could set triplets
4, 5, 6 to white and yet another  could set 7, 8, 9 to blue. An instruction
has a flag "with previous". So if, in the above example, the first instruction
(red) does not have the "with previous" flag set, but in the second (white)
and the third (blue) do have that flag set, the three instructions together
make one frame drawing the Red/White/Blue flag on triplets 1 to 9.

There is one complication, a script runs on a chain of any length (any number
of triplets). Therefore the start and end index of the region in the instruction
mapped to the number of triplets.

A script needs to be stored on a 256 bytes EEPROM, so everything about this
script is "tiny". Each instruction is 16 bit, and the start and end index are
only 3 bits each. In other words, there are only 8 regions.

Similarly, the red, green and blue brightness, which a typical OSP chain can
set with an accuracy of 15 bits, is also stored in 3 bits each. as a result,
an instruction is coded as follows:

  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
  | 15 | 14| 13| 12| 11| 10| 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
  |with| start of  |  end of   |    red    |   green   |   blue    |
  |prev|  region   |  region   | brightness| brightness| brightness|
  +----+-----------+-----------+-----------+-----------+-----------+

The regions are linearly distributed over all triplets, and the brightness
levels are exponentially scaled (currently to about 0x1000, not 0xEFFF because
that uses too much power for USB).

Note that each field of an instruction is 3 bits, so instructions
are relatively readable when coding them in octal. Let's have a look at an
example:

  0007007,
  0166100,
  0070000,

The first instruction starts with 0, C-syntax for octal. We strip that and
break the rest in pieces 0 07 007. The last three digits is 007 so red 0,
green 0 and blue 7, so brightest blue. The two digits before that 07 denote
the region, here region 0 to 7, which means the whole chain. The leading 0
means not with previous, so this starts a chain.

The second instruction is 1 66 100 (dropping the leading octal 0).
Here the color is lowest red, and the region is only number 6. The
with previous is set, so this instruction belongs to the same frame as the
first instruction. The third instruction starts a new frame; it has
"with previous" not set.

If the chain would be 16 long it would look as follows
  0 1 2 3 4 5 6 7 8 9101112131415  triplet index
  0 0 1 1 2 2 3 3 4 4 5 5 6 6 7 7  region index
  B B B B B B B B B B B B r r B B  resulting frame

The third instruction is special. The region runs from 7 to 0. This is not
legal. As a result, this instruction means end-of-script.
*/


// ==========================================================================
// Brightness lookup table, not to bright because fed from USB power
// for i in range(7) : print( hex(int(0x100*1.5**i)) )
uint16_t tscript_brightness[8] = {
  0x0000,//0
  0x0100,//1
  0x0180,//2
  0x0240,//3
  0x0360,//4
  0x0510,//5
  0x0798,//6
  0x0b64,//7
};


// ==========================================================================
// The core functions: an iterator on a series of int16_t (the instructions)


static const uint16_t * tscript_insts;      // List of instructions ("the script")
static int              tscript_cursor;     // Index of first instruction to play
static tscript_inst_t   tscript_inst;       // decode instruction under the cursor
static int              tscript_numtriplets;// Multiplier to go from triplet index 0..7 in instruction to triplet index in actual chain


static void tscript_decode( ) {
  // Helpers to slice bits form an instruction
  #define BITS_MASK(n)                  ((1<<(n))-1)                           // number of bits set BITS_MASK(3)=0b111 (max n=31)
  #define BITS_SLICE(v,lo,hi)           ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) ) // including lo, excluding hi
  // Get current instruction
  uint16_t code = tscript_insts[tscript_cursor];
  uint16_t tix0 = BITS_SLICE(code,12,15);
  uint16_t tix1 = BITS_SLICE(code, 9,12);
  // Get the instruction parts
  tscript_inst.cursor   = tscript_cursor;
  tscript_inst.code     = code;
  tscript_inst.atend    = tix0>tix1;
  tscript_inst.withprev = BITS_SLICE(code,15,16);
  tscript_inst.tix0     = (  tix0    * tscript_numtriplets + 4 ) / 8;
  tscript_inst.tix1     = ( (tix1+1) * tscript_numtriplets + 4 ) / 8;
  if( tscript_inst.tix1>tscript_numtriplets ) tscript_inst.tix1= tscript_numtriplets;
  tscript_inst.rgb.r    = tscript_brightness[ BITS_SLICE(code,6,9) ];
  tscript_inst.rgb.g    = tscript_brightness[ BITS_SLICE(code,3,6) ];
  tscript_inst.rgb.b    = tscript_brightness[ BITS_SLICE(code,0,3) ];
}


// Internal cursor is at the instruction that marks the end
bool tscript_atend() {
  return tscript_inst.atend;
}


// Move internal cursor to first instruction
void tscript_gotofirst() {
  tscript_cursor= 0;
  tscript_decode();
}


// Move internal cursor to next instruction (except when atend, the cursor does not move)
void tscript_gotonext() {
  if( !tscript_atend() ) tscript_cursor++;
  tscript_decode();
}


// Get pointer to current instruction
const tscript_inst_t * tscript_get() {
  return &tscript_inst;
}


// Installs a new script; numtriplets is needed to scale the region indices.
void tscript_install(const uint16_t *insts, uint16_t numtriplets) {
  tscript_insts= insts;
  tscript_numtriplets= numtriplets;
  tscript_gotofirst();
}


// ==========================================================================
// High level helpers


// Plays instruction the internal cursor points at; precondition: ! atend() - does not gotonext()
osp2_error_t tscript_playinst() {
  // Use internal `tscript_inst` instead of public `tscript_get()`
  // dbg_printf("#%d 0o%06o : %d [%d,%d) %04x.%04x.%04x\n", tscript_cursor,tscript_insts[tscript_cursor],   tscript_inst.withprev,  tscript_inst.tix0,tscript_inst.tix1,  tscript_inst.rgb.r,tscript_inst.rgb.g,tscript_inst.rgb.b );
  for( uint16_t tix=tscript_inst.tix0; tix<tscript_inst.tix1; tix++ ) {
    osp2_error_t err= topo_settriplet(tix, &tscript_inst.rgb );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  return OSP2_ERROR_NONE;
}


// Plays instruction the internal cursor points at; if next is tagged as "with prev" also executes, and so on.
// Does gotonext(); if there is the end marker, wraps around
osp2_error_t tscript_playframe() {
  if( tscript_atend() ) tscript_gotofirst();
  int n=1;
  do {
    if( n>8 ) return OSP2_ERROR_DATA; // can not have more then 8 withprev, because there are only 8 segments
    osp2_error_t err= tscript_playinst();
    if( err!=OSP2_ERROR_NONE ) return err;
    tscript_gotonext();
    n++;
  } while( tscript_get()->withprev );
  return OSP2_ERROR_NONE;
}


// ==========================================================================
// Stock animation scripts


static const uint16_t tscript_rainbow_[] = {
// Octal 0, 0 or 1 for with previous, 0..7 for lower index, 0..7 for upper index, 0..7 for red, 0..7 for green and 0..7 for blue
//oPLURGB

  // From all black to all white
  0007000,
  0007111,
  0007222,
  0007333,
  0007444,
  0007555,
  0007666,
  0007777,

  // All bands up
  // Segment 1 from white to red
  0011766,
  0011755,
  0011744,
  0011733,
  0011722,
  0011711,
  0011700,
  // Segment 2 from white to yellow
  0022776,
  0022775,
  0022774,
  0022773,
  0022772,
  0022771,
  0022770,
  // Segment 3 from white to green
  0033676,
  0033575,
  0033474,
  0033373,
  0033272,
  0033171,
  0033070,
  // Segment 4 from white to cyan
  0044677,
  0044577,
  0044477,
  0044377,
  0044277,
  0044177,
  0044077,
  // Segment 5 from white to blue
  0055667,
  0055557,
  0055447,
  0055337,
  0055227,
  0055117,
  0055007,
  // Segment 6 from white to purple
  0066767,
  0066757,
  0066747,
  0066737,
  0066727,
  0066717,
  0066707,

  // All bands down
  // Segment 0 from white to black
  0000666,
  0000555,
  0000444,
  0000333,
  0000222,
  0000111,
  0000000,
  // Segment 1 from red to black
  0011600,
  0011500,
  0011400,
  0011300,
  0011200,
  0011100,
  0011000,
  // Segment 2 from yellow to black
  0022660,
  0022550,
  0022440,
  0022330,
  0022220,
  0022110,
  0022000,
  // Segment 3 from green to black
  0033060,
  0033050,
  0033040,
  0033030,
  0033020,
  0033010,
  0033000,
  // Segment 4 from cyan to black
  0044066,
  0044055,
  0044044,
  0044033,
  0044022,
  0044011,
  0044000,
  // Segment 5 from blue to black
  0055006,
  0055005,
  0055004,
  0055003,
  0055002,
  0055001,
  0055000,
  // Segment 6 from purple to black
  0066606,
  0066505,
  0066404,
  0066303,
  0066202,
  0066101,
  0066000,
  // Segment 7 from white to black
  0077666,
  0077555,
  0077444,
  0077333,
  0077222,
  0077111,
  0077000,

  // End
  0070000,
};


const uint16_t * tscript_rainbow() {
  return tscript_rainbow_;
}

int tscript_rainbow_bytes() {
  return sizeof(tscript_rainbow_);
}


static const uint16_t tscript_bouncingblock_[] = {
// Octal 0, 0 or 1 for with previous, 0..7 for lower index, 0..7 for upper index, 0..7 for red, 0..7 for green and 0..7 for blue
//oPLURGB

  // Red block moving left to right (1) on blue background (7)
  0007007,
  0166100,

  0007007,
  0166100,

  0007007,
  0155100,

  0007007,
  0144100,

  0007007,
  0133100,

  0007007,
  0122100,

  0007007,
  0111100,

  0007007,
  0100100,

  // script was too long so skipping red=2,bg=6

  // Red block moving left to right (3) on blue background (5)
  0007005,
  0100300,

  0007005,
  0111300,

  0007005,
  0122300,

  0007005,
  0133300,

  0007005,
  0144300,

  0007005,
  0155300,

  0007005,
  0166300,

  0007005,
  0177300,

  // Red block moving back, more red (4), less blue (4)
  0007004,
  0177400,

  0007004,
  0166400,

  0007004,
  0155400,

  0007004,
  0144400,

  0007004,
  0133400,

  0007004,
  0122400,

  0007004,
  0111400,

  0007004,
  0100400,

  // Red block moving left to right (5) on blue background (3)
  0007003,
  0100500,

  0007003,
  0111500,

  0007003,
  0122500,

  0007003,
  0133500,

  0007003,
  0144500,

  0007003,
  0155500,

  0007003,
  0166500,

  0007003,
  0177500,

  // Red block moving back, more red (6), less blue (2)
  0007002,
  0177600,

  0007002,
  0166600,

  0007002,
  0155600,

  0007002,
  0144600,

  0007002,
  0133600,

  0007002,
  0122600,

  0007002,
  0111600,

  0007002,
  0100600,

  // Red block moving left to right (7) on blue background (1)
  0007001,
  0100700,

  0007001,
  0111700,

  0007001,
  0122700,

  0007001,
  0133700,

  0007001,
  0144700,

  0007001,
  0155700,

  0007001,
  0166700,

  0007001,
  0177700,

  // Red block moving back, more red (7) erasing blue
  0007000,
  0177700,

  0007000,
  0166700,

  0007000,
  0155700,

  0007000,
  0144700,

  0007000,
  0133700,

  0007000,
  0122700,

  0007000,
  0111700,

  0007000,
  0100700,

  // End
  0070000,
};


const uint16_t * tscript_bouncingblock() {
  return tscript_bouncingblock_;
}


int tscript_bouncingblock_bytes() {
  return sizeof(tscript_bouncingblock_);
}


static const uint16_t tscript_colormix_[] = {
// Octal 0, 0 or 1 for with previous, 0..7 for lower index, 0..7 for upper index, 0..7 for red, 0..7 for green and 0..7 for blue
//oPLURGB


  // white bg, red from left, green from right
  0007777, // 01234567
  0100700, // r-------

  0007777,
  0100700, // 01234567
  0177070, // r------g

  0007777,
  0101700, // 01234567
  0177070, // rr-----g

  0007777,
  0101700, // 01234567
  0167070, // rr----gg

  0007777,
  0112700, // 01234567
  0167070, // -rr---gg

  0007777,
  0112700, // 01234567
  0156070, // -rr--gg-

  0007777,
  0123700, // 01234567
  0156070, // --rr-gg-

  0007777,
  0123700, // 01234567
  0145070, // --rrgg--

  0007777,
  0133700, // 01234567
  0144770, // ---ryg--
  0155070,

  0007777, // 01234567
  0134770, // ---yy---

  0007777,
  0155700, // 01234567
  0144770, // ---gyr--
  0133070,

  0007777,
  0145700, // 01234567
  0123070, // --ggrr--

  0007777,
  0156700, // 01234567
  0123070, // --gg-rr-

  0007777,
  0156700, // 01234567
  0112070, // -gg--rr-

  0007777,
  0167700, // 01234567
  0112070, // -gg---rr

  0007777,
  0167700, // 01234567
  0101070, // gg----rr

  0007777,
  0177700, // 01234567
  0101070, // gg-----r

  0007777,
  0177700, // 01234567
  0100070, // g------r

  0007777, // 01234567
  0100070, // g-------

  0007777, // 01234567

  // back
  0007777, // 01234567
  0100070, // g-------

  0007777,
  0177700, // 01234567
  0100070, // g------r

  0007777,
  0177700, // 01234567
  0101070, // gg-----r

  0007777,
  0167700, // 01234567
  0101070, // gg----rr

  0007777,
  0167700, // 01234567
  0112070, // -gg---rr

  0007777,
  0156700, // 01234567
  0112070, // -gg--rr-

  0007777,
  0156700, // 01234567
  0123070, // --gg-rr-

  0007777,
  0145700, // 01234567
  0123070, // --ggrr--

  0007777,
  0155700, // 01234567
  0144770, // ---gyr--
  0133070,

  0007777, // 01234567
  0134770, // ---yy---

  0007777,
  0133700, // 01234567
  0144770, // ---ryg--
  0155070,

  0007777,
  0123700, // 01234567
  0145070, // --rrgg--

  0007777,
  0123700, // 01234567
  0156070, // --rr-gg-

  0007777,
  0112700, // 01234567
  0156070, // -rr--gg-

  0007777,
  0112700, // 01234567
  0167070, // -rr---gg

  0007777,
  0101700, // 01234567
  0167070, // rr----gg

  0007777,
  0101700, // 01234567
  0177070, // rr-----g

  0007777,
  0100700, // 01234567
  0177070, // r------g

  0007777, // 01234567
  0100700, // r-------

  0007777, // 01234567

  // End
  0070000,
};


const uint16_t * tscript_colormix() {
  return tscript_colormix_;
}


int tscript_colormix_bytes() {
  return sizeof(tscript_colormix_);
}


static const uint16_t tscript_heartbeat_[] = {
// Octal 0, 0 or 1 for with previous, 0..7 for lower index, 0..7 for upper index, 0..7 for red, 0..7 for green and 0..7 for blue
//oPLURGB

  // first heart beat
  0007100,
  0007100,
  0007100,
  0007300,
  0007500,
  0007700,
  0007700,
  0007500,
  0007300,
  0007100,
  // second heart beat
  0007100,
  0007300,
  0007500,
  0007700,
  0007700,
  0007700,
  0007700,
  0007700,
  0007700,
  0007500,
  0007300,
  0007100,

  // fade
  0007100,
  0007100,
  // long pause
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  0007010,
  // fade
  0007100,
  0007100,

  // End
  0070000,
};


const uint16_t * tscript_heartbeat() {
  return tscript_heartbeat_;
}


int tscript_heartbeat_bytes() {
  return sizeof(tscript_heartbeat_);
}


