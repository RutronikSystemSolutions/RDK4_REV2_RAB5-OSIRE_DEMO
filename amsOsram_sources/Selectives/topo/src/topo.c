// topo.c - compute a topological map of all nodes in the OSP chain
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
#include <topo/inc/topo.h>


// Terminology: a _node_ is an element in an OSP chain. Such an element has a type
// denoted by an _id_, which is either an OSIRE or a SAID. At the moment of writing this,
// there are no other OSP node types (chips) available. A node has an _address_ usually
// abbreviated to "addr" and addresses start at 1.
// A _triplet_ is a group of three LEDs: typically one red, one green, and one blue LED.
// An OSIRE embeds one triplet, a SAID has 3 _channels_, each channel driving one
// external triplet. However, it is possible that a SAID is configured to use its third
// channel as I2C channel. In that case the SAID only drives two triplets.
// a _triplet_ has a triplet index usually abbreviated to "tix" and unlike addresses
// triplet indices start at 0.
// An _i2cbridge_ is a node (of type SAID) whose third channel is configured for I2C.

// Once an OSP chain has been init (INITLOOP or INITBIDIR), typically topo_scan() is called.
// This builds a topological map of all triplets, nodes and I2C bridges. It also switches on
// the I2C channel of every SAID that has an I2C device (*) attached to its third channel.
// Once the scan is completed, it can be printed for debug with topo_dump(), but in normal
// applications, the topological map is inspect via the observers
// - triplets via topo_numtriplets(), topo_triplet_addr(), topo_triplet_onchan(), topo_triplet_chan()
// - nodes via topo_numnodes(), topo_node_id(), topo_node_numtriplets(), topo_node_triplet1()
// - i2c brdiges via topo_numi2cbridges(), topo_i2cbridge_addr()
// (*) The scan function checks for one I2C device. Its device address and read register address
// are hardwired in the code: TOPO_I2CBRIDGE_DADDR. TOPO_I2CBRIDGE_RADDR


#define TOPO_MAXNODES      100 // Theoretical max is 1000 (addr space of OSP)
#define TOPO_MAXTRIPLETS   200 // Theoretical max is 3000 (3 triplets on 1000 SAIDs)
#define TOPO_MAXI2CBRIDGES   5 // Theoretical max is 1000 (every one of teh 1000 SAIDs)

#define TOPO_CHAN_NONE     0xFF


static int      topo_loop_;

static uint16_t topo_numnodes_;
static uint32_t topo_node_id_[TOPO_MAXNODES];
static uint8_t  topo_node_numtriplets_[TOPO_MAXNODES];
static uint16_t topo_node_triplet1_[TOPO_MAXNODES];

static uint16_t topo_numtriplets_;
static uint16_t topo_triplet_addr_[TOPO_MAXTRIPLETS];
static uint8_t  topo_triplet_chan_[TOPO_MAXTRIPLETS];

static uint16_t topo_numi2cbridges_;
static uint16_t topo_i2cbridge_addr_[TOPO_MAXI2CBRIDGES];


#if 0
// The function topo_isi2cbridge() categorizes a SAID as having an I2C bridge by using heuristics; as a side effect the I2C pads are powered
static osp2_error_t topo_isi2cbridge(uint16_t addr, int*isbridge ) {

  // In the future, we only look at the I2C enable flag in OTP.
  osp2_error_t err;
  err = osp2_exec_i2cenable_get(addr, isbridge);
  if( err!=OSP2_ERROR_NONE ) return err;
  if( *isbridge ) {
    // Supply current to I2C pads (channel 2)
    err= osp2_send_setcurchn(addr, 2, 0, 4, 4, 4);
    if( err!=OSP2_ERROR_NONE ) return err;
    return OSP2_ERROR_NONE;
  }

  // When the OTPs are not yet burned, we can use a heuristic
  *isbridge = 0;

  // This code is complicated by resource management (C implementation of try finally).
  // We claim (set) OTP and current, but we must also free (clear) them.
  // The following flags indicate whether those resources need to freed.
  // That is done in the handler that we goto on error.
  int resource_otp=0;
  int resource_cur=0;

  // Switch SAIDs I2C bridge on
  err = osp2_exec_i2cenable_set(addr,1);
  if( err!=OSP2_ERROR_NONE ) goto free_resources;
  resource_otp = 1; // OTP is set, so we need to unset it in case of errors
  // Supply current to I2C pads (channel 2)
  err= osp2_send_setcurchn(addr, 2, 0, 4, 4, 4);
  if( err!=OSP2_ERROR_NONE ) goto free_resources;
  resource_cur = 1; // Pad current is set, so we need to unset it in case of errors

  uint8_t buf;
  // We check if this device is connected
  #define CHECK1_DADDR7 0x50
  #define CHECK1_RADDR  0x00
  err = osp2_exec_i2cread8(addr, CHECK1_DADDR7, CHECK1_RADDR, &buf, 1);
  if( err!=OSP2_ERROR_I2CNACK && err!=OSP2_ERROR_I2CTIMEOUT && err!=OSP2_ERROR_NONE ) goto free_resources;
  *isbridge = err==OSP2_ERROR_NONE;
  if( *isbridge ) {
    // Do not free resources: we have an I2C bus, so we need that registered in OTP and current
    return OSP2_ERROR_NONE;
  }

  // Or this device?
  #define CHECK2_DADDR7 0x51
  #define CHECK2_RADDR  0x00
  err = osp2_exec_i2cread8(addr, CHECK2_DADDR7, CHECK2_RADDR, &buf, 1);
  if( err!=OSP2_ERROR_I2CNACK && err!=OSP2_ERROR_I2CTIMEOUT && err!=OSP2_ERROR_NONE ) goto free_resources;
  *isbridge = err==OSP2_ERROR_NONE;
  if( *isbridge ) {
    // Do not free resources: we have an I2C bus, so we need that registered in OTP and current
    return OSP2_ERROR_NONE;
  }

  // Ended without errors, still need to free resources
  err=OSP2_ERROR_NONE;

  // Clean up by freeing claimed resources
free_resources:
  if( resource_cur ) osp2_send_setcurchn(addr, 2, 0, 0, 0, 0); // no further error handling here
  if( resource_otp ) osp2_exec_i2cenable_set(addr,0); // no further error handling here

  return err;
}
#endif



// Builds a topo map of all triplets.
osp2_error_t topo_scan( uint16_t numnodes, int loop ) {
  topo_loop_ = loop;
  topo_numnodes_ = 0;
  topo_numtriplets_ = 0;
  topo_numi2cbridges_ = 0;
  while( topo_numnodes_ < numnodes ) {
    uint16_t addr = topo_numnodes_ + 1; // node addresses start counting at 1
    // Get the id of the node
    uint32_t id;
    osp2_error_t err = osp2_send_identify( addr, &id );
    if( err!=OSP2_ERROR_NONE ) return err;
    // Record the node's id (if there is still space)
    if( topo_numnodes_>=TOPO_MAXNODES ) return OSP2_ERROR_OUTOFMEM;
    topo_node_id_[topo_numnodes_] = id;
    topo_node_triplet1_[topo_numnodes_] = topo_numtriplets_;
    // Register the triplets of the node
    if( id == TOPO_ID_OSIRE ) { // OSIRE: one triplet, no channel.
      // Record the triplet's address and channel (if there is still space)
      if( topo_numtriplets_>=TOPO_MAXTRIPLETS ) return OSP2_ERROR_OUTOFMEM;
      topo_triplet_addr_[topo_numtriplets_] = addr;
      topo_triplet_chan_[topo_numtriplets_] = TOPO_CHAN_NONE;
      topo_numtriplets_++;
      topo_node_numtriplets_[topo_numnodes_] = 1;
    } else if( id == TOPO_ID_SAID ) { // SAID: three triplets, or two plus I2C bridge
      // Record the channel 0 triplet's address and channel (if there is still space)
      if( topo_numtriplets_>=TOPO_MAXTRIPLETS ) return OSP2_ERROR_OUTOFMEM;
      topo_triplet_addr_[topo_numtriplets_] = addr;
      topo_triplet_chan_[topo_numtriplets_] = 0;
      topo_numtriplets_++;
      // Record the channel 1 triplet's address and channel (if there is still space)
      if( topo_numtriplets_>=TOPO_MAXTRIPLETS ) return OSP2_ERROR_OUTOFMEM;
      topo_triplet_addr_[topo_numtriplets_] = addr;
      topo_triplet_chan_[topo_numtriplets_] = 1;
      topo_numtriplets_++;
      // Is channel 2 of this SAID wired for I2C?
      int isbridge;
      osp2_error_t err = osp2_exec_i2cenable_get(addr, &isbridge );
      if( err!=OSP2_ERROR_NONE ) return err;
      if( isbridge ) {
        // Record the I2C bridge's address (if there is still space)
        if( topo_numi2cbridges_>=TOPO_MAXI2CBRIDGES ) return OSP2_ERROR_OUTOFMEM;
        topo_i2cbridge_addr_[topo_numi2cbridges_] = addr;
        topo_numi2cbridges_ ++;
        topo_node_numtriplets_[topo_numnodes_] = 2;
      } else {
        // Record the channel 2 triplet's address and channel (if there is still space)
        if( topo_numtriplets_>=TOPO_MAXTRIPLETS ) return OSP2_ERROR_OUTOFMEM;
        topo_triplet_addr_[topo_numtriplets_] = addr;
        topo_triplet_chan_[topo_numtriplets_] = 2;
        topo_numtriplets_++;
        topo_node_numtriplets_[topo_numnodes_] = 3;
      }
    } else { // Unknown ID
      return OSP2_ERROR_ID;
    }
    topo_numnodes_ ++;
  }
  return OSP2_ERROR_NONE;
}


// Returns the chain direction (loop/bidir)
int topo_loop() {
  return topo_loop_;
}


// Returns the number of nodes in the scanned chain
uint16_t topo_numnodes() {
  return topo_numnodes_;
}


// Returns the id of the node with address addr, 1<=addr<=topo_numnodes()
uint32_t topo_node_id( uint16_t addr ) {
  return topo_node_id_[addr-1]; // addr's are 1 based
}


// Returns the number of triplets driven by the node with address addr, 1<=addr<=topo_numnodes()
uint8_t topo_node_numtriplets( uint16_t addr ) {
  return topo_node_numtriplets_[addr-1]; // addr's are 1 based
}


// Returns the triplet index of the first triplet driven by the node with address addr, 1<=addr<=topo_numnodes()
uint16_t topo_node_triplet1( uint16_t addr ) {
  return topo_node_triplet1_[addr-1]; // addr's are 1 based
}


// Returns the number of triplets in the scanned chain
uint16_t topo_numtriplets() {
  return topo_numtriplets_;
}


// Returns the address of the node that drives triplet tix, 0<=tix<topo_numtriplets()
uint16_t topo_triplet_addr( uint16_t tix ) {
  return topo_triplet_addr_[tix];
}


// Returns if triplet tix, 0<=tix<topo_numtriplets(), is on a channel (or is stand-alone).
int topo_triplet_onchan( uint16_t tix ) {
  return topo_triplet_chan_[tix] != TOPO_CHAN_NONE;
}


// Returns the channel, of node topo_addr(tix), that drives triplet tix, 0<=tix<topo_numtriplets()
// Only defined when topo_triplet_haschan(tix).
uint8_t topo_triplet_chan( uint16_t tix ) {
  return topo_triplet_chan_[tix];
}

// Returns the number of I2C bridges in the scanned chain
uint16_t topo_numi2cbridges() {
  return topo_numi2cbridges_;
}


// Returns the address of the node that has i2c bride bix, 0<=bix<topo_numi2cbridges()
uint16_t topo_i2cbridge_addr( uint16_t bix ) {
 return topo_i2cbridge_addr_[bix];
}


// Dumps (using dbg_printf) the topology (section triplets) of the scanned chain
void topo_dump_triplets()
{
//  dbg_printf("topo: triplets %d\n", topo_numtriplets_ );
//  uint16_t bix = 0;
//  for( uint16_t tix=0; tix<topo_numtriplets_; tix++ ) {
//    uint16_t addr = topo_triplet_addr_[tix];
//    dbg_printf("T%d N%d", tix, addr );
//    if( topo_triplet_onchan(tix) ) dbg_printf(".C%d", topo_triplet_chan(tix) );
//    if( bix<topo_numi2cbridges_ && topo_i2cbridge_addr(bix)==addr ) { dbg_printf(" [I2C %d]",bix); bix++; }
//    dbg_printf("\n");
//  }
}


// Dumps (using dbg_printf) the topology (section nodes) of the scanned chain
void topo_dump_nodes()
{
//  dbg_printf("topo: nodes %d\n", topo_numnodes_ );
//  uint16_t bix = 0;
//  for( uint16_t addr=1; addr<=topo_numnodes_; addr++ ) {
//    dbg_printf("N%d (0x%lX)", addr,topo_node_id(addr) );
//    for( uint16_t tix=topo_node_triplet1(addr); tix<topo_node_triplet1(addr)+topo_node_numtriplets(addr); tix++ )
//      dbg_printf(" T%d",tix);
//    if( bix<topo_numi2cbridges_ && topo_i2cbridge_addr(bix)==addr ) { dbg_printf(" [I2C %d]",bix); bix++; }
//    dbg_printf("\n");
//  }
}


// Dumps (using dbg_printf) the topology (section i2cbridges) of the scanned chain
void topo_dump_i2cbridges()
{
//  dbg_printf("topo: i2cbridges %d\n", topo_numi2cbridges_ );
//  for( uint16_t bix=0; bix<topo_numi2cbridges_; bix++ ) {
//    dbg_printf("I%d N%d\n", bix,topo_i2cbridge_addr(bix) );
//  }
}


// ==========================================================================
// These are helpers for typical clients of topo: modules implementing a user mode


// OSIRE as well as SAID have 15 bit colors (0..32767 or 0000..7FFF).
// In this application (software stack), we have chosen to also use 15 bit brightness levels.
// We refer to this as the "demo brightness range".
// This means no conversion is needed when writing a software value into hardware register.

// This is only partially true however, the problem is that OSIREs use a drive current
// of 50 mA, and SAIDs (except for channel 0) do not support that. So we assume
// the SAIDs are configure for a drive current of 24 mA. Software brightness values
// are written directly to SAIDs, but need to be halved before writing to OSIREs.

// See also https://svntf.ams-osram.info/SAID-appl/trunk/5-software/0-docs/PWM%20settings%20for%20OSIRE%20and%20SAID.pptx


// Sets triplet tix to red/green/blue (each 0..32767).
osp2_error_t topo_settriplet( uint16_t tix, const topo_rgb_t *rgb  ) {
  if( tix>=topo_numtriplets_ ) return OSP2_ERROR_ARG;
  osp2_error_t err;
  uint16_t addr = topo_triplet_addr(tix);
  if( topo_triplet_onchan(tix) ) {
    // The application brightness levels are intended for 24mA drivers; we need to shift in the disable 0 for LSB dithering
    err= osp2_send_setpwmchn(addr, topo_triplet_chan(tix), rgb->r << 1, rgb->g << 1, rgb->b << 1 );
    if( err!=OSP2_ERROR_NONE ) return err;
  } else {
    // The application brightness levels are intended for 24mA drivers, so enable daytime (50 mA) and divide by 2
    err= osp2_send_setpwm( addr, rgb->r / 2, rgb->g / 2, rgb->b / 2, 0b111 );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  return OSP2_ERROR_NONE;
}


// We define some standard colors, however we should not make them too bright.
// Too high brightness levels will cause under voltage in nodes, making then switch off (SLEEP).
// For example white { 0x1FFF,0x1FFF,0x1FFF } is too bright.
// The standard colors use 0FFF, which is 8x dimmer than 7FFF (the max).
const topo_rgb_t topo_red    = { 0x0FFF,0x0000,0x0000 };
const topo_rgb_t topo_orange = { 0x0FFF,0x0FFF,0x0000 };
const topo_rgb_t topo_green  = { 0x0000,0x0FFF,0x0000 };
const topo_rgb_t topo_cyan   = { 0x0000,0x0FFF,0x0FFF };
const topo_rgb_t topo_blue   = { 0x0000,0x0000,0x0FFF };
const topo_rgb_t topo_magenta= { 0x0FFF,0x0000,0x0FFF };
const topo_rgb_t topo_white  = { 0x0FFF,0x0FFF,0x0FFF };


// RESETs and INITs (try both BIDIR/LOOP) telegrams, then scan topology
osp2_error_t topo_stdinit() {
  osp2_error_t err;
  int loop;
  uint16_t last;

  // reset and init
  err= osp2_exec_resetinit(&last,&loop);
  if( err!=OSP2_ERROR_NONE ) return err;

  // scan
  err=topo_scan(last,loop);
  if( err!=OSP2_ERROR_NONE ) return err;
  // topo_dump_triplets();
  // topo_dump_nodes();
  // topo_dump_i2cbridges();

  return OSP2_ERROR_NONE;
}


// For each node: clears errors, enable CRC, power I2C pads, set current 24mA (except OSIREs), goactive
osp2_error_t topo_stdsetup() {
  osp2_error_t err;

  // Broadcast clear error (to clear the under voltage flag of all SAIDs), must have, otherwise SAID will not go ACTIVE
  err= osp2_send_clrerror(0);

  // Enable CRC for all nodes (could be skipped)
  for( uint16_t addr=1; addr<=topo_numnodes(); addr++ ) {
    if( topo_node_id(addr) == TOPO_ID_OSIRE ) {
      err= osp2_send_setsetup(addr, OSP2_SETUP_FLAGS_OSIRE_DFLT | OSP2_SETUP_FLAGS_CRCEN );
      if( err!=OSP2_ERROR_NONE ) return err;
    } else if( topo_node_id(addr) == TOPO_ID_SAID ) {
      err= osp2_send_setsetup(addr, OSP2_SETUP_FLAGS_SAID_DFLT  | OSP2_SETUP_FLAGS_CRCEN );
      if( err!=OSP2_ERROR_NONE ) return err;
    } else {
      return OSP2_ERROR_ID;
    }
  }

  // Every I2C bridge needs its pads powered
  for( uint16_t bix=0; bix<topo_numi2cbridges_; bix++ ) {
    // Supply current to I2C pads (channel 2)
    err =osp2_send_setcurchn( topo_i2cbridge_addr(bix), /*chan*/2, /*flags*/0,  4, 4, 4);
    if( err!=OSP2_ERROR_NONE ) return err;

  }

  // To make all triplets have the same brightness, we define a fixed current: 24 mA
  // All SAIDs will get all their channel CURRENT set to 24mA, for OSIRE current is part of PWM.
  // We also enable dithering for the SAIDs
  for( uint16_t addr=1; addr<=topo_numnodes(); addr++ ) {
    if( topo_node_id(addr) == TOPO_ID_SAID ) {
      err=osp2_send_setcurchn(addr, 0, OSP2_CURCHN_FLAGS_DITHER, 3, 3, 3); // 3x 24mA for channel 0 (is higher powered)
      if( err!=OSP2_ERROR_NONE ) return err;
      err=osp2_send_setcurchn(addr, 1, OSP2_CURCHN_FLAGS_DITHER, 4, 4, 4); // 3x 24mA for channel 1
      if( err!=OSP2_ERROR_NONE ) return err;
      if( topo_node_numtriplets(addr)==3 ) {
        err=osp2_send_setcurchn(addr, 2, OSP2_CURCHN_FLAGS_DITHER, 4, 4, 4); // 3x 24mA for channel 2
        if( err!=OSP2_ERROR_NONE ) return err;
      }
    }
  }

  // Switch all nodes to active (LEDs on)
  err=osp2_send_goactive(0);
  if( err!=OSP2_ERROR_NONE ) return err;

  return OSP2_ERROR_NONE;
}


// Searches for a SAID with an I2C device with the 7-bit I2C address `daddr7`.
// If none found returns 0, else returns the OSP node address of the SAID.
// Make sure the I2C pads of the SAID are powered (see eg topo_stdsetup).
uint16_t topo_findi2cdev( uint8_t daddr7 ) {
  for( uint16_t bix=0; bix<topo_numi2cbridges_; bix++ ) {
    uint16_t saidaddr = topo_i2cbridge_addr(bix);
    uint8_t buf;
    osp2_error_t err;
    err = osp2_exec_i2cread8(saidaddr, daddr7, 0x00, &buf, 1);
    if( err==OSP2_ERROR_NONE ) return saidaddr;
  }
  return 0;
}

