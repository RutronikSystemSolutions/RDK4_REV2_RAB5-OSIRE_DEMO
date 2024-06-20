// topo.h - compute a topological map of all nodes in the OSP chain
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
#ifndef TOPO_H_
#define TOPO_H_

#ifdef __cplusplus
extern "C"
  {
#endif


#include <stdint.h>
#include <osp2/inc/osp2.h>



// IDs of the various OSP devices - topo_node_id()
#define TOPO_ID_OSIRE      0x00000000
#define TOPO_ID_SAID       0x00000040


osp2_error_t topo_scan( uint16_t numnodes, int loop ); // Builds a topo map of all triplets.


int          topo_loop(); // Returns the chain direction (loop/bidir)

uint16_t     topo_numnodes(); // Returns the number of nodes in the scanned chain
uint32_t     topo_node_id( uint16_t addr ); // Returns the id of the node with address addr, 1<=addr<=topo_numnodes()
uint8_t      topo_node_numtriplets( uint16_t addr ); // Returns the number of triplets driven by the node with address addr, 1<=addr<=topo_numnodes()
uint16_t     topo_node_triplet1( uint16_t addr ); // Returns the triplet index of the first triplet driven by the node with address addr, 1<=addr<=topo_numnodes()

uint16_t     topo_numtriplets(); // Returns the number of triplets in the scanned chain
uint16_t     topo_triplet_addr( uint16_t tix ); // Returns the address of the node that drives triplet tix, 0<=tix<topo_numtriplets()
int          topo_triplet_onchan( uint16_t tix ); // Returns if triplet tix, 0<=tix<topo_numtriplets(), is on a channel (or is stand-alone).
uint8_t      topo_triplet_chan( uint16_t tix ); // Returns the channel, of node topo_addr(tix), that drives triplet tix, 0<=tix<topo_numtriplets(). Only defined when topo_triplet_haschan(tix).

uint16_t     topo_numi2cbridges(); // Returns the number of I2C bridges in the scanned chain
uint16_t     topo_i2cbridge_addr( uint16_t bix ); // Returns the address of the node that has i2c bride bix, 0<=bix<topo_numi2cbridges()


void         topo_dump_triplets(); // Dumps (using dbg_printf) the topology (section triplets) of the scanned chain
void         topo_dump_nodes(); // Dumps (using dbg_printf) the topology (section nodes) of the scanned chain
void         topo_dump_i2cbridges(); // Dumps (using dbg_printf) the topology (section i2cbridges) of the scanned chain


// These are helpers for typical clients of topo: modules implementing a user mode
#define TOPO_BRIGHTNESS_MAX 32767
typedef struct topo_rgb_s { uint16_t r; uint16_t g; uint16_t b; } topo_rgb_t; // Each 0..32767 - "demo brightness range"
extern const topo_rgb_t topo_red, topo_orange, topo_green, topo_cyan, topo_blue, topo_magenta, topo_white;

osp2_error_t topo_stdinit(); // RESETs and INITs (try both BIDIR/LOOP) telegrams, then scan topology
osp2_error_t topo_stdsetup(); // For each node: clears errors, enable CRC, power I2C pads, set current 24mA (except OSIREs), goactive
osp2_error_t topo_settriplet( uint16_t tix, const topo_rgb_t*rgb ); // Sets triplet tix to red/green/blue (each 0..32767).
uint16_t     topo_findi2cdev( uint8_t daddr7 ); // Searches for a SAID with an I2C device with the 7-bit I2C address `daddr7`, retursn SAID address or 0


#ifdef __cplusplus
}
#endif

#endif  // TOPO_H_
