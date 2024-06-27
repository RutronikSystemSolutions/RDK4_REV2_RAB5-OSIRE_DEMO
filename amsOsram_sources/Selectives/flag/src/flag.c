// flag.c - Uses topo to paint flags spread out over an entire OSP string
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


#include <topo/inc/topo.h>
#include <flag/inc/flag.h>


static osp2_error_t flag_3bands(const topo_rgb_t*band1,const topo_rgb_t*band2,const topo_rgb_t*band3) {
  // determine number of triplets we have
  int numtot  = topo_numtriplets(); // total number of triplets in chain
  int nummcu1 = topo_node_numtriplets(1); // number of triplets on MCU board at the start of the chain
  int nummcu3 = topo_loop()? topo_node_numtriplets(topo_numnodes()) : 0; // number of triplets on MCU board at the end of the chain
  int numpcb  = numtot-nummcu1-nummcu3; // number of triplets on the pcb(s), ie not on the MCU board
  int numflag = numpcb>=3 ? numpcb : numtot; // if there is enough triplets on the pcb, use only pcb for flag else use all triplets in chain

  // divide triplets over 3 bands
  int div = numflag / 3; // number of triplets for each band
  int mod = numflag % 3; // triplets left over

  // If one triplet is left over, put it in the middle band; if two are left over put it on both side bands
  int num1 = div + (mod==2?1:0); // number of colors for left side band
  int num2 = div + (mod==1?1:0); // number of colors for middle band
  int num3 = div + (mod==2?1:0); // number of colors for right side band

  // if we ignored the triplets on the MCU, add them again
  if( numpcb>=3 ) { num1+=nummcu1; num3+=nummcu3; }

  // generate all three bands
  osp2_error_t err;
  uint16_t tix=0;
  for( int i=0; i<num1; i++,tix++ ) {
    err= topo_settriplet(tix, band1 );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num2; i++,tix++ ) {
    err= topo_settriplet(tix, band2 );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num3; i++,tix++ ) {
    err= topo_settriplet(tix, band3 );
    if( err!=OSP2_ERROR_NONE ) return err;
  }

  return OSP2_ERROR_NONE;
}


osp2_error_t flag_dutch() { // ~France, Luxembourg
  return flag_3bands( &topo_red, &topo_white, &topo_blue );
}


osp2_error_t flag_columbia() { // ~ Ecuador, Venezuela
  return flag_3bands( &topo_orange, &topo_blue, &topo_red );
}


osp2_error_t flag_japan() {
  return flag_3bands( &topo_white, &topo_red, &topo_white );
}


osp2_error_t flag_mali() { //  ~ Benin, Cameroon, Ghana, Senegal
  return flag_3bands( &topo_green, &topo_orange, &topo_red );
}


// blue*-yellow1-blue*-yellow1-blue*
osp2_error_t flag_europe() {
  int numtot  = topo_numtriplets(); // total number of triplets in chain
  int numstart= topo_node_numtriplets(1); // number of triplets on the start of the chain that must be blue
  int numend  = topo_loop()? topo_node_numtriplets(topo_numnodes()) : 0; // number of triplets on the end of the chain that must be blue
  int numpcb  = numtot-numstart-numend; // number of triplets on the PCB
  int numstars= numpcb<5 ? 0 : 2 ; //number of yellow stars
  int numblue = numpcb-numstars; // number of triplets on the pcb(s) that we want to be blue

  // there are three blue bands
  int div = numblue / 3; // number of triplets for each band
  int mod = numblue % 3; // triplets left over

  // If one triplet is left over, put it in the middle band; if two are left over put it on both side bands
  int num1 = div + (mod==2?1:0); // number of blues before first star
  int num2 = numstars/2; // yellow star
  int num3 = div + (mod==1?1:0); // number of blues between the two stars
  int num4 = numstars/2; // yellow star
  int num5 = div + (mod==2?1:0); // number of blues after the star

  num1+= numstart;
  num5+= numend;

  // generate three blue bands with two yellow stars
  osp2_error_t err;
  uint16_t tix=0;
  for( int i=0; i<num1; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_blue );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num2; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_orange );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num3; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_blue );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num4; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_orange );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num5; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_blue );
    if( err!=OSP2_ERROR_NONE ) return err;
  }

  return OSP2_ERROR_NONE;
}


osp2_error_t flag_usa() {
  // determine number of triplets we have
  int numtot    = topo_numtriplets(); // total number of triplets in chain
  int nummain   = numtot-2-topo_node_numtriplets(1); // we have 1 blue to start then several white-blue and then several red-white pairs, and finally one red. The outer two are removed here.
  int numpairs  = nummain/2;
  int numcorner = numpairs/3; // the blue-white part

  osp2_error_t err;
  uint16_t tixnum=topo_numtriplets();
  uint16_t tix=0;
  // First blue
  int mcu=0;
  while( mcu<topo_node_numtriplets(1)+1 && tix<tixnum ) {
    err= topo_settriplet(tix, &topo_blue );
    if( err!=OSP2_ERROR_NONE ) return err;
    tix++;
    mcu++;
  }
  // white/blue pairs
  int pairs=0;
  while( pairs<numcorner && tix<tixnum ) {
    err= topo_settriplet(tix, &topo_white );
    if( err!=OSP2_ERROR_NONE ) return err;
    tix++;
    if( tix<tixnum ) {
      err= topo_settriplet(tix, &topo_blue );
      if( err!=OSP2_ERROR_NONE ) return err;
      tix++;
    }
    pairs++;
  }
  // first red
  if( tix<tixnum ) {
    err= topo_settriplet(tix, &topo_red );
    if( err!=OSP2_ERROR_NONE ) return err;
    tix++;
  }
  // white/red pairs
  while( tix<tixnum ) {
    err= topo_settriplet(tix, &topo_white );
    if( err!=OSP2_ERROR_NONE ) return err;
    tix++;
    if( tix<tixnum ) {
      err= topo_settriplet(tix, &topo_red );
      if( err!=OSP2_ERROR_NONE ) return err;
      tix++;
    }
  }
  return OSP2_ERROR_NONE;
}


// (asia) china: red*-yellow2-red-yellow1-red*
osp2_error_t flag_china() {
  int numtot  = topo_numtriplets(); // total number of triplets in chain
  int numstart= topo_node_numtriplets(1); // number of triplets on the start of the chain that must be red
  int numend  = topo_loop()? topo_node_numtriplets(topo_numnodes()) : 0; // number of triplets on the end of the chain that must be red
  int numpcb  = numtot-numstart-numend; // number of triplets on the PCB
  int numstars= numpcb<7 ? 0 : 3 ; //number of yellow stars
  int numred  = numpcb-numstars; // number of triplets on the pcb(s) that we want to be red

  // If one triplet is left over, put it in the middle band; if two are left over put it on both side bands
  int num1 = numred>1 ? 1 : 0; // number of reds before first star
  int num2 = (numstars+1)/2; // yellow star
  int num3 = numred>2 ? 1 : 0; // number of reds between the two stars
  int num4 = numstars/2; // yellow star
  int num5 = numred-num1-num3; // number of reds after the star

  num1+= numstart;
  num5+= numend;

  // generate three red bands with two yellow stars
  osp2_error_t err;
  uint16_t tix=0;
  for( int i=0; i<num1; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_red );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num2; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_orange );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num3; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_red );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num4; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_orange );
    if( err!=OSP2_ERROR_NONE ) return err;
  }
  for( int i=0; i<num5; i++,tix++ ) {
    err= topo_settriplet(tix, &topo_red );
    if( err!=OSP2_ERROR_NONE ) return err;
  }

  return OSP2_ERROR_NONE;
}


