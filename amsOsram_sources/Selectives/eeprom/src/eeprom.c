// eeprom.c - read and write from an I2C EEPROM connected to a SAID
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


#include <string.h>                 // eg memcpy()
#include <SwTimer/inc/swTimer.h>    // eg delay_ms()
#include <osp2/inc/osp2.h>          // eg osp2_exec_i2cread8
#include <eeprom/inc/eeprom.h>


// Checks if an EEPROM with 7-bit device address `daddr7` is connected to OSP node with address `addr`.
// Returns OSP2_ERROR_NONE when device is there, OSP2_ERROR_MISSI2CDEV when not, anything else for telegram errors.
osp2_error_t eeprom_present(uint16_t addr, uint8_t daddr7 ) {
  uint8_t buf;
  osp2_error_t err;
  err = osp2_exec_i2cread8(addr, daddr7, 0x00, &buf, 1);
  if( err!=OSP2_ERROR_I2CNACK && err!=OSP2_ERROR_I2CTIMEOUT && err!=OSP2_ERROR_NONE ) return err;
  if( err==OSP2_ERROR_I2CNACK || err==OSP2_ERROR_I2CTIMEOUT ) return OSP2_ERROR_MISSI2CDEV;
  return OSP2_ERROR_NONE;
}


// Read `size `size` bytes into Write buffer `buf` from I2C EEPROM with 7-bit device addres `daddr7` connected to OSP node with address `addr`.
// Buffer will be read from the EEPROM from (register) address `raddr` onwards.
osp2_error_t eeprom_read(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, int count ) {
  if( raddr+count>256 ) return OSP2_ERROR_OUTOFMEM;
  #define EEPROM_MAXREADCHUNK 8
  osp2_error_t err;

  while( count>0 ) {
    uint8_t chunk= count > EEPROM_MAXREADCHUNK  ?  EEPROM_MAXREADCHUNK  :  count;
    err= osp2_exec_i2cread8(addr, daddr7, raddr, buf, chunk);
    if( err!=OSP2_ERROR_NONE ) return err;
    raddr+= chunk;
    buf+= chunk;
    count-= chunk;
  }
  return OSP2_ERROR_NONE;
}


// Write buffer `buf` of `size `size` to I2C EEPROM with 7-bit device addres `daddr7` connected to OSP node with address `addr`.
// Buffer will be written in the EEPROM from (register) address `raddr` onwards.
osp2_error_t eeprom_write(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, int count ) {
  if( raddr+count>256 ) return OSP2_ERROR_OUTOFMEM;
  #define EEPROM_PAGE_SIZE 8
  osp2_error_t err;

  // There are several issues when writing to EEPROM
  // (1) When an I2C write transaction to an EEPROM is completed (when STOP received), the EEPROM starts an internal write cycle
  //     The "Self-timed write cycle" takes 5ms max (AT24C02C), so we want to minimize write cycles
  // (2) Writes are buffered in a 8 byte "page" buffer, and the target address should not cross a 16 byte boundary
  //     Note, some have a page size of 16, using 8 is safe in all cases (but slightly slower)
  // (3) osp2_exec_i2cwrite8() only allows payloads of  1, 2, 4, or 6 bytes
  while( count>0 ) {
    int fit_in_page   = EEPROM_PAGE_SIZE - (raddr % EEPROM_PAGE_SIZE); // see (2)
    int write_to_page = count > fit_in_page ? fit_in_page : count;
    int chunk;  // see (3)
    if( write_to_page>=6 ) chunk=6;
    else if( write_to_page>=4 ) chunk=4;
    else if( write_to_page>=2 ) chunk=2;
    else chunk=1;
    // dbg_printf("eeprom write %02x %d -> %s\n",raddr, chunk, osp2_buf_str(buf, chunk) );
    err= osp2_exec_i2cwrite8(addr, daddr7, raddr, buf, chunk);
    Cy_SysLib_Delay(5); // see (1)
    if( err!=OSP2_ERROR_NONE ) return err;
    raddr+= chunk;
    buf+= chunk;
    count-= chunk;
  }
  return OSP2_ERROR_NONE;
}


// Read `size `size` bytes from I2C EEPROM with 7-bit device addres `daddr7` connected to OSP node with address `addr` and compare with buffer `buf`.
// Compare from the EEPROM from (register) address `raddr` onwards.
osp2_error_t eeprom_compare(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, int count ) {
  if( raddr+count>256 ) return OSP2_ERROR_OUTOFMEM;
  #define EEPROM_MAXREADCHUNK 8
  osp2_error_t err;
  uint8_t temp[8];

  while( count>0 ) {
    uint8_t chunk= count > EEPROM_MAXREADCHUNK  ?  EEPROM_MAXREADCHUNK  :  count;
    err= osp2_exec_i2cread8(addr, daddr7, raddr, temp, chunk);
    if( err!=OSP2_ERROR_NONE ) return err;
    if( memcmp(temp,buf,chunk)!=0 ) {
      // dbg_printf("EPM %02x: %s\n", raddr,osp2_buf_str(temp,chunk) );
      // dbg_printf("MCU %02x: %s\n", raddr,osp2_buf_str(buf,chunk ) );
      return OSP2_ERROR_DATA;
    }
    raddr+= chunk;
    buf+= chunk;
    count-= chunk;
  }
  return OSP2_ERROR_NONE;
}

