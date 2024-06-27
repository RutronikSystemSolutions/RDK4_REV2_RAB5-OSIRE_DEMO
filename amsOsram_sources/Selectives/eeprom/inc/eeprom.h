// eeprom.h - read and write from an I2C EEPROM connected to a SAID
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
#ifndef EEPROM_H_
#define EEPROM_H_

#ifdef __cplusplus
extern "C"
  {
#endif


#include <stdint.h>
#include <osp2/inc/osp2.h>


#define EEPROM_INTERNAL  0x50 // I2C address EEPROM on SAID basic board
#define EEPROM_EXTERNAL  0x51 // I2C address EEPROM on "of I2C EEPROM sticks"


osp2_error_t eeprom_present(uint16_t addr, uint8_t daddr7 );
osp2_error_t eeprom_read   (uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, int count );
osp2_error_t eeprom_write  (uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, int count );
osp2_error_t eeprom_compare(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, int count );



#ifdef __cplusplus
}
#endif

#endif  // EEPROM_H_
