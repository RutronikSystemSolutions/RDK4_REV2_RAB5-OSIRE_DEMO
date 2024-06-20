// Selectives/osp2/inc/osp2.h
/*****************************************************************************
 * Copyright 2024 by ams OSRAM AG                                            *
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
#ifndef OSP2_H_

#define OSP2_H_
#ifdef __cplusplus
extern "C"
  {
#endif


#include <Osp/inc/genericDevice.h> // eg enum OSP_ERROR_CODE
#include <stdint.h>



// ==========================================================================
// The osp2 library constructs telegrams (converts int arguments to a byte
// array), and destructs (response) telegrams (converts a byte array to int result
// fields).

// Definition of a telegram
#define OSP_TELE_MAXSIZE 12
typedef struct osp2_tele_s { uint8_t data[OSP_TELE_MAXSIZE]; uint8_t size; } osp2_tele_t;


// A unified list of OSP2 errors
typedef enum osp2_error_e {
  OSP2_ERROR_NONE                                   = 0x00, // No error

  OSP2_ERROR_SPI_MAX_LENGHT_TO_HIGH_SPI             = 0x01, // Must be identical to codes from Hal/Spi/inc/spiGeneral.h
  OSP2_ERROR_SPI_BUSY                               = 0x02,
  OSP2_ERROR_SPI_SEND                               = 0x03,
  OSP2_ERROR_SPI_RECEIVE                            = 0x04,
  OSP2_ERROR_SPI_DUMMY_RECEIVE                      = 0x05,
  OSP2_ERROR_SPI_TIME_OUT                           = 0x06,
  OSP2_ERROR_SPI_NO_RESET_NON_BLOCKING_SEND         = 0x07,
  OSP2_ERROR_SPI_COULD_NOT_ADD_NEW_DATA_BUFFER_FULL = 0x08,
  OSP2_ERROR_SPI_NO_NEW_DATA_RECEIVED               = 0x09,
  OSP2_ERROR_SPI_CORRUPT_DATA                       = 0x0A,
  OSP2_ERROR_SPI_NOT_DEF                            = 0xFF,

  OSP2_ERROR_ARG                                          , // One of the arguments for a (to be sent) telegram is invalid
  OSP2_ERROR_ADDR                                         , // The address for a (to be sent) telegram is invalid
  OSP2_ERROR_OUTARGNULL                                   , // A (pointer for an) output parameter is NULL
  OSP2_ERROR_TELEFIELD                                    , // One of the fields of a (received) telegram is invalid
  OSP2_ERROR_CRC                                          , // The CRC of a (received) telegram is incorrect

  OSP2_ERROR_ID                                           , // OSP node has an unknown ID (known is 00000000 for OSIRE, 00000040 for SAID)
  OSP2_ERROR_I2CNACK                                      , // I2C transaction completed with NACK
  OSP2_ERROR_I2CTIMEOUT                                   , // I2C transaction took too long to complete
  OSP2_ERROR_STATUSFLAGS                                  , // A node has errors in its status flags

  OSP2_ERROR_OUTOFMEM                                     , // Out of memory, eg when building the topology map
  OSP2_ERROR_MISSI2CBRIDGE                                , // Missing (OSP node with) I2C bridge, eg when a user mode must access some I2C device
  OSP2_ERROR_MISSI2CDEV                                   , // Missing specific I2C device, eg when a user mode must have a SAID with an EEPROM or IOExpander
  OSP2_ERROR_DATA                                         , // Generic (dynamic) data error, e.g. when script data read from an EEPROM is inconsistent

} osp2_error_t;


char * osp2_error_str(osp2_error_t err);        // readable string


// ==========================================================================
// Helpers to nicely print telegram fields. Some are OSIRE or SAID specific


int    osp2_temp_osire(uint8_t temp);             // Converts OSIRE raw temperature to Celsius.
int    osp2_temp_said(uint8_t temp);              // Converts SAID raw temperature to Celsius.
char * osp2_stat_osire_str(uint8_t stat);         // Converts OSIRE raw status to a string (like "SLEEP/oL/clou")
char * osp2_stat_said_str(uint8_t stat);          // Converts SAID raw status to a string (like "ACTIVE/tv/clou")
char * osp2_pwm_osire_str(uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes); // Converts OSIRE raw rgb values to a string (like "0.0000/0.0000/0.0000")
char * osp2_pwm_said_str(uint16_t red, uint16_t green, uint16_t blue); // Converts SAID raw rgb values to a string (like "0000/0000/0000")
char * osp2_com_osire_str(uint8_t com);           // Converts OSIRE raw com status values to a string (like "LVDS/LVDS")
char * osp2_com_said_str(uint8_t com);            // Converts SAID raw com status values to a string (like "LOOP/LVDS/LVDS")
char * osp2_setup_str(uint8_t flags);             // Converts a raw setup value to a string (like "pccT/clOU")
char * osp2_buf_str(void * buf, int size);        // Converts a byte array to a string of hex numbers (like "1D 9F 95 6F 42 00 00 00");
char * osp2_i2ccfg_str(uint8_t flags);            // Converts a raw i2c confiog flags value to a string (like "itnb")
int    osp2_i2ccfg_speed(uint8_t temp);           // Converts SAID raw I2C bus speed to bits/second.


// ==========================================================================
// High level helpers (use 'exec' in their name)


osp2_error_t osp2_exec_reset();                   // Sends a RESET telegram (with HAL manipulation and wait).
osp2_error_t osp2_exec_resetinit(uint16_t *last, int *loop ); // Sends RESET/BIDIR telegrams, if that fails sends RESET/LOOP
osp2_error_t osp2_exec_i2cenable_get(uint16_t addr, int *enable);// Reads OTP, and extracts I2C-ENABLE bit.
osp2_error_t osp2_exec_i2cenable_set(uint16_t addr, int enable); // Reads OTP, updates I2C-ENABLE bit and writes it back to OTP (P2RAM cache actually).
osp2_error_t osp2_exec_otpdump(uint16_t addr, int flags);        // Reads the whole SAID otp and prints private and customer area
osp2_error_t osp2_exec_i2cwrite8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, uint8_t count);// Writes count bytes from buf, into register raddr in i2c device daddr7, attached to node addr.
osp2_error_t osp2_exec_i2cread8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, uint8_t count); // Reads count bytes into buf, from register raddr in i2c device daddr7, attached to node addr.


// `flags` for osp2_exec_otpdump determining what to print
#define OSP2_OTPDUMP_RESERVED_HEX    0x01
#define OSP2_OTPDUMP_CUSTOMER_HEX    0x02
#define OSP2_OTPDUMP_RESERVED_FIELDS 0x04
#define OSP2_OTPDUMP_CUSTOMER_FIELDS 0x08
#define OSP2_OTPDUMP_HEX             (OSP2_OTPDUMP_RESERVED_HEX    | OSP2_OTPDUMP_CUSTOMER_HEX    )
#define OSP2_OTPDUMP_FIELDS          (OSP2_OTPDUMP_RESERVED_FIELDS | OSP2_OTPDUMP_CUSTOMER_FIELDS )
#define OSP2_OTPDUMP_ALL             (OSP2_OTPDUMP_HEX | OSP2_OTPDUMP_FIELDS  )


// ==========================================================================
// Logging

#define OSP2_LOG_LEVEL_NONE 0 // Nothing is logged (default)
#define OSP2_LOG_LEVEL_ARG  1 // Log of sent and received parameters
#define OSP2_LOG_LEVEL_TELE 2 // Also logs raw (sent and received) telegram bytes

void osp2_log_set_level(int level);
int  osp2_log_get_level();


// ==========================================================================
// Telegram senders

osp2_error_t osp2_send_reset(uint16_t addr ); // Telegram 00 RESET
osp2_error_t osp2_send_clrerror(uint16_t addr); // Telegram 01 CLRERROR
osp2_error_t osp2_send_initbidir(uint16_t addr, uint16_t * last, uint8_t * temp, uint8_t * stat );// Telegram 02 INITBIDIR
osp2_error_t osp2_send_initloop(uint16_t addr, uint16_t * last, uint8_t * temp, uint8_t * stat );// Telegram 03 INITLOOP
// Telegram 04 GOSLEEP
osp2_error_t osp2_send_goactive(uint16_t addr ); // Telegram 05 GOACTIVE
// Telegram 06 GODEEPSLEEP
osp2_error_t osp2_send_identify(uint16_t addr, uint32_t * id ); // Telegram 07 IDENTIFY
// Telegram 08 P4ERRBIDIR
// Telegram 09 P4ERRLOOP
// Telegram 0A ASK_TINFO
// Telegram 0B ASK_VINFO
// Telegram 0C READMULT
// Telegram 0D SETMULT
osp2_error_t osp2_send_sync(uint16_t addr); // Telegram 0F SYNC
// Telegram 11 IDLE
// Telegram 12 FOUNDRY
// Telegram 13 CUST
// Telegram 14 BURN
// Telegram 15 AREAD
// Telegram 16 LOAD
// Telegram 17 GLOAD
osp2_error_t osp2_send_i2cread8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t num ); // Telegram 18 I2CREAD
osp2_error_t osp2_send_i2cwrite8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t * buf, int size); // Telegram 19 I2CWRITE
osp2_error_t osp2_send_readlast(uint16_t addr, uint8_t * buf, int size); // Telegram 1E READLAST
// Telegram 21 CLRERROR with SR
// Telegram 24 GOSLEEP with SR
// Telegram 25 GOACTIVE with SR
// Telegram 26 GODEEPSLEEP with SR
// Telegram 2D SETMULT with SR
// Telegram 31 IDLE with SR
// Telegram 32 FOUNDRY with SR
// Telegram 33 CUST with SR
// Telegram 34 BURN with SR
// Telegram 35 AREAD with SR
// Telegram 36 LOAD with SR
// Telegram 37 GLOAD with SR
// Telegram 38 I2CREAD with SR
// Telegram 39 I2CWRITE with SR
#define OSP2_STATUS_FLAGS_OTPCRC1      0x20 // OTP crc error (or in test mode)
#define OSP2_STATUS_FLAGS_TESTMODE     0x20 // (OTP crc error or) in test mode
#define OSP2_STATUS_FLAGS_OV           0x10 // Over Voltage [SAID only]
#define OSP2_STATUS_FLAGS_DIRLOOP      0x10 // DIRection is LOOP (not bidir) [OSIRE only]
#define OSP2_STATUS_FLAGS_CE           0x08 // Communication error
#define OSP2_STATUS_FLAGS_LOS          0x04 // Led Open or Short
#define OSP2_STATUS_FLAGS_OT           0x02 // Over Temperature
#define OSP2_STATUS_FLAGS_UV           0x01 // Under Voltage
#define OSP2_SETUP_FLAGS_OSIRE_ERRORS  ( OSP2_STATUS_FLAGS_OTPCRC1 |                        OSP2_STATUS_FLAGS_CE | OSP2_STATUS_FLAGS_LOS | OSP2_STATUS_FLAGS_OT | OSP2_SETUP_FLAGS_UV )
#define OSP2_SETUP_FLAGS_SAID_ERRORS   ( OSP2_STATUS_FLAGS_OTPCRC1 | OSP2_STATUS_FLAGS_OV | OSP2_STATUS_FLAGS_CE | OSP2_STATUS_FLAGS_LOS | OSP2_STATUS_FLAGS_OT | OSP2_SETUP_FLAGS_UV )
osp2_error_t osp2_send_readstat(uint16_t addr, uint8_t * stat); // Telegram 40 READST
osp2_error_t osp2_send_readtempstat(uint16_t addr, uint8_t * temp, uint8_t * stat ); // Telegram 42 READTEMPST
osp2_error_t osp2_send_readcomst(uint16_t addr, uint8_t * com ); // Telegram 44 READCOMST
// Telegram 46 READLEDST
osp2_error_t osp2_send_readtemp(uint16_t addr, uint8_t * temp); // Telegram 48 READTEMP
// Telegram 4A READOTTH
// Telegram 4B SETOTTH
#define OSP2_SETUP_FLAGS_PWMF       0x80 // PWM uses fast clock
#define OSP2_SETUP_FLAGS_COMCLKINV  0x40 // mcu spi COMmunication has CLocK INVerted
#define OSP2_SETUP_FLAGS_CRCEN      0x20 // OTP crc error or in test mode
#define OSP2_SETUP_FLAGS_OTP        0x10 // OTP crc error or in test mode [SAID only]
#define OSP2_SETUP_FLAGS_TEMPCK     0x10 // TEMPerature sensor has low update ClocK [OSIRE only]
#define OSP2_SETUP_FLAGS_CE         0x08 // Communication error
#define OSP2_SETUP_FLAGS_LOS        0x04 // Led Open or Short
#define OSP2_SETUP_FLAGS_OT         0x02 // Over Temperature
#define OSP2_SETUP_FLAGS_UV         0x01 // Under Voltage
#define OSP2_SETUP_FLAGS_OSIRE_DFLT ( OSP2_SETUP_FLAGS_TEMPCK | OSP2_SETUP_FLAGS_OT | OSP2_SETUP_FLAGS_UV )
#define OSP2_SETUP_FLAGS_SAID_DFLT  ( OSP2_SETUP_FLAGS_OTP    | OSP2_SETUP_FLAGS_OT | OSP2_SETUP_FLAGS_UV )
osp2_error_t osp2_send_readsetup(uint16_t addr, uint8_t *flags );// Telegram 4C READSETUP
osp2_error_t osp2_send_setsetup(uint16_t addr, uint8_t flags );// Telegram 4D SETSETUP
osp2_error_t osp2_send_readpwm(uint16_t addr, uint16_t *red, uint16_t *green, uint16_t *blue, uint8_t *daytimes ); // Telegram 4E READPWM (OSIRE only) - the three 1-bit daytimes flags are clubbed into one daytimes argument
osp2_error_t osp2_send_readpwmchn(uint16_t addr, uint8_t chn, uint16_t *red, uint16_t *green, uint16_t *blue ); // Telegram 4E READPWMCHN (SAID only) - the meaning of the 16 color bits varies, not detailed here at telegram level
osp2_error_t osp2_send_setpwm(uint16_t addr, uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes ); // Telegram 4F SETPWM (OSIRE only) - the three 1-bit daytimes flags are clubbed into one daytimes argument
osp2_error_t osp2_send_setpwmchn(uint16_t addr, uint8_t chn, uint16_t red, uint16_t green, uint16_t blue ); // Telegram 4F SETPWMCHN (SAID only) - the meaning of the 16 color bits varies, not detailed here at telegram level
#define OSP2_CURCHN_FLAGS_RESRVD  0x08
#define OSP2_CURCHN_FLAGS_SYNCEN  0x04
#define OSP2_CURCHN_FLAGS_HYBRID  0x02
#define OSP2_CURCHN_FLAGS_DITHER  0x01
// cur   0    1    2    3    4
// chn0  3mA  6mA 12mA 24mA 48mA
// chn1 1.5mA 3mA  6mA 12mA 24mA
// chn2 1.5mA 3mA  6mA 12mA 24mA
osp2_error_t osp2_send_readcurchn(uint16_t addr, uint8_t chn, uint8_t *flags, uint8_t *rcur, uint8_t *gcur, uint8_t *bcur ); // Telegram 50 READCURCHN
osp2_error_t osp2_send_setcurchn(uint16_t addr, uint8_t chn, uint8_t flags, uint8_t rcur, uint8_t gcur, uint8_t bcur); // Telegram 51 SETCURCHN
// Telegram 52 READTCOEFF
// Telegram 53 SETTCOEFF
// Telegram 54 READ_ADC
// Telegram 55 SET_ADC
#define OSP2_I2CCFG_FLAGS_INT      0x08 // status of INT pin
#define OSP2_I2CCFG_FLAGS_12BIT    0x04 // uses 12 bit addressing mode
#define OSP2_I2CCFG_FLAGS_NACK     0x02 // last i2c transaction ended with NACK
#define OSP2_I2CCFG_FLAGS_BUSY     0x01 // last i2c transaction still busy
#define OSP2_I2CCFG_FLAGS_DEFAULT  0x00 // Hardware default in SAID
#define OSP2_I2CCFG_SPEED_DIV0     0x00
#define OSP2_I2CCFG_SPEED_1188kHz  0x01
#define OSP2_I2CCFG_SPEED_594kHz   0x02
#define OSP2_I2CCFG_SPEED_396kHz   0x03
#define OSP2_I2CCFG_SPEED_297kHz   0x04
#define OSP2_I2CCFG_SPEED_238kHz   0x05
#define OSP2_I2CCFG_SPEED_198kHz   0x06
#define OSP2_I2CCFG_SPEED_170kHz   0x07
#define OSP2_I2CCFG_SPEED_148kHz   0x08
#define OSP2_I2CCFG_SPEED_132kHz   0x09
#define OSP2_I2CCFG_SPEED_119kHz   0x0A
#define OSP2_I2CCFG_SPEED_108kHz   0x0B
#define OSP2_I2CCFG_SPEED_99kHz    0x0C
#define OSP2_I2CCFG_SPEED_91kHz    0x0D
#define OSP2_I2CCFG_SPEED_85kHz    0x0E
#define OSP2_I2CCFG_SPEED_79kHz    0x0F
#define OSP2_I2CCFG_SPEED_DEFAULT  0x0C // Hardware default in SAID
osp2_error_t osp2_send_readi2ccfg(uint16_t addr, uint8_t *flags, uint8_t * speed ); // Telegram 56 READI2CCFG
osp2_error_t osp2_send_seti2ccfg(uint16_t addr, uint8_t flags, uint8_t speed ); // Telegram 57 SETI2CCFG
osp2_error_t osp2_send_readotp(uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size); // Telegram 58 READOTP
osp2_error_t osp2_send_setotp(uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size); // Telegram 59 SETOTP
// Telegram 5A TESTDATAREAD
osp2_error_t osp2_send_settestdata(uint16_t addr, uint16_t data ); // Telegram 5B TESTDATASET
// Telegram 5C READADCDAT
// Telegram 5D TESTSCAN
osp2_error_t osp2_send_testpw(uint16_t addr, uint64_t pw); // Telegram 5F TESTPW
// Telegram 6B SETOTTH with SR
// Telegram 6D SETSETUP with SR
// Telegram 6F SETPWM with SR
// Telegram 6F SETPWMCHN with SR
// Telegram 71 SETCURCHN with SR
// Telegram 73 SETTCOEFF with SR
// Telegram 75 SETADC with SR
// Telegram 77 WRITEI2CCFG with SR
// Telegram 79 SETOTP with SR
// Telegram 7F TESTPW with SR


#ifdef __cplusplus
}
#endif

#endif  // OSP2_H_






