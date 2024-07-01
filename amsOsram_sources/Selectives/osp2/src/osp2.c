// Selectives/osp2/src/osp2.c
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


#include <string.h>
#include <stdio.h>
#include <Hal/CY_SPI/inc/spiGeneral.h>
#include <Hal/Osire/inc/osire.h>
#include <SwTimer/inc/swTimer.h>
#include <Crc/inc/crc.h>
#include <osp2/inc/osp2.h>


// ==========================================================================
// Errors

// Readable string
char * osp2_error_str(osp2_error_t err) {
  switch( err ) {
    case OSP2_ERROR_NONE                                   : return "NONE";
    case OSP2_ERROR_SPI_MAX_LENGHT_TO_HIGH_SPI             : return "SPI_MAX_LENGHT_TO_HIGH_SPI";
    case OSP2_ERROR_SPI_BUSY                               : return "SPI_BUSY";
    case OSP2_ERROR_SPI_SEND                               : return "SPI_SEND";
    case OSP2_ERROR_SPI_RECEIVE                            : return "SPI_RECEIVE";
    case OSP2_ERROR_SPI_DUMMY_RECEIVE                      : return "SPI_DUMMY_RECEIVE";
    case OSP2_ERROR_SPI_TIME_OUT                           : return "SPI_TIME_OUT";
    case OSP2_ERROR_SPI_NO_RESET_NON_BLOCKING_SEND         : return "SPI_NO_RESET_NON_BLOCKING_SEND";
    case OSP2_ERROR_SPI_COULD_NOT_ADD_NEW_DATA_BUFFER_FULL : return "SPI_COULD_NOT_ADD_NEW_DATA_BUFFER_FULL";
    case OSP2_ERROR_SPI_NO_NEW_DATA_RECEIVED               : return "SPI_NO_NEW_DATA_RECEIVED";
    case OSP2_ERROR_SPI_CORRUPT_DATA                       : return "SPI_CORRUPT_DATA";
    case OSP2_ERROR_SPI_NOT_DEF                            : return "SPI_NOT_DEF";
    case OSP2_ERROR_ARG                                    : return "ARG";
    case OSP2_ERROR_ADDR                                   : return "ADDR";
    case OSP2_ERROR_OUTARGNULL                             : return "OUTARGNULL";
    case OSP2_ERROR_TELEFIELD                              : return "TELEFIELD";
    case OSP2_ERROR_CRC                                    : return "CRC";
    case OSP2_ERROR_ID                                     : return "ID";
    case OSP2_ERROR_I2CNACK                                : return "I2CNACK";
    case OSP2_ERROR_I2CTIMEOUT                             : return "I2CTIMEOUT";
    case OSP2_ERROR_STATUSFLAGS                            : return "STATUSFLAGS";
    case OSP2_ERROR_OUTOFMEM                               : return "OUTOFMEM";
    case OSP2_ERROR_MISSI2CBRIDGE                          : return "MISSI2CBRIDGE";
    case OSP2_ERROR_MISSI2CDEV                             : return "MISSI2CDEV";
    case OSP2_ERROR_DATA                                   : return "DATA";
    default                                                : return "<unknown>";
  }
}


// ==========================================================================
// Helpers to pretty print telegram fields. Some are OSIRE or SAID specific

static const char * osp2_stat_names[]         = { "UNINTIALIZED", "SLEEP", "ACTIVE", "DEEPSLEEP" };                // RGBi uses name UNINITIALIZED (SAID uses INITIALIZED but that seems wrong)
static const char * osp2_stat_flags46_osire[] = { "ol", "oL", "Ol", "OL" };                                        // Otp error, bidir/Loop // RGBi only
static const char * osp2_stat_flags46_said [] = { "tv", "tV", "Tv", "TV" };                                        // Test mode (or OTP err), over Voltage // SAID only
static const char * osp2_stat_flags04[]       = { "clou", "cloU", "clOu", "clOU", "cLou", "cLoU", "cLOu", "cLOU",  // Communication, LED, Over temperature, Under voltage // RGBi and SAID
                                                  "Clou", "CloU", "ClOu", "ClOU", "CLou", "CLoU", "CLOu", "CLOU",};
static const char * osp2_setup_flags48[]      = { "pcct", "pccT", "pcCt", "pcCT",  "pcct", "pCcT", "pCCt", "pCCT", // PWM fast, mcu spi CLK inverted, CRC check enabled, Temp sensor slow rate
                                                  "Pcct", "PccT", "PcCt", "pcCT",  "Pcct", "PCcT", "PCCt", "PCCT", };
static const char * osp2_com_names[]          = { "LVDS", "EOL","MCU", "CAN" };
static const char * osp2_curchn_flags[]       = { "rshd", "rshD", "rsHd", "rsHD", "rShd", "rShD", "rSHd", "rSHD", // Reserved, Sync enabled, Hybrid PWM, Dithering enabled
                                                  "Rshd", "RshD", "RsHd", "RsHD", "RShd", "RShD", "RSHd", "RSHD",};
static const char * osp2_i2ccfg_flags[]       = { "itnb", "itnB", "itNb", "itNB", "iTnb", "iTnB", "iTNb", "iTNB", // Interrupt, Twelve bit addressing, Nack/ack, I2C transaction Busy
                                                  "itnb", "itnB", "itNb", "itNB", "iTnb", "iTnB", "iTNb", "iTNB", };



#define BITS_MASK(n)                  ((1<<(n))-1)                           // number of bits set BITS_MASK(3)=0b111 (max n=31)
#define BITS_SLICE(v,lo,hi)           ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) ) // including lo, excluding hi


// Pretty print functions that return a char * all use the one global buffer osp2_str_buf[].
// This means that only one of the pretty printer can be used at a time,
// eg this will not work: printf("%s-%s", osp2_stat_osire_str(x), osp2_stat_said_str(x) ).
#define OSP2_STR_BUF_SIZE 40
static char osp2_buf[OSP2_STR_BUF_SIZE];

// Converts OSIRE raw temperature to Celsius.
int osp2_temp_osire(uint8_t temp) {
  return ((int)(temp)*108+50)/100-126;
}
// Converts SAID raw temperature to Celsius.
int osp2_temp_said(uint8_t temp) {
  return (int)(temp)-86;
}
// SLEEP/oL/clou
char * osp2_stat_osire_str(uint8_t stat) {
  snprintf( osp2_buf, OSP2_STR_BUF_SIZE, "%s:%s:%s",
    osp2_stat_names[BITS_SLICE(stat,6,8)], osp2_stat_flags46_osire[BITS_SLICE(stat,4,6)], osp2_stat_flags04[BITS_SLICE(stat,0,4)] );
  return osp2_buf;
}
// ACTIVE/tv/clou
char * osp2_stat_said_str(uint8_t stat) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%s:%s:%s",
    osp2_stat_names[BITS_SLICE(stat,6,8)], osp2_stat_flags46_said[BITS_SLICE(stat,4,6)], osp2_stat_flags04[BITS_SLICE(stat,0,4)] );
  return osp2_buf;
}
// 0.0000/0.0000/0.0000
char * osp2_pwm_osire_str(uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%X.%04X:%X.%04X:%X.%04X",
    BITS_SLICE(daytimes,2,3), red, BITS_SLICE(daytimes,1,2), green, BITS_SLICE(daytimes,0,1), blue );
  return osp2_buf;
}
// 0000/0000/0000
char * osp2_pwm_said_str(uint16_t red, uint16_t green, uint16_t blue) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%04X:%04X:%04X", red, green, blue );
  return osp2_buf;
}
// LVDS/LVDS
char * osp2_com_osire_str(uint8_t com) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%s:%s",
    osp2_com_names[BITS_SLICE(com,2,4)], osp2_com_names[BITS_SLICE(com,0,2)]  );
  return osp2_buf;
}
// LOOP/LVDS/LVDS
char * osp2_com_said_str(uint8_t com) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%s:%s:%s",
    BITS_SLICE(com,4,5)?"LOOP":"BIDIR", osp2_com_names[BITS_SLICE(com,2,4)], osp2_com_names[BITS_SLICE(com,0,2)]  );
  return osp2_buf;
}
// pccT/clOU
char * osp2_setup_str(uint8_t flags) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%s:%s",
    osp2_setup_flags48[BITS_SLICE(flags,4,8)], osp2_stat_flags04[BITS_SLICE(flags,0,4)]  );
  return osp2_buf;
}
// 1D 9F 95 6F 42 00 00 00
char * osp2_buf_str(void * buf, int size ) { // may be misused to print a telegram as raw bytes
  char * p = osp2_buf;
  int remain = OSP2_STR_BUF_SIZE;
  for( int i=0; i<size; i++ ) {
    int num = snprintf(p, remain, "%02X ", ((uint8_t*)buf)[i] );
    p+=num;
    remain-=num;
  }
  if( size>0 ) *(p-1)='\0'; // overwrite last space
  return osp2_buf;
}
// rshd
char * osp2_curchn_str(uint8_t flags) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%s", osp2_curchn_flags[flags] );
  return osp2_buf;
}
// itnb
char * osp2_i2ccfg_str(uint8_t flags) {
  snprintf(osp2_buf, OSP2_STR_BUF_SIZE, "%s", osp2_i2ccfg_flags[flags] );
  return osp2_buf;
}
// Converts SAID raw I2C bus speed to bits/second
int osp2_i2ccfg_speed(uint8_t temp) {
  return 19*1000*1000 / ((int)(temp)<<3) / 2;
}


// ==========================================================================
// High level helpers (use 'exec' in their name)


// Sends a RESET telegram (with HAL manipulation and wait).
osp2_error_t osp2_exec_reset() {
  hal_reset_osire_start(); // Note sure what the HAL calls do; copied from old demo
  osp2_error_t err = osp2_send_reset(0);
  hal_reset_osire_end();
  Cy_SysLib_Delay(2); // After issuing the RESET command, wait for at least 150 us before sending out the next command.
  return err;
}


// Sends RESET/BIDIR telegrams, if that fails sends RESET/LOOP. Outputs address of last node (=number of nodes in chain) and loop/bidir direction.
osp2_error_t osp2_exec_resetinit(uint16_t *last, int *loop ) {
  osp2_error_t err;
  uint8_t temp;
  uint8_t stat;
  // reset
  err= osp2_exec_reset();
  if( err!=OSP2_ERROR_NONE ) return err;
  // init (try bidir then loop)
  *loop=-1; // 0=bidir, 1=loop (-1=undef)
  err= osp2_send_initbidir(1,last,&temp,&stat);
  if( err==OSP2_ERROR_NONE ) {
    *loop= 0;
  } else {
    err= osp2_exec_reset();
    if( err!=OSP2_ERROR_NONE ) return err;
    err= osp2_send_initloop(1,last,&temp,&stat);
    if( err!=OSP2_ERROR_NONE ) return err;
    *loop= 1;
  }
  return OSP2_ERROR_NONE;
}


// Reads OTP, and extracts I2C-ENABLE bit.
osp2_error_t osp2_exec_i2cenable_get(uint16_t addr, int * enable) {
  if( enable==0 ) return OSP2_ERROR_OUTARGNULL;
  uint8_t       otp_addr_i2cen = 0x0D;
  uint8_t       buf[8];

  // Read current OTP row
  osp2_error_t  err = osp2_send_readotp(addr,otp_addr_i2cen,buf,8);

  // Mask in the new I2C enable bit
  *enable = buf[0] & 0x01;

  return err;
}


// Reads OTP, updates I2C-ENABLE bit and writes it back to OTP (P2RAM cache actually).
// Still need to power the I2C pads: err= osp2_send_setcurchn(addr, /*chan*/2, /*flags*/0, 4, 4, 4);
osp2_error_t osp2_exec_i2cenable_set(uint16_t addr, int enable) {
  osp2_error_t  err;
  uint8_t       otp_addr_i2cen = 0x0D;
  uint8_t       buf[8];


  // This code is complicated by resource management (C implementation of try finally).
  // We set the password, but we _MUST_ undo that (otherwise this SAID garbles passing messages) .
  // The following flags indicate whether the password needs to unset.
  // That is done in the handler that we goto on error.
  int resource_pw=0;

  // Set password for writing
  err= osp2_send_testpw(addr,0x4247525F4143ULL);
  if( err!=OSP2_ERROR_NONE ) goto free_resources;
  resource_pw = 1; // password is set, so we need to unset it in case of errors

  // Read current OTP row
  err= osp2_send_readotp(addr,otp_addr_i2cen,buf,8);
  if( err!=OSP2_ERROR_NONE ) goto free_resources;

  // Mask in the new I2C enable bit
  buf[0] &= ~ 0x01;
  buf[0] |= (enable!=0);

  // Write back updated row
  err=osp2_send_setotp(addr,otp_addr_i2cen,buf,7);
  if( err!=OSP2_ERROR_NONE ) goto free_resources;

  // Ended without errors, still need to free resources
  err=OSP2_ERROR_NONE;

  // Clean up by freeing claimed resources (that is, unset password)
free_resources:
  if( resource_pw ) osp2_send_testpw(addr,0); // no further error handling here
  return err;
}


// Reads the whole SAID otp and prints private and customer area
osp2_error_t osp2_exec_otpdump(uint16_t addr) {
  #define OTPSIZE 0x20
  #define OTPSTEP 8
  osp2_error_t   err;
  uint8_t        otp[OTPSIZE];

  for( int otpaddr=0x00; otpaddr<OTPSIZE; otpaddr+=OTPSTEP ) {
    err= osp2_send_readotp(addr,otpaddr,otp+otpaddr,OTPSTEP);
    if( err!=OSP2_ERROR_NONE ) return err;
  }

  printf("OTP 0x%02X:",0x00);
  for( int otpaddr=0x00; otpaddr<0x0D; otpaddr+=1 ) printf(" %02X",otp[otpaddr]);
  printf("\n");

  printf("OTP 0x%02X:",0x0D);
  for( int otpaddr=0x0D; otpaddr<0x20; otpaddr+=1 ) printf(" %02X",otp[otpaddr]);
  printf("\n");

  return err;
}


// Writes count bytes from buf, into register raddr in i2c device daddr7, attached to node addr.
osp2_error_t osp2_exec_i2cwrite8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, uint8_t count) {
  // Send an I2C write telegram
  osp2_error_t err = osp2_send_i2cwrite8(addr,daddr7,raddr,buf,count);
  if( err!=OSP2_ERROR_NONE ) return err;
  // Wait (with timeout) until I2C transaction is completed (not busy)
  uint8_t flags=OSP2_I2CCFG_FLAGS_BUSY;
  uint8_t tries=10;
  while( (flags&OSP2_I2CCFG_FLAGS_BUSY) && (tries>0) ) {
    uint8_t speed;
    err = osp2_send_readi2ccfg(addr,&flags,&speed);
    if( err!=OSP2_ERROR_NONE ) return err;
    Cy_SysLib_Delay(1);
    tries--;
  }
  // Was transaction successful
  if( flags & OSP2_I2CCFG_FLAGS_BUSY ) return OSP2_ERROR_I2CTIMEOUT;
  if( flags & OSP2_I2CCFG_FLAGS_NACK ) return OSP2_ERROR_I2CNACK;
  return OSP2_ERROR_NONE;
}


// Reads count bytes into buf, from register raddr in i2c device daddr7, attached to node addr.
osp2_error_t osp2_exec_i2cread8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t *buf, uint8_t count) {
  // Send an I2C read telegram
  osp2_error_t err = osp2_send_i2cread8(addr,daddr7,raddr,count);
  if( err!=OSP2_ERROR_NONE ) return err;
  // Wait (with timeout) until I2C transaction is completed (not busy)
  uint8_t flags=OSP2_I2CCFG_FLAGS_BUSY;
  uint8_t tries=10;
  while( (flags&OSP2_I2CCFG_FLAGS_BUSY) && (tries>0) ) {
    uint8_t speed;
    err = osp2_send_readi2ccfg(addr,&flags,&speed);
    if( err!=OSP2_ERROR_NONE ) return err;
    Cy_SysLib_Delay(1);
    tries--;
  }
  // Was transaction successful
  if( flags & OSP2_I2CCFG_FLAGS_BUSY ) return OSP2_ERROR_I2CTIMEOUT;
  if( flags & OSP2_I2CCFG_FLAGS_NACK ) return OSP2_ERROR_I2CNACK;
  // Get the read bytes
  err = osp2_send_readlast(addr,buf,count);
  if( err!=OSP2_ERROR_NONE ) return err;
  return OSP2_ERROR_NONE;
}



// ==========================================================================
// Logging


// todo: change to level 0=none, 1=in/out, 2=with telegram
static int osp2_log_level = OSP2_LOG_LEVEL_NONE;

void osp2_log_set_level(int level) {
  osp2_log_level = level;
}

int osp2_log_get_level() {
  return osp2_log_level;
}


// ==========================================================================
/*
Per telegram ID there are three functions.
Let the telegram name (ID) be xxx, argx the arguments and resx the results in the response.


The first function constructs a telegram, eg converts (int) arguments to a byte array.
  static osp2_error_t osp2_con_xxx(osp2_tele_t * tele, uint16_t addr, uintx_t arg0, uintx_t arg1, ... , uint8_t * respsize)

  - osp2_tele_t * tele    (OUT) caller allocated (content irrelevant), filled with preamble/addr/tid/args/crc on exit
  - uint16_t addr         (IN)  is the address of the destination node (unicast), or 0 (broadcast), or a group address (3F0..2FE)
  - uintx_t arg0          (IN)  telegram xxx specific argument
  - uintx_t arg1          (IN)  telegram xxx specific argument
  - ...
  - uint8_t * respsize    (OUT) if not NULL set to expected response size (telegram size, so payload size plus 4, in bytes)
                                telegrams without a response do not have this out parameter

  - returns OSP2_ERROR_NONE if all ok, or OSP2_ERROR_ARG, OSP2_ERROR_ADDR or OSP2_ERROR_OUTARGNULL.


The second function destructs a (response) telegram, e.g. converts a byte array to (int) error fields
  static osp2_error_t osp2_des_xxx(osp2_tele_t * tele, uintx_t * res0, uintx_t * res1, ... )
  - osp2_tele_t * tele    (IN)  caller allocated, checked for correct telegram id, size and CRC

  - uintx_t * res0        (OUT) set to response field (telegram xxx specific response)
  - uintx_t * res1        (OUT) set to response field (telegram xxx specific response)
  - ...

  - returns OSP2_ERROR_NONE if all ok, or OSP2_ERROR_TELEFIELD, OSP2_ERROR_CRC, or OSP2_ERROR_OUTARGNULL.

The third function is a helper.
  osp2_error_t osp2_send_xxx(...)
  - has all payload arguments of osp2_con_xxx, then has all result fields of osp2_des_xxx
    in other words, all arguments of both functions with the exception of tele, respsize, tele
  - constructs a telegram, sends it, waits for response, destructs it.
  - optionally prints logging info
  - returns OSP2_ERROR_NONE if all ok, or any of the constructor or destructor errors, or OSP2_ERROR_SPI_XXX.

*/


#define PSI(payloadsize)              ((payloadsize)<8 ? (payloadsize) : 7)


// ==========================================================================
// Telegram 00 RESET
static osp2_error_t osp2_con_reset(osp2_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x00; // RESET
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_reset(uint16_t addr) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_reset(&tele,addr);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("reset(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n");
  };

  return error;
}


// ==========================================================================
// Telegram 01 CLRERROR
static osp2_error_t osp2_con_clrerror(osp2_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x01; // CLRERROR
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_clrerror(uint16_t addr) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_clrerror(&tele,addr);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("clrerror(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n");
  };

  return error;
}


// ==========================================================================
// Telegram 02 INITBIDIR
static osp2_error_t osp2_con_initbidir(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x02; // INITBIDIR
  if( respsize ) *respsize = 4+2; // temp, stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_initbidir(osp2_tele_t * tele, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Check telegram consistency
  if( tele==0 || last==0 || temp==0 || stat==0 ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x02      ) return OSP2_ERROR_TELEFIELD;
  if( tele->size!=4+2                          ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0          ) return OSP2_ERROR_CRC;

  // Get fields
  *last = BITS_SLICE(tele->data[0],0,4)<<6 | BITS_SLICE(tele->data[1],2,8);
  *temp = tele->data[3];
  *stat = tele->data[4];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_initbidir(uint16_t addr, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_initbidir(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_initbidir(&resp, last, temp, stat);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("initbidir(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" last=0x%02X=%d temp=0x%02X=%d stat=0x%02X=%s",  *last, *last,
      *temp, osp2_temp_said(*temp), *stat, osp2_stat_said_str(*stat) );
    printf(" (%d, %s)\n",  osp2_temp_osire(*temp), osp2_stat_osire_str(*stat) );
  }

  return error;
}


// ==========================================================================
// Telegram 03 INITLOOP
static osp2_error_t osp2_con_initloop(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x03; // INITLOOP
  if( respsize ) *respsize = 4+2; // temp, stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_initloop(osp2_tele_t * tele, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Check telegram consistency
  if( tele==0 || last==0 || temp==0 || stat==0 ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x03      ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+2                          ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0          ) return OSP2_ERROR_CRC;

  // Get fields
  *temp = tele->data[3];
  *stat = tele->data[4];
  *last = BITS_SLICE(tele->data[0],0,4)<<6 | BITS_SLICE(tele->data[1],2,8);

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_initloop(uint16_t addr, uint16_t * last, uint8_t * temp, uint8_t * stat) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_initloop(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_initloop(&resp, last, temp, stat);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("initloop(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" last=0x%02X=%d temp=0x%02X=%d stat=0x%02X=%s",  *last, *last,
      *temp, osp2_temp_said(*temp), *stat, osp2_stat_said_str(*stat) );
    printf(" (%d, %s)\n",  osp2_temp_osire(*temp), osp2_stat_osire_str(*stat) );
  }

  return error;
}


// ==========================================================================
// Telegram 04 GOSLEEP
static osp2_error_t osp2_con_gosleep(osp2_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x04; // GOSLEEP
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_gosleep(uint16_t addr) {
	  // Telegram and error vars
	  osp2_tele_t     tele;
	  osp2_error_t    error    = OSP2_ERROR_NONE;
	  osp2_error_t    con_error= OSP2_ERROR_NONE;
	  errorSpi_t      spi_error= NO_ERROR_SPI;

	  // Construct, send and optionally destruct
	  con_error = osp2_con_gosleep(&tele,addr);
	  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
	  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
	  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

	  // Log
	  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
	    printf("gosleep(0x%03X)",addr);
	    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
	    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
	      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
	    printf("\n");
	  };

	  return error;
	}

// ==========================================================================
// Telegram 05 GOACTIVE
static osp2_error_t osp2_con_goactive(osp2_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x05; // GOACTIVE
  //if( respsize ) *respsize = 4+0;


  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_goactive(uint16_t addr) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_goactive(&tele,addr);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("goactive(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n");
  }

  return error;
}



// ==========================================================================
// Telegram 06 GODEEPSLEEP

static osp2_error_t osp2_con_godeepsleep(osp2_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x06; // GOSLEEP
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_godeepsleep(uint16_t addr) {
	  // Telegram and error vars
	  osp2_tele_t     tele;
	  osp2_error_t    error    = OSP2_ERROR_NONE;
	  osp2_error_t    con_error= OSP2_ERROR_NONE;
	  errorSpi_t      spi_error= NO_ERROR_SPI;

	  // Construct, send and optionally destruct
	  con_error = osp2_con_godeepsleep(&tele,addr);
	  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
	  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
	  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

	  // Log
	  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
	    printf("godeepsleep(0x%03X)",addr);
	    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
	    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
	      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
	    printf("\n");
	  };

	  return error;
	}
// ==========================================================================
// Telegram 07 IDENTIFY
static osp2_error_t osp2_con_identify(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x07; // IDENTIFY
  if( respsize ) *respsize = 4+4; // id

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_identify(osp2_tele_t * tele, uint32_t * id) {
  // Check telegram consistency
  if( tele==0 || id==0                    ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x07 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+4                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *id = (uint32_t)(tele->data[3])<<24 | (uint32_t)(tele->data[4])<<16 | (uint32_t)(tele->data[5])<<8 | (uint32_t)(tele->data[6]);

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_identify(uint16_t addr, uint32_t * id) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_identify(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_identify(&resp, id);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("identify(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" id=0x%08lX\n",*id);
  }

  return error;
}


// ==========================================================================
// Telegram 08 P4ERRBIDIR
// Telegram 09 P4ERRLOOP
// Telegram 0A ASK_TINFO
// Telegram 0B ASK_VINFO
// Telegram 0C READMULT
// Telegram 0D SETMULT


// ==========================================================================
// Telegram 0F SYNC
static osp2_error_t osp2_con_sync(osp2_tele_t * tele, uint16_t addr) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x0F; // SYNC
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_sync(uint16_t addr) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_sync(&tele,addr);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("sync(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n");
  };

  return error;
}


// ==========================================================================
// Telegram 11 IDLE
// Telegram 12 FOUNDRY
// Telegram 13 CUST
// Telegram 14 BURN
// Telegram 15 AREAD
// Telegram 16 LOAD
// Telegram 17 GLOAD


// ==========================================================================
// Telegram 18 I2CREAD
static osp2_error_t osp2_con_i2cread8(osp2_tele_t * tele, uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t num) {
  // Check input parameters
  if( tele==0                     ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS   ) return OSP2_ERROR_ADDR;
  if( daddr7>127                  ) return OSP2_ERROR_ARG;
  if( raddr>255                   ) return OSP2_ERROR_ARG;
  if( num<1 || num>8              ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 3;
  const uint8_t tid = 0x18; // I2CREAD
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = daddr7 << 1; // 7 bits address needs shifting
  tele->data[4] = raddr;
  tele->data[5] = num;

  tele->data[6] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_i2cread8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t num ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_i2cread8(&tele,addr,daddr7,raddr,num);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("i2cread(0x%03X,0x%02X,0x%02X,%d)",addr,daddr7,raddr,num );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 19 I2CWRITE
static osp2_error_t osp2_con_i2cwrite8(osp2_tele_t * tele, uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t * buf, int size) {
  // Check input parameters
  if( tele==0 || buf==0         ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( daddr7>127                ) return OSP2_ERROR_ARG;
  if( raddr>255                 ) return OSP2_ERROR_ARG;
  if( size<1                    ) return OSP2_ERROR_ARG; // SAID wants minimally one
  if( size+2>8                  ) return OSP2_ERROR_ARG; // telegram payload cannot exceed 8 bytes (two byte is for daddr/raddr)
  if( size+2==5 || size+2==7    ) return OSP2_ERROR_ARG; // telegram payloads 5 and 7 are not supported in OSP

  // Set constants
  const uint8_t payloadsize = 2+size; // daddr, raddr and buf(size)
  const uint8_t tid = 0x19; // I2CWRITE
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = daddr7 << 1; // 7 bits device address needs shifting
  tele->data[4] = raddr;
  for( int i=0; i<size; i++ ) tele->data[5+i] = buf[i];

  tele->data[3+2+size] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


// Allowed values for `size` are 1, 2, 4, and 6
osp2_error_t osp2_send_i2cwrite8(uint16_t addr, uint8_t daddr7, uint8_t raddr, uint8_t * buf, int size) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_i2cwrite8(&tele,addr,daddr7,raddr,buf,size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("i2cwrite(0x%03X,0x%02X,0x%02X,%s)",addr,daddr7,raddr,osp2_buf_str(buf,size));
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 1E READLAST
static osp2_error_t osp2_con_readlast(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x1E; // READLAST
  if( respsize ) *respsize = 4+8; // i2c read buffer

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readlast(osp2_tele_t * tele, uint8_t * buf, int size) {
  // Check telegram consistency
  if( tele==0 || buf==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x1E ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+8                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;
  if( size<1 || size>8                    ) return OSP2_ERROR_ARG;

  // Get fields
  for( int i=0; i<size; i++ ) buf[i] = tele->data[11-size+i];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readlast(uint16_t addr, uint8_t * buf, int size) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readlast(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readlast(&resp, buf, size);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readlast(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" i2c %s\n", osp2_buf_str(buf,size) );
  }

  return error;
}


// ==========================================================================
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


// ==========================================================================
// Telegram 40 READSTAT
static osp2_error_t osp2_con_readstat(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x40; // READSTAT
  if( respsize ) *respsize = 4+1; //  stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readstat(osp2_tele_t * tele, uint8_t * stat) {
  // Check telegram consistency
  if( tele==0 || stat==0                  ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x40 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!= 4+1                    ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *stat = tele->data[3];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readstat(uint16_t addr, uint8_t * stat) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readstat(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readstat(&resp, stat);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readstat(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" stat=0x%02X=%s", *stat, osp2_stat_said_str(*stat) );
    printf(" (%s)\n", osp2_stat_osire_str(*stat) );
  }

  return error;
}


// ==========================================================================
// Telegram 42 READTEMPSTAT
static osp2_error_t osp2_con_readtempstat(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x42; // READTEMPSTAT
  if( respsize ) *respsize = 4+2; // temp, stat

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readtempstat(osp2_tele_t * tele, uint8_t * temp, uint8_t * stat) {
  // Check telegram consistency
  if( tele==0 || temp==0 || stat==0       ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x42 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+2                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *temp = tele->data[3];
  *stat = tele->data[4];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readtempstat(uint16_t addr, uint8_t * temp, uint8_t * stat) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readtempstat(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readtempstat(&resp, temp, stat);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readtempstat(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" temp=0x%02X=%d stat=0x%02X=%s", *temp, osp2_temp_said(*temp), *stat, osp2_stat_said_str(*stat) );
    printf(" (%d, %s)\n",  osp2_temp_osire(*temp), osp2_stat_osire_str(*stat) );
  }

  return error;
}


// ==========================================================================
// Telegram 44 READCOMST
static osp2_error_t osp2_con_readcomst(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x44; // READCOMST
  if( respsize ) *respsize = 4+1; // comst

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readcomst(osp2_tele_t * tele, uint8_t * com) {
  // Check telegram consistency
  if( tele==0 || com==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x44 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+1                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *com= tele->data[3];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readcomst(uint16_t addr, uint8_t * com) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readcomst(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readcomst(&resp, com);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    // Log
    printf("readcomst(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" com=0x%02X=%s", *com, osp2_com_said_str(*com) );
    printf(" (%s)\n", osp2_com_osire_str(*com) );
  }

  return error;
}




// ==========================================================================
// Telegram 46 READLEDST


// ==========================================================================
// Telegram 48 READTEMP
static osp2_error_t osp2_con_readtemp(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x48; // READTEMP
  if( respsize ) *respsize = 4+1; // temp

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readtemp(osp2_tele_t * tele, uint8_t * temp) {
  // Check telegram consistency
  if( tele==0 || temp==0                  ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x48 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!= 4+1                    ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *temp = tele->data[3];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readtemp(uint16_t addr, uint8_t * temp) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readtemp(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readtemp(&resp, temp);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readtemp(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" temp=0x%02X=%d", *temp, osp2_temp_said(*temp) );
    printf(" (%d)\n", osp2_temp_osire(*temp) );
  }

  return error;
}


// ==========================================================================
// Telegram 4A READOTTH
// Telegram 4B SETOTTH



// ==========================================================================
// Telegram 4C READSETUP
static osp2_error_t osp2_con_readsetup(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x4C; // READSETUP
  if( respsize ) *respsize = 4+1; // flags

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readsetup(osp2_tele_t * tele, uint8_t *flags ) {
  // Check telegram consistency
  if( tele==0 || flags==0                 ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x4C ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+1                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *flags= tele->data[3];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readsetup(uint16_t addr, uint8_t *flags ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readsetup(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readsetup(&resp, flags);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readsetup(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" flags=0x%02X=%s\n", *flags, osp2_setup_str(*flags) );
  }

  return error;
}


// ==========================================================================
// Telegram 4D SETSETUP
static osp2_error_t osp2_con_setsetup(osp2_tele_t * tele, uint16_t addr, uint8_t flags) {
  // Check input parameters
  if( tele==0               ) return OSP2_ERROR_OUTARGNULL;
  if( addr> MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x4D; // SETSETUP
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = flags;

  tele->data[4] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_setsetup(uint16_t addr, uint8_t flags) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_setsetup(&tele, addr, flags);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("setsetup(0x%03X,0x%02X)",addr,flags);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}



// ==========================================================================
// Telegram 4E READPWM (OSIRE only) - the three 1-bit daytimes flags are clubbed into one daytimes argument
static osp2_error_t osp2_con_readpwm(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x4E; // READPWM
  if( respsize ) *respsize = 4+6; // red, green, blue

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readpwm(osp2_tele_t * tele, uint16_t *red, uint16_t *green, uint16_t *blue, uint8_t *daytimes) {
  // Check telegram consistency
  if( tele==0 || red==0 || green==0 || blue==0 || daytimes==0 ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA                      ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x4E                     ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+6                                         ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0                         ) return OSP2_ERROR_CRC;

  // Get fields
  *red  = BITS_SLICE(tele->data[3],0,7)<<8 | tele->data[4] ;
  *green= BITS_SLICE(tele->data[5],0,7)<<8 | tele->data[6] ;
  *blue = BITS_SLICE(tele->data[7],0,7)<<8 | tele->data[8] ;
  *daytimes = BITS_SLICE(tele->data[3],7,8)<<2 | BITS_SLICE(tele->data[5],7,8)<<1 |  BITS_SLICE(tele->data[7],7,8);

  return OSP2_ERROR_NONE;
}



osp2_error_t osp2_send_readpwm(uint16_t addr, uint16_t *red, uint16_t *green, uint16_t *blue, uint8_t *daytimes) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readpwm(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readpwm(&resp, red, green, blue, daytimes);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readpwm(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" rgb=%s\n", osp2_pwm_osire_str(*red,*green,*blue,*daytimes) );
  }

  return error;
}


// ==========================================================================
// Telegram 4E READPWMCHN (SAID only) - the meaning of the 16 color bits varies, not detailed here at telegram level
static osp2_error_t osp2_con_readpwmchn(osp2_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( chn!=0 && chn!=1 && chn!=2) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x4E; // READPWMCHN
  if( respsize ) *respsize = 4+6; // red, green, blue

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readpwmchn(osp2_tele_t * tele, uint16_t *red, uint16_t *green, uint16_t *blue ) {
  // Check telegram consistency
  if( tele==0 || red==0 || green==0 || blue==0 ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA       ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x4E      ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+6                          ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0          ) return OSP2_ERROR_CRC;

  // Get fields
  *red  = tele->data[3]<<8 | tele->data[4] ;
  *green= tele->data[5]<<8 | tele->data[6] ;
  *blue = tele->data[7]<<8 | tele->data[8] ;

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readpwmchn(uint16_t addr, uint8_t chn, uint16_t *red, uint16_t *green, uint16_t *blue ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readpwmchn(&tele,addr,chn,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readpwmchn(&resp, red, green, blue);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readpwmchn(0x%03X,%X)",addr,chn);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" rgb=%s\n", osp2_pwm_said_str(*red,*green,*blue) );
  }

  return error;
}


// ==========================================================================
// Telegram 4F SETPWM (OSIRE only) - the three 1-bit daytimes flags are clubbed into one daytimes argument
static osp2_error_t osp2_con_setpwm(osp2_tele_t * tele, uint16_t addr, uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes) {
  // Check input parameters
  if( tele==0                    ) return OSP2_ERROR_OUTARGNULL;
  if( addr     > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( red      & ~BITS_MASK(15)  ) return OSP2_ERROR_ARG;
  if( green    & ~BITS_MASK(15)  ) return OSP2_ERROR_ARG;
  if( blue     & ~BITS_MASK(15)  ) return OSP2_ERROR_ARG;
  if( daytimes & ~BITS_MASK(3)   ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 6;
  const uint8_t tid = 0x4F; // SETPWM
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = BITS_SLICE(daytimes,2,3)<<7 | BITS_SLICE(red,8,15);
  tele->data[4] = BITS_SLICE(red,0,8);
  tele->data[5] = BITS_SLICE(daytimes,1,2)<<7 | BITS_SLICE(green,8,15);
  tele->data[6] = BITS_SLICE(green,0,8);
  tele->data[7] = BITS_SLICE(daytimes,0,1)<<7 | BITS_SLICE(blue,8,15);
  tele->data[8] = BITS_SLICE(blue,0,8);

  tele->data[9] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_setpwm(uint16_t addr, uint16_t red, uint16_t green, uint16_t blue, uint8_t daytimes) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_setpwm(&tele, addr, red, green, blue, daytimes);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("setpwm(0x%03X,0x%04X,0x%04X,0x%04X,%X)",addr,red,green,blue,daytimes);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 4F SETPWMCHN (SAID only) - the meaning of the 16 color bits varies, not detailed here at telegram level
static osp2_error_t osp2_con_setpwmchn(osp2_tele_t * tele, uint16_t addr, uint8_t chn, uint16_t red, uint16_t green, uint16_t blue) {
  // Check input parameters
  if( tele==0                    ) return OSP2_ERROR_OUTARGNULL;
  if( addr     > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( chn!=0 && chn!=1 && chn!=2 ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 8;
  const uint8_t tid = 0x4F; // SETPWMCHN (same SETPWM of OSP V1)
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;
  tele->data[4] = 0xFF; // dummy;

  tele->data[5] = BITS_SLICE(red,8,15);
  tele->data[6] = BITS_SLICE(red,0,8);
  tele->data[7] = BITS_SLICE(green,8,15);
  tele->data[8] = BITS_SLICE(green,0,8);
  tele->data[9] = BITS_SLICE(blue,8,15);
  tele->data[10]= BITS_SLICE(blue,0,8);

  tele->data[11] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_setpwmchn(uint16_t addr, uint8_t chn, uint16_t red, uint16_t green, uint16_t blue) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_setpwmchn(&tele, addr, chn, red, green, blue);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("setpwmchn(0x%03X,%X,0x%04X,0x%04X,0x%04X)",addr,chn,red,green,blue);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 50 READCURCHN
static osp2_error_t osp2_con_readcurchn(osp2_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( chn!=0 && chn!=1 && chn!=2) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x50; // READCURCHN
  if( respsize ) *respsize = 4+2; // 3*4 PWM bits, 4 flags

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readcurchn(osp2_tele_t * tele, uint8_t *flags, uint8_t *rcur, uint8_t *gcur, uint8_t *bcur ) {
  // Check telegram consistency
  if( tele==0 || flags==0 || rcur==0 || gcur==0 || bcur==0 ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA                   ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x50                  ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+2                                      ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0                      ) return OSP2_ERROR_CRC;

  // Get fields
  *flags= BITS_SLICE(tele->data[3],4,8);
  *rcur  = BITS_SLICE(tele->data[3],0,4);
  *gcur= BITS_SLICE(tele->data[4],4,8);
  *bcur = BITS_SLICE(tele->data[4],0,4);

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readcurchn(uint16_t addr, uint8_t chn, uint8_t *flags, uint8_t *rcur, uint8_t *gcur, uint8_t *bcur ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readcurchn(&tele,addr,chn,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readcurchn(&resp, flags, rcur, gcur, bcur);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readcurchn(0x%03X,%X)",addr,chn);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" flags=%s rcur=%X gcur=%X bcur=%X\n", osp2_curchn_str(*flags), *rcur,*gcur,*bcur );
  }

  return error;
}


// ==========================================================================
// Telegram 51 SETCURCHN
static osp2_error_t osp2_con_setcurchn(osp2_tele_t * tele, uint16_t addr, uint8_t chn, uint8_t flags, uint8_t rcur, uint8_t gcur, uint8_t bcur )
{
  /* Check input parameters */
  if( tele == NULL             ) 							return OSP2_ERROR_OUTARGNULL;
  if( addr > MAXIMUM_ADDRESS   ) 							return OSP2_ERROR_ADDR;
  if( flags  & ~0x07           ) 							return OSP2_ERROR_ARG;
  if(((rcur > 0x04) && (rcur < 0x08)) || (rcur > 0x0B)) 	return OSP2_ERROR_ARG;
  if(((gcur > 0x04) && (gcur < 0x08)) || (gcur > 0x0B)) 	return OSP2_ERROR_ARG;
  if(((bcur > 0x04) && (bcur < 0x08)) || (bcur > 0x0B)) 	return OSP2_ERROR_ARG;
  if( chn > 2 ) 											return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 3;
  const uint8_t tid = 0x51; // SETCURCHN
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = chn;

  tele->data[4] = flags<<4 | rcur;
  tele->data[5] = gcur<<4 | bcur ;

  tele->data[6] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_setcurchn(uint16_t addr, uint8_t chn, uint8_t flags, uint8_t rcur, uint8_t gcur, uint8_t bcur ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_setcurchn(&tele, addr, chn, flags, rcur, gcur, bcur);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("setcurchn(0x%03X,%X,%s,%X,%X,%X)",addr,chn,osp2_curchn_str(flags),rcur,gcur,bcur);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 52 READ_T_COEFF
// Telegram 53 SET_T_COEFF
// ==========================================================================
// Telegram 54 READ_ADC
static osp2_error_t osp2_con_ADCread(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x54; // READADC
  if( respsize ) *respsize = 4+1; // ADC read

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_ADCread(osp2_tele_t *tele, uint8_t *adc) {
  // Check telegram consistency
  if( tele==0 || adc==0                  ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x54 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!= 4+1                    ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *adc = tele->data[3];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_ADCread(uint16_t addr, uint8_t *adc) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_ADCread(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_ADCread(&resp, adc);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readADC(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" adc read = 0x%02X", *adc);
    printf(" (%d)\n", osp2_temp_osire(*adc) );
  }

  return error;
}

// ==========================================================================
// Telegram 55 SET_ADC
//
static osp2_error_t osp2_con_setADC(osp2_tele_t * tele, uint16_t addr, uint8_t flags) {
  // Check input parameters
  if( tele==0               ) return OSP2_ERROR_OUTARGNULL;
  if( addr> MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x55; // SETADC
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = flags;

  tele->data[4] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_setADC(uint16_t addr, uint8_t flags) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_setADC(&tele, addr, flags);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("setadc(0x%03X,0x%02X)",addr,flags);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}

// ==========================================================================
// Telegram 56 READI2CCFG
static osp2_error_t osp2_con_readi2ccfg(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x56; // READI2CCFG
  if( respsize ) *respsize = 4+1; // flags:speed

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readi2ccfg(osp2_tele_t * tele, uint8_t *flags, uint8_t * speed ) {
  // Check telegram consistency
  if( tele==0 || flags==0 || speed==0     ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x56 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+1                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *flags = BITS_SLICE(tele->data[3],4,8);
  *speed = BITS_SLICE(tele->data[3],0,4);

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readi2ccfg(uint16_t addr, uint8_t *flags, uint8_t *speed ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readi2ccfg(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readi2ccfg(&resp, flags, speed);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readi2ccfg(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" flags=0x%02X=%s speed=0x%02X=%d\n", *flags, osp2_i2ccfg_str(*flags), *speed, osp2_i2ccfg_speed(*speed) );
  }

  return error;
}


// ==========================================================================
// Telegram 57 SETI2CCFG
static osp2_error_t osp2_con_seti2ccfg(osp2_tele_t * tele, uint16_t addr, uint8_t flags, uint8_t speed ) {
  // Check input parameters
  if( tele==0               ) return OSP2_ERROR_OUTARGNULL;
  if( addr> MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( flags & ~0x0F         ) return OSP2_ERROR_ADDR;
  if( speed & ~0x0F         ) return OSP2_ERROR_ADDR;
  if( speed==0              ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x4D; // SETI2CCFG
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = flags << 4 | speed;

  tele->data[4] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_seti2ccfg(uint16_t addr, uint8_t flags, uint8_t speed ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_seti2ccfg(&tele, addr, flags, speed);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("seti2ccfg(0x%03X,0x%02X,0x%02X)",addr,flags,speed);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 58 READOTP
static osp2_error_t osp2_con_readotp(osp2_tele_t * tele, uint16_t addr, uint8_t otpaddr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( otpaddr > 0x1F            ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 1;
  const uint8_t tid = 0x58; // READOTP
  if( respsize ) *respsize = 4+8; // otp row

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = otpaddr;

  tele->data[4] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_readotp(osp2_tele_t * tele, uint8_t * buf, int size) {
  // Check telegram consistency
  if( tele==0 || buf==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x58 ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!=4+8                     ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;
  if( size<1 || size>8                    ) return OSP2_ERROR_ARG;

  // Get fields
  // OSP telegrams are big endian, C byte arrays are little endian, so reverse.
  for( int i=0; i<size; i++ ) buf[i] = tele->data[10-i];

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_readotp(uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_readotp(&tele,addr,otpaddr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_readotp(&resp, buf, size);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("readotp(0x%03X,0x%02X)",addr,otpaddr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" otp 0x%02X: %s\n", otpaddr, osp2_buf_str(buf,size) );
  }

  return error;
}


// ==========================================================================
// Telegram 59 SETOTP
// (1) SETOTP only works when the correct password is first sent using TESTPW.
// (2) SETOTP can only write blocks of 7 bytes, no more, no less.
// (3) SETOTP doesn't write to OTP, rather it writes to its shadow P2RAM.
// (4) P2RAM is initialized (copied) from OTP at startup.
// (5) P2RAM is non-persistent over power cycles.
// (6) P2RAM is persistent over a RESET telegram.
// (7) The I2C_EN (in OTP at 0D.0) is inspected by the SAID when sending it I2C telegrams.
// (8) The SPI_MODE (in OTP at 0D.3) is inspected by the SAID at startup, so P2RAM value is irrelevant.
// (9) At this moment I do not know how to flash P2RAM to OTP to make settings persistent.
static osp2_error_t osp2_con_setotp(osp2_tele_t * tele, uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;
  if( otpaddr > 0x1F            ) return OSP2_ERROR_ARG;
  if( size   != 7               ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 8; // 1 for otp target address, 7 for data
  const uint8_t tid = 0x59; // SETOTP
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  // OSP telegrams are big endian, C byte arrays are little endian, so reverse
  for( int i=0; i<size; i++ ) tele->data[9-i] = buf[i];
  tele->data[10] = otpaddr;

  tele->data[11] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_setotp(uint16_t addr, uint8_t otpaddr, uint8_t * buf, int size) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_setotp(&tele,addr,otpaddr,buf,size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("setotp(0x%03X,0x%02X,%s)",addr,otpaddr,osp2_buf_str(buf,size) );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 5A READTESTDATA

static osp2_error_t osp2_con_ADCdataread(osp2_tele_t * tele, uint16_t addr, uint8_t * respsize) {
  // Check input parameters
  if( tele==0                   ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 0;
  const uint8_t tid = 0x5C; // READ DATA ADC
  if( respsize ) *respsize = 4+2; // ADC data read

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


static osp2_error_t osp2_des_ADCdataread(osp2_tele_t *tele, uint16_t *adcdata) {
  // Check telegram consistency
  if( tele==0 || adcdata==0                  ) return OSP2_ERROR_OUTARGNULL;
  if( BITS_SLICE(tele->data[0],4,8)!=0xA  ) return OSP2_ERROR_TELEFIELD; // PREAMBLE
  if( BITS_SLICE(tele->data[2],0,7)!=0x5C ) return OSP2_ERROR_TELEFIELD; // TID
  if( tele->size!= 4+2                    ) return OSP2_ERROR_TELEFIELD;
  if( crc(tele->data,tele->size) != 0     ) return OSP2_ERROR_CRC;

  // Get fields
  *adcdata = (uint16_t) (tele->data[3]) << 8 | (uint16_t) (tele->data[4]);

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_ADCdataread(uint16_t addr, uint16_t *adcdata) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_tele_t     resp;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;
  osp2_error_t    des_error= OSP2_ERROR_NONE;

  // Construct, send and optionally destruct
  con_error = osp2_con_ADCdataread(&tele,addr,&resp.size);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_and_receive_data_over_spi_blocking(tele.data,resp.data,tele.size,resp.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;
  if(     error==OSP2_ERROR_NONE ) des_error = osp2_des_ADCdataread(&resp, adcdata);
  if( des_error!=OSP2_ERROR_NONE ) error=des_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
  printf("readDataADC(0x%03X)",addr);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
      else if( des_error!=OSP2_ERROR_NONE ) printf(" [destructor ERROR %s]", osp2_error_str(des_error) );
    printf(" ->" );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [resp %s]",osp2_buf_str(resp.data,resp.size));
    printf(" adc data read = 0x%02X \n", *adcdata);
////    dbg_printf(" (%d)\n", osp2_temp_osire(*adc) );
  }

  return error;
}



// ==========================================================================
// Telegram 57 SETTESTDATA
static osp2_error_t osp2_con_settestdata(osp2_tele_t * tele, uint16_t addr, uint16_t data ) {
  // Check input parameters
  if( tele==0               ) return OSP2_ERROR_OUTARGNULL;
  if( addr> MAXIMUM_ADDRESS ) return OSP2_ERROR_ADDR;

  // Set constants
  const uint8_t payloadsize = 2;
  const uint8_t tid = 0x5B; // SETTESTDATA
  //if( respsize ) *respsize = 4+0;

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  tele->data[3] = BITS_SLICE(data,8,16);
  tele->data[4] = BITS_SLICE(data,0,8);

  tele->data[5] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_settestdata(uint16_t addr, uint16_t data ) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_settestdata(&tele, addr, data);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("settestdata(0x%03X,0x%04X)",addr,data);
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 5C READ_ADC_DATA


// Telegram 5D TESTSCAN


// ==========================================================================
// Telegram 5F TESTPW
static osp2_error_t osp2_con_testpw(osp2_tele_t * tele, uint16_t addr, uint64_t pw) {
  // Check input parameters
  if( tele==0                     ) return OSP2_ERROR_OUTARGNULL;
  if( addr    > MAXIMUM_ADDRESS   ) return OSP2_ERROR_ADDR;
  if( pw & ~0x0000FFFFFFFFFFFFULL ) return OSP2_ERROR_ARG;

  // Set constants
  const uint8_t payloadsize = 6;
  const uint8_t tid = 0x5F; // TESTPW
  // if( respsize ) *respsize = 4+0; // nothing

  // Build telegram
  tele->size    = payloadsize+4;

  tele->data[0] = 0xA0 | BITS_SLICE(addr,6,10);
  tele->data[1] = BITS_SLICE(addr,0,6)<<2 | BITS_SLICE(PSI(payloadsize),1,3);
  tele->data[2] = BITS_SLICE(PSI(payloadsize),0,1)<<7 | tid;

  memcpy( &(tele->data[3]), &pw, 6); // 3..8

  tele->data[9] = crc( tele->data , tele->size - 1 );

  return OSP2_ERROR_NONE;
}


osp2_error_t osp2_send_testpw(uint16_t addr, uint64_t pw) {
  // Telegram and error vars
  osp2_tele_t     tele;
  osp2_error_t    error    = OSP2_ERROR_NONE;
  osp2_error_t    con_error= OSP2_ERROR_NONE;
  errorSpi_t      spi_error= NO_ERROR_SPI;

  // Construct, send and optionally destruct
  con_error = osp2_con_testpw(&tele,addr,pw);
  if( con_error!=OSP2_ERROR_NONE ) error=con_error;
  if(     error==OSP2_ERROR_NONE ) spi_error = send_data_over_spi_blocking(tele.data,tele.size);
  if( spi_error!=NO_ERROR_SPI    ) error=(osp2_error_t)spi_error;

  // Log
  if( osp2_log_level >= OSP2_LOG_LEVEL_ARG ) {
    printf("testpw(0x%03X,%s)",addr,osp2_buf_str(&pw,6) );
    if( osp2_log_level >= OSP2_LOG_LEVEL_TELE ) printf(" [tele %s]",osp2_buf_str(tele.data,tele.size));
    if( con_error!=OSP2_ERROR_NONE ) printf(" [constructor ERROR %s]", osp2_error_str(con_error) );
      else if( spi_error!=NO_ERROR_SPI ) printf(" [SPI ERROR %s]", osp2_error_str((osp2_error_t)spi_error) );
    printf("\n" );
  }

  return error;
}


// ==========================================================================
// Telegram 6B SETOTTH with SR
// Telegram 6D SETSETUP with SR
// Telegram 6F SETPWM with SR
// Telegram 6F SETPWMCHN with SR
// Telegram 71 SETCURCHN with SR
// Telegram 73 SETTCOEFF with SR
// Telegram 75 SETADC with SR
// Telegram 77 SETI2CCFG with SR
// Telegram 79 SETOTP with SR
// Telegram 7F TESTPW with SR




/* Review notes on OSP1
 * - Bitfield are not portable (due to padding, endianess) so don't use them to form telegrams
 * - ospCmdBuffer_t has in/out in field names, but in/out are properties of functions not of struct members
 * - ospCmdBuffer_t should be called telegram instead of command (a telegram can be either a command or a register read/write)
 * - ospCmdBuffer_t *p_cmdInfo argument name (cmdinfo) should be same as type name (cmdbuffer)
 * - global variable cmdBuffer makes whole lib non re-entrent
 * - splitting telegram composition between osireDevice (osp2_osire_set_pwm) and ospCmdBuffer (osp2_cmd_buffer) is too high coupling
 * - not possible to have set_pwm and set_pwm_chn in current style (switch/case in osp2_cmd_buffer wont work)
 * - osp lib functions should use osp as prefix, not osire
 */


















