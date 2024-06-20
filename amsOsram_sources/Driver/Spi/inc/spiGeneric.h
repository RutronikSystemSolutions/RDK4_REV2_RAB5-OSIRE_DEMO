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

#ifndef DRIVER_SPI_INC_SPIGENERIC_H_
#define DRIVER_SPI_INC_SPIGENERIC_H_

#ifdef __cplusplus
extern "C"
  {
#endif


#include <stdint.h>

typedef union
{
  uint32_t tcr;
  struct //Note: frameSize starts at bit0!
  {
    uint32_t frameSize :12;
    uint32_t reservedFillWithZero :4;
    uint32_t transWith :2;
    uint32_t transmitMask :1;
    uint32_t receiveMask :1;
    uint32_t contCommand :1;
    uint32_t contTrans :1;
    uint32_t byteSwap :1;
    uint32_t lsbFirst :1;
    uint32_t csn :2;
    uint32_t reserved :1;
    uint32_t prescal :3;
    uint32_t clkPhase :1;
    uint32_t clkPol :1;

  } bit;

} tcr_t;

typedef union
{
  uint32_t sr;
  struct
  {
    uint32_t transmitDataFlag :1;
    uint32_t recieveDataFlag :1;
    uint32_t reserved :7;
    uint32_t wordCompleteFlag :1;
    uint32_t frameCompleteFlag :1;
    uint32_t transferCompleteFlag :1;
    uint32_t transmitErrorFlag :1;
    uint32_t receiveErrorFlag :1;
    uint32_t dataMatchFlag :1;
    uint32_t reserved2 :10;
    uint32_t moduleBusyFlag :1;
    uint32_t reserved3 :7;
  } bit;

} spiStatusRegister_t;

typedef union
{
  uint32_t ier;
  struct
  {
    uint32_t transmitData :1;
    uint32_t receiveData :1;
    uint32_t reserved :6;
    uint32_t wordComplete :1;
    uint32_t frameComplete :1;
    uint32_t transferComplete :1;
    uint32_t transmitError :1;
    uint32_t receiveError :1;
    uint32_t dataMatch :1;
    uint32_t reserved2 :18;
  } bit;
} spiInterruptRegister_t;

typedef union
{
  uint32_t config1;
  struct
  {
    uint32_t masterMode :1;
    uint32_t samplePoint :1;
    uint32_t automaticPCS :1;
    uint32_t noStall :1;
    uint32_t reserved :4;
    uint32_t csnPolarity :4;
    uint32_t reserved2 :4;
    uint32_t matchConfig :3;
    uint32_t reserved3 :5;
    uint32_t pinConfig :2;
    uint32_t outputConfig :1;
    uint32_t csnConfig :1;
    uint32_t reserved4 :4;
  } bit;

} spiConfig1Register_t;

typedef union
{
  uint32_t config0;
  struct
  {
    uint32_t hostRequestEnabled :1;
    uint32_t HostRequestPolarity :1;
    uint32_t HostRequestSelect :1;
    uint32_t reserved :5;
    uint32_t circularFifoEnable :1;
    uint32_t receiveDataMatchOnly :1;
    uint32_t reserved2 :22;
  } bit;

} spiConfig0Register_t;

typedef union
{
  uint32_t fifo;
  struct
  {
    uint32_t transCount :3;
    uint32_t reserved :13;
    uint32_t receiveCount :3;
    uint32_t reserved2 :13;

  } bit;
} spiFifoStatus_t;
typedef enum
{
  SPI_DRIVER_COMMAND_NO_ERROR = 0,
  SPI_DRIVER_COMMAND_NO_RESET = 1,
  SPI_DRIVER_COMMAND_NO_RESET_NO_DATA_TO_SEND = 2

} spiCommandError_t;

typedef enum
{
  SPI_DRIVER_NO_ERROR = 0,
  SPI_DRIVER_RECEIVE_ERROR = 1,
  SPI_DRIVER_SEND_ERROR = 2,
  SPI_DRIVER_FINISHED_WITHOUT_ERROR = 3,
  SPI_DRIVER_BUSY = 4,
  SPI_DRIVER_RECEIVE_TO_MANY_BYTE = 5,
  SPI_DRIVER_ON_HOLD = 6,
  SPI_DRIVER_WAIT_FOR_ANSWER = 7,
  SPI_DRIVER_WAIT_DELAY = 8
} spiStatusDriver_t;

typedef enum
{
  SPI_DRIVER_IDLE = 0,
  SPI_DRIVER_START_RECEIVE = 1,
  SPI_DRIVER_RECEIVE_COMPLETE = 2,
  SPI_DRIVER_RECEIVE_ERROR_STATE = 3
} spiDriverStateReceive_t;

typedef struct
{
  uint8_t *p_dataSendBuf;
  uint8_t *p_dataReceiveBuf;
  uint16_t counterBuf;
  uint16_t ByteNbr;
  uint16_t wordCount;
  int8_t timesFifo;
  spiStatusDriver_t status;
  spiDriverStateReceive_t startReceive;

} statusSpi_t;

typedef struct
{
  spiStatusDriver_t status;
  spiDriverStateReceive_t startReceive;

} statusSpi2_t;

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SPI_INC_SPIGENERIC_H_ */
