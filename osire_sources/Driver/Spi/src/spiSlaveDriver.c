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

#include <Driver/Spi/inc/spiSlaveDriver.h>
#include <Hal/CY_Gpios/inc/pin.h>
#include <SwTimer/inc/swTimer.h>

/*****************************************************************************/
/*****************************************************************************/
//static volatile statusSpi_t statusSlaveDriver;

//static volatile uint32_t dummyRead = 0;
static uint8_t bufferMessage[SIZE_OF_SPI_RECEIVE_BUFFER];
static uint32_t bufferMessageTimeStamp[SIZE_OF_SPI_RECEIVE_BUFFER];
static volatile uint8_t bufferCount = 0;
static volatile uint8_t endMarkerPosition = 0;
//static volatile uint32_t *p_timer;

/*****************************************************************************/
/*****************************************************************************/
void dri_spi_s_set_tcr (tcr_t *p_tcrSlave)
{
  p_tcrSlave->bit.clkPol = 1; // 0: SCK inactive = low, 1: SCK inactive = high;
  //0: Data is captured on the leading edge of SCK and changed on the following edge of SCK,
  //1:Data is changed on the leading edge of SCK and captured on the following edge of SCK
  p_tcrSlave->bit.clkPhase = 1; // see above
  p_tcrSlave->bit.prescal = 0; //Divide by 1;
  p_tcrSlave->bit.reserved = 0;
  p_tcrSlave->bit.csn = 2; // LPSPI_PCS[2] (PTE13)
  p_tcrSlave->bit.lsbFirst = 0; //0 = MSB first, 1 = LSB first
  p_tcrSlave->bit.byteSwap = 0; //byte swap disabled
  p_tcrSlave->bit.contTrans = 0; //Continuous transfer is disabled
  p_tcrSlave->bit.contCommand = 0; //command word for start of new transfer
  p_tcrSlave->bit.receiveMask = 0; //0 = normal transfer
  p_tcrSlave->bit.transmitMask = 0; //0 = normal transfer
  p_tcrSlave->bit.transWith = 0; //0 = 1bit transfer (no parallel transfer)
  p_tcrSlave->bit.reservedFillWithZero = 0;
  p_tcrSlave->bit.frameSize = 1 * 8 - 1; //7; //0b111 = FRAMZESZ+1 -> 8bit
}

/*****************************************************************************/
/*****************************************************************************/
void dri_spi_s_set_interrupt (spiInterruptRegister_t *p_intSlave)
{
  p_intSlave->bit.dataMatch = 0;
  p_intSlave->bit.receiveError = 1;
  p_intSlave->bit.transmitError = 0; //1;
  p_intSlave->bit.transferComplete = 0;
  p_intSlave->bit.frameComplete = 0;
  p_intSlave->bit.wordComplete = 0;
  p_intSlave->bit.receiveData = 1;
  p_intSlave->bit.transmitData = 0;
}

/*****************************************************************************/
/*****************************************************************************/
void dri_spi_s_set_configuration_register_1 (spiConfig1Register_t *p_configReg1)
{
  p_configReg1->bit.masterMode = 0; //slave mode
  p_configReg1->bit.samplePoint = 0;
  p_configReg1->bit.automaticPCS = 1; //only if TCR[CPHA]=1
  p_configReg1->bit.noStall = 0;
  p_configReg1->bit.csnPolarity = 0; //1;//0;
  p_configReg1->bit.matchConfig = 0;
  p_configReg1->bit.pinConfig = 0;
  p_configReg1->bit.outputConfig = 1;
  p_configReg1->bit.csnConfig = 0;
}

/*****************************************************************************/
/*****************************************************************************/
void dri_spi_s_init (void)
{
//  LPSPI_Type *p_lpspi = LPSPI2;
//  tcr_t slaveTcr;
//  spiConfig1Register_t configReg1;
//  spiInterruptRegister_t intSlave;
//
//  p_lpspi->CR = 0x00000302; // Disable module for configuration //Reset Receive Fifo, Reset Transmit Fifo, Software reset, module disabled
//  p_lpspi->CR = 0x00000000; // Disable module for configuration //set reset(s) back
//  p_lpspi->DER = 0x00000000;    // DMA not used
//  p_lpspi->CFGR0 = 0x00000000;    // Defaults:
//
//  dri_spi_s_set_configuration_register_1 (&configReg1);
//  p_lpspi->CFGR1 = configReg1.config1;
//
//  dri_spi_s_set_tcr (&slaveTcr);
//  p_lpspi->TCR = slaveTcr.tcr;
//
//  p_lpspi->CCR = 0x00000000; //not used in slave mode //0x02020208;    // Clock dividers
//  p_lpspi->FCR = 0x00000003;    // RXWATER=0: Rx flags set when Rx FIFO >0
//  // TXWATER=3: Tx flags set when Tx FIFO <= 3
//
//  dri_spi_s_set_interrupt (&intSlave);
//  p_lpspi->IER = intSlave.ier;    // Interrupts used
//  p_lpspi->CR = 0x00000001;    // Enable module for operation
//
//  INT_SYS_InstallHandler (LPSPI2_IRQn, dri_spi_s_receive_isr, (isr_t*) 0);
//  INT_SYS_EnableIRQ (LPSPI2_IRQn);
//  INT_SYS_SetPriority (LPSPI2_IRQn, 1);
//
//  p_timer = get_sysTick_int_pointer ();

}

/*****************************************************************************/
/*****************************************************************************/

//static volatile uint8_t test = 0;

void dri_spi_s_receive_isr (void)
{
//  LPSPI_Type *p_lpspi = LPSPI2;
//  volatile spiStatusRegister_t *p_spiStatus;
//  volatile spiFifoStatus_t *p_spiFifo;
//  volatile int8_t count = 0;
//
//  p_spiStatus = (spiStatusRegister_t*) &p_lpspi->SR;
//  p_spiFifo = (spiFifoStatus_t*) &p_lpspi->FSR;
//
//  p_spiStatus->bit.wordCompleteFlag = 1;    //clear by write
//  p_spiStatus->bit.frameCompleteFlag = 1;    //clear by write
//  p_spiStatus->bit.transferCompleteFlag = 1;    //clear by write
//
//  count = (int8_t) p_spiFifo->bit.receiveCount;
//
//  if (p_spiStatus->bit.transmitErrorFlag == 1)
//    {
//      // check status
//      p_spiStatus->bit.transmitErrorFlag = 1;    //clear by write
//      statusSlaveDriver.status = SPI_DRIVER_SEND_ERROR;
//      statusSlaveDriver.startReceive = SPI_DRIVER_RECEIVE_ERROR_STATE;
//
//    }
//  else if (p_spiStatus->bit.receiveErrorFlag == 1)
//    {
//      p_spiStatus->bit.receiveErrorFlag = 1; //clear by write
//      statusSlaveDriver.status = SPI_DRIVER_RECEIVE_ERROR;
//      statusSlaveDriver.startReceive = SPI_DRIVER_RECEIVE_ERROR_STATE;
//    }
//  else
//    {
//      while (count > 0)
//        {
//          dummyRead = p_lpspi->RDR;
//          count--;
//          bufferMessage[bufferCount] = dummyRead;
//          bufferMessageTimeStamp[bufferCount] = *p_timer;
//
//          bufferCount++;
//          endMarkerPosition++;
//
//          if (endMarkerPosition >= SIZE_OF_SPI_RECEIVE_BUFFER)
//            {
//              endMarkerPosition = 0;
//            }
//
//          bufferMessage[endMarkerPosition] = 0xBB; //set the next to 0xBB
//
//          if (bufferCount >= SIZE_OF_SPI_RECEIVE_BUFFER)
//            {
//              bufferCount = 0;
//            }
//        }
//
//      if (p_spiFifo->bit.receiveCount != 0)
//        {
//          statusSlaveDriver.startReceive = SPI_DRIVER_RECEIVE_ERROR_STATE;
//        };
//
//    }
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t* get_buffer (void)
{
  return (bufferMessage);
}

uint32_t* get_buffer_time_stamp (void)
{
  return (bufferMessageTimeStamp);
}

uint8_t get_buffer_position (void)
{
  return (bufferCount);
}

uint8_t get_max_size_buffer (void)
{
  return (SIZE_OF_SPI_RECEIVE_BUFFER);
}

void dri_spi_s_reset_buffer (void)
{
  bufferCount = 0;
  endMarkerPosition = 0;
  for (uint8_t i = 0; i < SIZE_OF_SPI_RECEIVE_BUFFER; i++)
    {
      bufferMessage[i] = 0;
      bufferMessageTimeStamp[i] = 0;
    }
}
