/*****************************************************************************
 * Copyright 2022 by ams OSRAM AG                                                  *
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

#include <Driver/BufferControl/inc/bufferControl.h>
#include "Driver/SPI/inc/spiMasterDriver.h"
#include "HAL/CY_GPIOs/inc/pin.h"
#include <Driver/SPI/inc/spiMasterTimerDriver.h>

//-------------------------local function declaration--------------------------

void dri_spi_m_switch_pointer_buffer_send(void);
void dri_spi_m_set_new_frame_size(uint8_t frameSizeByte);
void dri_spi_m_clear_receive(void);
void dri_spi_m_load_data_into_fifo(void);
void dri_spi_m_restart_spi(bool changeInterruptSetting);

//-------------------------------Variable Definition---------------------------
static volatile statusSpi2_t statusMasterDriver;
//static int8_t timesFifo;
static uint8_t counterLoadedData = 0;
//static uint8_t counterLoadedDataMsg = 0;
//static uint8_t *p_activeSendBuffer;
static uint8_t activeBufferNbr = 0;
//static uint8_t nbrOfMessage = 0;
//tatic spiBuffer_t *p_spiBuffer[3];
//tatic volatile uint8_t dataCorruptionMarker = 0;
static volatile uint8_t dataCorruptionLoopingCounter = 0;
//static bool messageSendCompl = false;
//static volatile uint32_t dummyRead = 0;
//static uint32_t standardValueInt = 0;
//static uint32_t standardValueTcr = 0;

//-------------------------------Init------------------------------------------
/*
 * init RGBi SPI master
 */
void dri_spi_m_init(void)
    {
//  dri_spi_m_register_init ((const isr_t) dri_spi_m_send_isr);
//  buffer_init ();
//  p_activeSendBuffer = p_spiBuffer[0]->p_Buffer;
//  activeBufferNbr = 0;
//  counterLoadedData = 0;
//  nbrOfMessage = 0;
//  statusMasterDriver.status = SPI_DRIVER_ON_HOLD;
//  spiInterruptRegister_t masterInt;
//  dri_spi_m_set_interrupt (&masterInt);
//  standardValueInt = masterInt.ier;
//  tcr_t masterTcr;
//  dri_spi_m_set_tcr (&masterTcr);
//  standardValueTcr = masterTcr.tcr;

    }
spiMasterDriverError_t setPointer_SPI_Driver(spiBuffer_t *p_buffer0, spiBuffer_t *p_buffer1, spiBuffer_t *p_buffer2)
    {
    spiMasterDriverError_t err = SPI_MASTER_DRIVER_NO_ERROR;
//
//    if ((p_buffer0 == NULL) || (p_buffer1 == NULL) || (p_buffer2 == NULL))
//	{
//	err = SPI_MASTER_DRIVER_NULL_POINTER;
//	return (err);
//	}
//
//    p_spiBuffer[0] = p_buffer0;
//    p_spiBuffer[1] = p_buffer1;
//    p_spiBuffer[2] = p_buffer2;
//
    return (err);
    }

//----------------------local function definition------------------------------

void dri_spi_m_switch_pointer_buffer_send(void)
    {
//    LPSPI_Type *p_lpspi = LPSPI0;
//
//    enum state_t
//	{
//	DEFAULT_STATE,
//	STILL_SENDING,
//	END_OF_MSG,
//	END_OF_BUFFER,
//	BUFFER_EMPTY
//	} state;
//
//    state = DEFAULT_STATE;
//    bool nextBufferEmpty = false;
//    bool delayNeeded = false;
//    uint32_t delay = 0;
//
//    //first check where we are:
//    if (counterLoadedDataMsg >= (*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage)))
//	{
//	state = END_OF_MSG;
//	}
//
//    if (counterLoadedData >= *p_spiBuffer[activeBufferNbr]->p_BufferFilledLength)
//	{
//	state = END_OF_BUFFER;
//	}
//
//    if (*(p_spiBuffer[activeBufferNbr]->p_delayBuffer + nbrOfMessage) != 0)
//	{
//	delayNeeded = true;
//	delay = *(p_spiBuffer[activeBufferNbr]->p_delayBuffer + nbrOfMessage);
//	}
//
//    switch (state)
//	{
//    case END_OF_MSG:
//
//	counterLoadedDataMsg = 0;
//	nbrOfMessage++;
//
//	//load new Data:
//	if (delayNeeded == false)
//	    {
//	    dri_spi_m_restart_spi(false);
//	    }
//	break;
//
//    case END_OF_BUFFER:
//
//	*p_spiBuffer[activeBufferNbr]->p_BufferFilledLength = 0; //reset buffer length to 0
//	p_spiBuffer[activeBufferNbr]->countMessageStored = 0;
//	dri_spi_m_clear_receive();
//	counterLoadedData = 0;
//	counterLoadedDataMsg = 0;
//	nbrOfMessage = 0;
//	activeBufferNbr++;
//	dataCorruptionMarker++;
//	dataCorruptionLoopingCounter++;
//	if (activeBufferNbr >= 3)
//	    {
//	    activeBufferNbr = 0; //wrap around
//	    }
//
//	p_activeSendBuffer = p_spiBuffer[activeBufferNbr]->p_Buffer;
//
//	if (*p_spiBuffer[activeBufferNbr]->p_BufferFilledLength == 0)
//	    {
//	    nextBufferEmpty = true;
//	    }
//	else
//	    {
//	    if (delayNeeded == false)
//		{
//		dri_spi_m_restart_spi(false);
//		}
//	    }
//	break;
//    default:
//	return;
//	break; //this line can not be reached....
//	}
//
////we are only here if we have finished a message!
//
//    if (nextBufferEmpty == true)
//	{
//	//we are on hold!
//	p_lpspi->IER = 0;
//	statusMasterDriver.status = SPI_DRIVER_ON_HOLD;
//	}
//
//    if (delayNeeded == true)
//	{
//	start_delay_timer(delay);
//	p_lpspi->IER = 0;
//	statusMasterDriver.status = SPI_DRIVER_WAIT_DELAY;
//	}

    }

void dri_spi_m_set_new_frame_size(uint8_t frameSizeByte)
    {
//    LPSPI_Type *p_lpspi = LPSPI0;
//    tcr_t masterTcr;
//    if (frameSizeByte == 0)
//	{
//	frameSizeByte = 1; //calculation later must be min. 7
//	}
//    //standardValueTcr -7  = TCR register value after init, but -7 because we remove the lower bits because this are the frameSize
//    //frame size is stored as bit value in TCR register -> frameSizeyByte*8
//    //frame size [bit] - 1 because we need to store it one bit lower because of TCR register definition
//    masterTcr.tcr = standardValueTcr - 7 + (frameSizeByte * 8 - 1);
//
//    p_lpspi->TCR = masterTcr.tcr;
    }
void dri_spi_m_clear_receive(void)
    {
//    volatile spiFifoStatus_t *p_spiFifo;
//    LPSPI_Type *p_lpspi = LPSPI0;
//    p_spiFifo = (spiFifoStatus_t*) &p_lpspi->FSR;
//
//    while (p_spiFifo->bit.receiveCount != 0) //get out the received data if there should be any... normally no byte should be here because of Match Register!
//	{
//	dummyRead = p_lpspi->RDR;
//	}
    }
void __attribute__((optimize("O3"))) dri_spi_m_load_data_into_fifo(void)
    {
//    LPSPI_Type *p_lpspi = LPSPI0;
//    volatile spiFifoStatus_t *p_spiFifo;
//    p_spiFifo = (spiFifoStatus_t*) &p_lpspi->FSR;
//    volatile uint32_t *p_temp;
//    bool dataLoadValid = true;
//
//    p_temp = (uint32_t*) p_activeSendBuffer;
//    p_temp += counterLoadedData >> 2;
//
//    statusMasterDriver.status = SPI_DRIVER_BUSY;
//
//    volatile uint8_t transCountStamp = p_spiFifo->bit.transCount;
//
//    for (uint8_t i = 0; ((i < (4 - transCountStamp)) && (timesFifo >= 0)); i++)
//	{
//	uint32_t t = *(p_temp + i);
//	//Note: the data here is "Byte-Swapped" which means LSB is now MSB etc. the hardware is also Byte-Swapping so in the end we have the order correct again
//
//	timesFifo--; // timesFifo is signed
//
//	if (timesFifo < 0)
//	    {
//	    volatile int16_t rest = ((*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage)) - counterLoadedDataMsg);
//
//	    if (rest <= 0)
//		{
//		dataLoadValid = false; //we have loaded everything!
//		}
//
//	    rest = 4 - rest; //because word (32bit) are used we need to shift
//	    t = t << (rest * 8);
//	    }
//
//	if (dataLoadValid == true)
//	    {
//	    counterLoadedDataMsg += 4;
//	    counterLoadedData += 4;
//	    p_lpspi->TDR = t;
//	    }
//
//	}
    }

void dri_spi_m_restart_spi(bool changeInterruptSetting)
    {
//    LPSPI_Type *p_lpspi = LPSPI0;
//    dri_spi_m_clear_receive();
//
//    timesFifo = (*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage)) >> 2; //times 4
//
//    dri_spi_m_set_new_frame_size((*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage)));
//    //reload interrupt
//    if (changeInterruptSetting == true)
//	{
//	p_lpspi->IER = standardValueInt; //masterInt.ier;
//	}

    }
//--------------------------global functions-----------------------------------
uint8_t dri_spi_m_get_active_buffer_nbr(void)
    {
    return (activeBufferNbr);
    }

spiCommandError_t dri_spi_m_restart_with_timer(void)
    {
    spiCommandError_t err = SPI_DRIVER_COMMAND_NO_ERROR;
//
//    if (statusMasterDriver.status == SPI_DRIVER_WAIT_DELAY) //we are on hold!
//	{
//	if ((*p_spiBuffer[activeBufferNbr]->p_BufferFilledLength != 0) && (*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage) != 0)) //buffer is not empty!
//	    {
//	    dri_spi_m_restart_spi(true);
//	    }
//	else
//	    {
//	    err = SPI_DRIVER_COMMAND_NO_RESET_NO_DATA_TO_SEND;
//	    statusMasterDriver.status = SPI_DRIVER_ON_HOLD; //we change the setting!
//	    }
//	dataCorruptionMarker++;
//	dataCorruptionLoopingCounter++;
//
//	}
//    else
//	{
//
//	err = SPI_DRIVER_COMMAND_NO_RESET;
//	}
//
    return (err);
    }

spiCommandError_t dri_spi_m_restart_data_send(void) //restart the buffer send with actual buffer
    {
    spiCommandError_t err = SPI_DRIVER_COMMAND_NO_ERROR;
//
//    if (statusMasterDriver.status == SPI_DRIVER_ON_HOLD) //we are on hold!
//	{
//	if (*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage) != 0) //we can only restart if we have new data!
//	    {
//	    dri_spi_m_restart_spi(true);
//	    }
//	else
//	    {
//	    err = SPI_DRIVER_COMMAND_NO_RESET_NO_DATA_TO_SEND;
//	    }
//	}
//    else
//	{
//
//	err = SPI_DRIVER_COMMAND_NO_RESET;
//	}
//
    return (err);
    }

spiStatusDriver_t dri_spi_m_get_status_master(void)
    {
    return statusMasterDriver.status;
    }

uint8_t dri_spi_m_get_send_count_spi_driver(void)
    {
    return (counterLoadedData);
    }

uint8_t dri_spi_m_get_counter_marker_interrupt_change(void)
    {
    return (dataCorruptionLoopingCounter);
    }

//-------------------------------Interrupt-------------------------------------
void dri_spi_m_send_isr(void)
    {
//    LPSPI_Type *p_lpspi = LPSPI0;
//    volatile spiStatusRegister_t *p_spiStatus;
//    volatile spiFifoStatus_t *p_spiFifo;
//    spiInterruptRegister_t masterInt;
//
//    p_spiStatus = (spiStatusRegister_t*) &p_lpspi->SR;
//    p_spiFifo = (spiFifoStatus_t*) &p_lpspi->FSR;
//
//    p_spiStatus->bit.wordCompleteFlag = 1; //clear by write
//    p_spiStatus->bit.frameCompleteFlag = 1; //clear by write
//    p_spiStatus->bit.transferCompleteFlag = 1; //clear by write
//
//    if (p_spiStatus->bit.transmitErrorFlag == 1)
//	{
//	p_spiStatus->bit.transmitErrorFlag = 1;    //clear by write
//	statusMasterDriver.status = SPI_DRIVER_SEND_ERROR;
//	p_lpspi->IER = 0;
//	return;
//
//	}
//
//    if (p_spiStatus->bit.receiveErrorFlag == 1)
//	{
//	p_spiStatus->bit.receiveErrorFlag = 1; //clear by write
//	statusMasterDriver.status = SPI_DRIVER_RECEIVE_ERROR;
//	p_lpspi->IER = 0;
//	return;
//	}
//
//    //we are only here if we don't have a problem...
//    if (messageSendCompl == true)
//	{
//	if (p_spiFifo->bit.transCount == 0)
//	    {
//	    messageSendCompl = false;
//	    dri_spi_m_switch_pointer_buffer_send();
//
//	    if (statusMasterDriver.status == SPI_DRIVER_BUSY)
//		{
//		dri_spi_m_load_data_into_fifo();
//
//		if (counterLoadedDataMsg >= (*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage)))
//		    {
//		    //we don't need to change the interrupt -> we are already working fine!
//		    messageSendCompl = true;
//		    }
//		else
//		    { //go back to standard Value!
//		    masterInt.ier = standardValueInt;
//		    p_lpspi->IER = masterInt.ier;
//		    }
//		}
//	    }
//	}
//    else
//	{
//	dri_spi_m_load_data_into_fifo();
//
//	if (counterLoadedDataMsg >= (*(p_spiBuffer[activeBufferNbr]->p_MessageLengthBuffer + nbrOfMessage)))
//	    {
//	    masterInt.ier = standardValueInt;
//	    masterInt.bit.transmitData = 0;
//	    masterInt.bit.frameComplete = 1; //change to "frame complete interrupt because fifo is fully loaded"
//	    p_lpspi->IER = masterInt.ier;
//
//	    messageSendCompl = true;
//	    }
//	}
    }
