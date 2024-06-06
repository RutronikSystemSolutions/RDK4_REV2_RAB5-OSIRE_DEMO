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

#include "Driver/BufferControl/inc/bufferControl.h"
#include "Driver/SPI/inc/spiMasterDriver.h"
#include "Driver/SPI/inc/spiMasterTimerDriver.h"
#include "HAL/CY_GPIOs/inc/pin.h"

#define SIZE_BUFFER 255 //Note: 3x200 = 600 -> 21Leds * 28Byte (for PWM) = 588
#define MAX_COUNT_MESSAGE (SIZE_BUFFER>>2) // =SIZE_BUFFER/4 -> 4Byte min. length message
#define COUNT_BUFFER 3
#define MAX_COUNT_FILL_UP 3 //needed to be again aligned to 32bit
#define MAX_SIZE_MSG 28
#define SAFETY_DISTANCE 29

static spiBuffer_t realBuffer[COUNT_BUFFER];
static uint8_t realSendBuffer[COUNT_BUFFER][SIZE_BUFFER];
static uint32_t realDelayBuffer[COUNT_BUFFER][MAX_COUNT_MESSAGE];
static bool realAnswerBuffer[COUNT_BUFFER][MAX_COUNT_MESSAGE];
static uint8_t realSendBufferMessage[COUNT_BUFFER][MAX_COUNT_MESSAGE];
static uint8_t bufferFilled[COUNT_BUFFER];
static uint8_t bufferFillActiveBuffer = 0;

static bufferStatus_t bufferStatus = BUFFER_STATUS_UNDEF;

void buffer_init (void)
{
  for (uint8_t i = 0; i < COUNT_BUFFER; i++)
    {
      realBuffer[i].MaxLengthBuffer = SIZE_BUFFER;
      realBuffer[i].countMessageStored = 0;

      realBuffer[i].p_Buffer = realSendBuffer[i];
      realBuffer[i].p_BufferFilledLength = &bufferFilled[i];
      realBuffer[i].p_MessageLengthBuffer = realSendBufferMessage[i];

      realBuffer[i].p_delayBuffer = realDelayBuffer[i];
      realBuffer[i].p_answerBuffer = realAnswerBuffer[i];
    }

  spiMasterDriverError_t err = setPointer_SPI_Driver (&realBuffer[0],
                                                      &realBuffer[1],
                                                      &realBuffer[2]);
  init_spi_timer ();

  if (err == SPI_MASTER_DRIVER_NO_ERROR)
    {
      bufferStatus = BUFFER_STATUS_OK;
    }
  else
    {
      bufferStatus = BUFFER_STATUS_ERROR;
    }
  bufferFillActiveBuffer = 0;
}

bufferStatus_t get_buffer_control_status (void)
{
  return (bufferStatus);
}

void update_buffer_control_status (void)
{
  uint8_t counter = 0;
  int16_t sizeLeft = 0;

  for (uint8_t i = 0; i < COUNT_BUFFER; i++)
    {
      sizeLeft = (realBuffer[i].MaxLengthBuffer)
          - (*realBuffer[i].p_BufferFilledLength);

      if ((*realBuffer[i].p_BufferFilledLength == 0)
          || (sizeLeft > MAX_SIZE_MSG))
        {
          counter++;
        }
    }

  if (counter != 0)
    {
      if (bufferStatus == BUFFER_STATUS_FULL)
        {
          bufferStatus = BUFFER_STATUS_OK;
        }
    }
}

void part_function_buffer_set_new_data (uint8_t *p_bufferSwitchCounter,
                                        uint8_t length)
{
  volatile uint8_t activeBuffer = 0;
  volatile spiStatusDriver_t statusMaster;
  volatile int16_t filledSize = 0;
  volatile uint8_t spiSendDataCount = 0;
  volatile int16_t sizeLeft = 0;

  //check if buffer is used now
  activeBuffer = dri_spi_m_get_active_buffer_nbr ();
  statusMaster = dri_spi_m_get_status_master ();

  if (statusMaster == SPI_DRIVER_ON_HOLD)
    {
      bufferFillActiveBuffer = activeBuffer; //only if we are on hold!
    }
  else
    {
      if ((statusMaster == SPI_DRIVER_WAIT_DELAY)
          && (*realBuffer[activeBuffer].p_BufferFilledLength == 0))
        {
          bufferFillActiveBuffer = activeBuffer;
        }
      else
        {
          bufferFillActiveBuffer = activeBuffer + 1; //we need the next one....
          *p_bufferSwitchCounter = *p_bufferSwitchCounter + 1;
          if (bufferFillActiveBuffer >= 3)
            {
              bufferFillActiveBuffer = 0;
            }
          spiSendDataCount = dri_spi_m_get_send_count_spi_driver ();

          filledSize = *realBuffer[activeBuffer].p_BufferFilledLength
              - SAFETY_DISTANCE;

          if ((spiSendDataCount > filledSize))
            {

              if (*realBuffer[bufferFillActiveBuffer].p_BufferFilledLength != 0) //if we are 0 then we are empty and can store here!
                {
                  bufferFillActiveBuffer++;
                  *p_bufferSwitchCounter = *p_bufferSwitchCounter + 1;
                  if (bufferFillActiveBuffer >= 3)
                    {
                      bufferFillActiveBuffer = 0;
                    }
                }
            }
        }
    }

  //check if we still have space left:
  do
    {
      sizeLeft = (realBuffer[bufferFillActiveBuffer].MaxLengthBuffer)
          - (*realBuffer[bufferFillActiveBuffer].p_BufferFilledLength);

      if (sizeLeft < length)
        {
          bufferFillActiveBuffer++;
          *p_bufferSwitchCounter = *p_bufferSwitchCounter + 1;
          if (bufferFillActiveBuffer >= 3)
            {
              bufferFillActiveBuffer = 0;
            }
        }

    }
  while ((sizeLeft < length) && ((*p_bufferSwitchCounter) < 3));

}

bufferErrorCode_t __attribute__((optimize("O0"))) part_2_buffer_set_net_data (
    uint32_t *p_marker, uint32_t *p_markerNew, uint8_t *p_bufferSwitchCounter,
    uint8_t length)
{
  if (bufferStatus == BUFFER_STATUS_ERROR)
    {
      return (BUFFER_ERROR);
    }

  *p_marker = dri_spi_m_get_counter_marker_interrupt_change ();

  part_function_buffer_set_new_data (p_bufferSwitchCounter, length);
  //check if we have switched in Interrupt!
  *p_markerNew = dri_spi_m_get_counter_marker_interrupt_change ();

  return (BUFFER_NO_ERROR);
}

bufferErrorCode_t buffer_set_new_data (uint8_t *p_data, uint8_t length,
                                       uint8_t answerWait, uint32_t delay)
{
  uint32_t marker = 0, markerNew = 0;
  uint8_t bufferSwitchCounter = 0;
  volatile uint8_t eternalLoopCounter = 0;
  bufferErrorCode_t err = BUFFER_NO_ERROR;

  do
    {

      bufferSwitchCounter = 0; //we start again....
      eternalLoopCounter++;

      err = part_2_buffer_set_net_data (&marker, &markerNew,
                                        &bufferSwitchCounter, length);

      if (eternalLoopCounter > 100)
        {
          err = BUFFER_STUCK_IN_LOOP;
          bufferStatus = BUFFER_STATUS_ERROR;
          break;
        }

    }
  while ((marker != markerNew) && (bufferSwitchCounter < 3));

  if (bufferSwitchCounter >= 3)
    {
      err = BUFFER_FULL;
      bufferStatus = BUFFER_STATUS_FULL;
    }

  //in Case something went wrong end here!
  if (err != BUFFER_NO_ERROR)
    {
      return (err);
    }
  else
    {
      bufferStatus = BUFFER_STATUS_OK;
    }

  buffer_fill_up (p_data, length, answerWait, delay);

  return (err);
}

void copy_data (uint8_t *p_bufferDataOld, uint8_t *p_bufferDataNew,
                uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
    {
      *(p_bufferDataNew) = *(p_bufferDataOld + i); //copy data
      p_bufferDataNew++;
    }
}

void __attribute__((optimize("O0"))) buffer_fill_up_end (uint8_t length)
{

  uint8_t filledStatus =
      *realBuffer[bufferFillActiveBuffer].p_BufferFilledLength + length; //Add to Length
  *realBuffer[bufferFillActiveBuffer].p_BufferFilledLength = filledStatus; //this needs to be the last line!!!!
}

void buffer_fill_up (uint8_t *p_data, uint8_t length, bool answerWait,
                     uint32_t delay) //Note: delay is 32�s longer because of setup and interrupt time!
{
  uint8_t *bufferPosition = realBuffer[bufferFillActiveBuffer].p_Buffer
      + *realBuffer[bufferFillActiveBuffer].p_BufferFilledLength;
  uint8_t realLengthMsg = length;

  copy_data (p_data, bufferPosition, length); //for optimization purpose...
  bufferPosition += length;

  //check if we are aligned to 32bit!
  volatile uint8_t diff;
  volatile uint8_t LengthCorrected = length >> 2;
  LengthCorrected = LengthCorrected << 2;
  diff = length - LengthCorrected;

  if (diff != 0)
    {
      //add 0xFF to End to align!
      length = length + 4 - diff;

      for (int8_t i = (4 - diff); i > 0; i--)
        {
          *(bufferPosition) = 0xFF;
          bufferPosition++;
        }
    }

  uint8_t *p_length = realBuffer[bufferFillActiveBuffer].p_MessageLengthBuffer
      + realBuffer[bufferFillActiveBuffer].countMessageStored;
  *p_length = realLengthMsg;

  uint32_t *p_delay = realBuffer[bufferFillActiveBuffer].p_delayBuffer
      + realBuffer[bufferFillActiveBuffer].countMessageStored;

  *p_delay = delay;

  bool *p_answer = realBuffer[bufferFillActiveBuffer].p_answerBuffer
      + realBuffer[bufferFillActiveBuffer].countMessageStored;

  *p_answer = answerWait;

  realBuffer[bufferFillActiveBuffer].countMessageStored++;

  buffer_fill_up_end (length);
}
