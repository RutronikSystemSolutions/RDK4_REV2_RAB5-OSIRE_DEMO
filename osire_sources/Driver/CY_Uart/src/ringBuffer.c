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

#include <Driver/CY_Uart/inc/ringBuffer.h>

bool add_to_ring_buffer (ringBuffer_t *p_handler, const uint8_t *p_srcBuf,uint16_t length)
{
  if ((p_handler == NULL) || (p_handler->p_buffer == NULL))
    {
      // Invalid handler.
      return false;
    }
  if ((p_handler->bufferSize - get_ring_buffer_fill_level (p_handler)) < length)
    {
      // Not enough space in the buffer.
      return false;
    }

  uint16_t spaceTillEdge = p_handler->bufferSize - p_handler->writeIndex;

  if (spaceTillEdge < length)
    {
      memcpy ((p_handler->p_buffer + p_handler->writeIndex), p_srcBuf,
              spaceTillEdge);
      memcpy (p_handler->p_buffer, (p_srcBuf + spaceTillEdge),
              (length - spaceTillEdge));
      p_handler->writeIndex = length - spaceTillEdge;
    }
  else
    {
      memcpy ((p_handler->p_buffer + p_handler->writeIndex), p_srcBuf, length);
      p_handler->writeIndex += length;
    }
  return true;
}

bool get_from_ring_buffer (ringBuffer_t *p_handler, uint8_t *p_dstBuf,uint16_t length)
{
  if ((p_handler == NULL) || (p_handler->p_buffer == NULL))
    {
      // Invalid handler.
      return false;
    }
  if (get_ring_buffer_fill_level (p_handler) < length)
    {
      // Not enough data in the buffer.
      return false;
    }

  uint16_t spaceTillEdge = p_handler->bufferSize - p_handler->readIndex;

  if (spaceTillEdge < length)
    {
      memcpy (p_dstBuf, (p_handler->p_buffer + p_handler->readIndex),
              spaceTillEdge);
      memcpy ((p_dstBuf + spaceTillEdge), p_handler->p_buffer,
              (length - spaceTillEdge));
      p_handler->readIndex = length - spaceTillEdge;
    }
  else
    {
      memcpy (p_dstBuf, (p_handler->p_buffer + p_handler->readIndex), length);
      p_handler->readIndex += length;
    }
  return true;
}

uint16_t get_ring_buffer_fill_level (ringBuffer_t *p_handler)
{
  if ((p_handler == NULL) || (p_handler->p_buffer == NULL))
    {
      // Invalid handler.
      return 0;
    }
  uint16_t bufferFillLevel = 0;
  if (p_handler->writeIndex > p_handler->readIndex)
    {
      bufferFillLevel = p_handler->writeIndex - p_handler->readIndex;
    }
  else if (p_handler->readIndex > p_handler->writeIndex)
    {
      // Write went over the size of the buffer so we need to take that into account.
      bufferFillLevel = (p_handler->bufferSize - p_handler->readIndex)
          + p_handler->writeIndex;
    }
  return bufferFillLevel;
}

bool reset_ring_buffer (ringBuffer_t *p_handler)
{
  if ((p_handler == NULL) || (p_handler->p_buffer == NULL))
    {
      // Invalid handler.
      return false;
    }
  p_handler->readIndex = 0;
  p_handler->writeIndex = 0;
  memset (p_handler->p_buffer, 0, p_handler->bufferSize);
  return true;
}
