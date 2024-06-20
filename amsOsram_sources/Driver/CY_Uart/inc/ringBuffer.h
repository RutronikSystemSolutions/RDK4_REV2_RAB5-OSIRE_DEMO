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

#ifndef DRIVER_UART_INC_RINGBUFFER_H_
#define DRIVER_UART_INC_RINGBUFFER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
  uint8_t *p_buffer;
  uint16_t writeIndex;
  uint16_t readIndex;
  uint16_t bufferSize;
} ringBuffer_t;

/**
 * Add elements to the Ring Buffer.
 *
 * @param p_handler Ring buffer handler.
 * @param p_srcBuf Source of the elements.
 * @param length Length of the elements in bytes.
 * @return True if the elements are added to buffer, false otherwise.
 */
bool add_to_ring_buffer (ringBuffer_t *p_handler, const uint8_t *p_srcBuf,
                         uint16_t length);

/**
 * Gets elements from the Ring Buffer.
 *
 * @param p_handler Ring buffer handler.
 * @param p_dstBuf Destination of the get elements.
 * @param length Length of the elements in bytes.
 * @return True if the elements are retrived from buffer, false otherwise.
 */
bool get_from_ring_buffer (ringBuffer_t *p_handler, uint8_t *p_dstBuf,
                           uint16_t length);

/**
 * Getter for the fill level of the Ring Buffer.
 *
 * @param p_handler Ring buffer handler.
 * @return Number of elements in the buffer (in bytes).
 */
uint16_t get_ring_buffer_fill_level (ringBuffer_t *p_handler);

/**
 * Reset Ring Buffer, set write and read index to 0, and clear buffer.
 *
 * @param p_handler Ring buffer handler.
 * @return True if the Ring Buffer is reset, false otherwise.
 */
bool reset_ring_buffer (ringBuffer_t *p_handler);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_UART_INC_RINGBUFFER_H_ */
