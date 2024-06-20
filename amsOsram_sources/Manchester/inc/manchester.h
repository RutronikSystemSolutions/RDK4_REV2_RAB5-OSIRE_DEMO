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

#ifndef MANCHESTER_INC_MANCHESTER_H_
#define MANCHESTER_INC_MANCHESTER_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include "stdint.h"


typedef enum
{
  NO_ERROR = 0, ERROR_SIZE, ERROR_NULL_POINTER
} errorCodeManchester_t;

/**
 * @brief Manchester Code encoder
 * This function codes the bytes given by p_buffer to Manchester code and stores the new bytes in the buffer p_bufferNew.
 * The buffer (p_bufferNew) must be twice as large as the number of bytes given by byteCount.
 *
 * @param p_buffer Pointer to an array where the bytes of the message are stored
 * @param p_bufferNew Pointer to an array where new bytes will be saved
 * @param byteCount Specifies how many bytes form p_buffer will be encoded
 * @param sizeBufferNew to check if the size of the buffer is at least twice byteCount
 * @return Error code if the size check was successful or not
 *
 */

errorCodeManchester_t manchester_encoding_buffer_swap (uint8_t *p_buffer,
                                                       uint8_t *p_bufferNew,
                                                       uint8_t byteCount,
                                                       uint8_t sizeBufferNew);


#ifdef __cplusplus
}
#endif

#endif /* MANCHESTER_INC_MANCHESTER_H_ */
