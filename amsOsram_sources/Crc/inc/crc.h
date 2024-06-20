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

#ifndef CRC_INC_CRC_H_
#define CRC_INC_CRC_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>

/**
 * @brief CRC Calculation with Polynom 0x2F
 * This function calculates the CRC of the given bytes with given length by
 * the variables p_buffer and byteCount.
 * The polynomial 0x2F is used for this calculation and an 8-bit CRC is
 * returned.
 *
 * @param p_buffer Pointer to the array of bytes
 * @param byteCount Specifies how many bytes are used for the calculation
 *
 * @return 8-Bit CRC
 */
uint8_t crc (uint8_t *p_buffer, uint8_t byteCount);

#ifdef __cplusplus
}
#endif

#endif /* CRC_INC_CRC_H_ */
