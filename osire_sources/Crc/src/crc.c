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

#include <Crc/inc/crc.h>
#include "cyhal.h"

/* Using  Algorithm */
#define CRC_BITLEN          (8u)
#define CRC_POLYNOMIAL      (0x2Fu)
#define CRC_LFSR_SEED       (0x00u)
#define CRC_DATA_REVERSE    (0)
#define CRC_DATA_XOR        (0)
#define CRC_REM_REVERSE     (0)
#define CRC_REM_XOR         (0x00u)

extern cyhal_crc_t crc_obj;

/* Set CRC algorithm parameters */
crc_algorithm_t bit8_2f_algo =
{
    .width = CRC_BITLEN,
    .polynomial = CRC_POLYNOMIAL,
    .lfsrInitState = CRC_LFSR_SEED,
    .dataReverse = CRC_DATA_REVERSE,
    .dataXor = CRC_DATA_XOR,
    .remReverse = CRC_REM_REVERSE,
    .remXor = CRC_REM_XOR
};

/* Hardware CRC */
uint8_t crc (uint8_t *p_buffer, uint8_t byteCount)
{
  uint32_t crc_hw = 0;

  /* Initialize the CRC algorithm parameters */
  (void)cyhal_crc_start(&crc_obj, &bit8_2f_algo);

  /* Compute CRC */
  (void)cyhal_crc_compute(&crc_obj, p_buffer, byteCount);

  /* Finish computation */
  (void)cyhal_crc_finish(&crc_obj, &crc_hw);

  return (uint8_t)crc_hw;
}

