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

#ifndef UARTPROTOCOL_INC_PASSTHROUGH_H_
#define UARTPROTOCOL_INC_PASSTHROUGH_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <UartProtocol/inc/uartProtocolHandler.h>

/**
 * @brief Passthrough UART Message to microcontroller
 *
 * The received UART message contains the information whether the microcontroller expects an
 * answer from the LED. A response from the LED is output directly UART
 *
 * @param p_msg, Pointer to respose message via UART
 * @param hdr, Passes the header information of the received UART message
 */

void passthrough (uint8_t *p_msg, uartHeader_t hdr);

#ifdef __cplusplus
}
#endif

#endif /* UARTPROTOCOL_INC_PASSTHROUGH_H_ */

