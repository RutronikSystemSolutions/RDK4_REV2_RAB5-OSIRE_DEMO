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

#ifndef UARTPROTOCOL_INC_CONFIGUC_H_
#define UARTPROTOCOL_INC_CONFIGUC_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <UartProtocol/inc/uartProtocolHandler.h>

enum UART_CONFIG_CMDS
{
  CONFIG_DEVICE_GET_TYPE = 0x00,
  CONFIG_DEVICE_GET_VERSION = 0x01,
  CONFIG_DEVICE_GETFW_VERSION = 0x03
};

#define LENGTH_GET_TYPE 2
#define LENGTH_GET_VERSION 3
#define LENGTH_GETFW_VERSION 4 // has to be defined

#define CONFIG_DEVICE_VERSION_MAJOR 1
#define CONFIG_DEVICE_VERSION_MINOR 0

/**
 * @brief Protocol Handler for Configuration microcontroller messages
 *
 * The requested functions will be executed and can send a response via the UART interface depending on the function
 *
 * @param p_msg, Pointer to respose message via UART
 * @param hdr, Passes the header information of the received UART message
 */

void config_uc (uint8_t *p_msg, uartHeader_t hdr);

#ifdef __cplusplus
}
#endif

#endif /* UARTPROTOCOL_INC_CONFIGUC_H_ */
