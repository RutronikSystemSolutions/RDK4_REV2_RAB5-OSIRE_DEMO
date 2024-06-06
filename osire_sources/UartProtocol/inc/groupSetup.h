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

#ifdef __cplusplus
extern "C"
  {
#endif

#ifndef UART_PROTOCOL_INC_GROUP_SETUP_H_
#define UART_PROTOCOL_INC_GROUP_SETUP_H_

#include <UartProtocol/inc/uartProtocolHandler.h>

typedef enum
{
  UART_COMMAND_SET_UART_MODE = 0,
  UART_COMMAND_SET_COLOR_CORRECTION_MODE,
  UART_COMMAND_SET_MINIMAL_RGBI_MODE,
  UART_COMMAND_SET_RUNNING_LIGHT_MODE,
  UART_COMMAND_SET_UPDATE_EXAMPLE_MODE
} setupCommands_t;

typedef enum
{
  UART_COMMAND_RUNNING_LIGHT_COLOR = 0,
  UART_COMMAND_RUNNING_LIGHT_WHITE_DIMMING_FAST_NIGHT,
  UART_COMMAND_RUNNING_LIGHT_WHITE_DIMMING_SLOW_NIGHT,
  UART_COMMAND_RUNNING_LIGHT_WHITE_DIMMING_FAST_DAY,
  UART_COMMAND_RUNNING_LIGHT_WHITE_DIMMING_SLOW_DAY,

} setupCommandRunningLight_t;

typedef enum
{
  UART_COMMAND_UPDATE_EXAMPLE_START_DEMO_DEFAULT_COLOR = 0,
  UART_COMMAND_UPDATE_EXAMPLE_START_DEMO_CHANGE_COLOR,
  UART_COMMAND_UPDATE_EXAMPLE_CHANGE_COLOR
} setupCommandUpdateExample_t;

typedef enum
{
  CHANGE_SETUP_NO_ERROR = 0,
  CHANGE_SETUP_ERROR_WRONG_LENGHT,
  CHANGE_SETUP_COMMAND_NOT_IMPLEMENTED
} setupCommandsErrorCode_t;

setupCommandsErrorCode_t change_setup (uint8_t *p_msg, uartHeader_t hdr);

#endif /* UART_PROTOCOL_INC_GROUP_SETUP_H_ */

#ifdef __cplusplus
}
#endif
