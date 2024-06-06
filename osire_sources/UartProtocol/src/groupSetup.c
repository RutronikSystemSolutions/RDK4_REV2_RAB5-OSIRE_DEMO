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

#include <Demos/DemoControl/inc/demoControl.h>
#include <stdint.h>
#include <UartProtocol/inc/groupSetup.h>

#define OFFSET_COMMAND_AFTER_HEADER 3
#define MAX_ALLOWED_LENGHT_COMMAND_CHANGE_SETUP 8
#define UPDATE_EXAMPLE_VALUE_CODING 10000
#define OFFSET_COMMAND_MODI 1

setupCommandsErrorCode_t change_setup (uint8_t *p_msg, uartHeader_t hdr)
{
  setupCommands_t command = *(p_msg + OFFSET_COMMAND_AFTER_HEADER);
  setupCommandsErrorCode_t error = CHANGE_SETUP_NO_ERROR;

  if (hdr.bit.length > MAX_ALLOWED_LENGHT_COMMAND_CHANGE_SETUP)
    {
      error = CHANGE_SETUP_ERROR_WRONG_LENGHT;
    }
  else
    {
      switch (command)
        {
        case UART_COMMAND_SET_UART_MODE:
          start_uart_mode ();
          break;
        case UART_COMMAND_SET_COLOR_CORRECTION_MODE:
          start_colorCorrection_mode ();
          break;
        case UART_COMMAND_SET_MINIMAL_RGBI_MODE:
          start_minimal_rbgi_mode ();
          break;
        case UART_COMMAND_SET_RUNNING_LIGHT_MODE:
          {
            setupCommandRunningLight_t commandColor = *(p_msg
                + OFFSET_COMMAND_AFTER_HEADER + OFFSET_COMMAND_MODI);
            bool errorMode = start_runningLight_mode (commandColor);

            if (errorMode == true)
              {
                error = CHANGE_SETUP_COMMAND_NOT_IMPLEMENTED;
              }
            break;
          }
        case UART_COMMAND_SET_UPDATE_EXAMPLE_MODE:
          {
        	  break;
          }
          break;
        default:
          error = CHANGE_SETUP_COMMAND_NOT_IMPLEMENTED;
          break;
        }
    }
  return (error);
}

