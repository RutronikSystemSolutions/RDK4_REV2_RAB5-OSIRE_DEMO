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

#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>
#include <Crc/inc/crc.h>
#include <Osp/inc/ospCmdBuffer.h>
#include <string.h>

#define MAX_CMD_BUFFER_SIZE 16
static uint8_t cmdBuffer[MAX_CMD_BUFFER_SIZE];

/*****************************************************************************/
/*****************************************************************************/

enum OSP_ERROR_CODE osp_cmd_buffer (ospCmdBuffer_t *p_cmdInfo)
{
  memset (cmdBuffer, 0, MAX_CMD_BUFFER_SIZE);

  switch (p_cmdInfo->inCmdId)
    {
    /*****************************************************************************/
  	  // for genericDevice.c
    /*****************************************************************************/
    case (OSP_INIT_BIDIR): //error Parameter Handling
      {
        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress != 1)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_INIT_MSG);

        cmdBuffer[LENGTH_INIT_MSG - 1] = crc (cmdBuffer,
        LENGTH_INIT_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_INIT_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_INIT_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected
        break;
      }

    case (OSP_INIT_LOOP):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress != 1)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_INIT_MSG);

        cmdBuffer[LENGTH_INIT_MSG - 1] = crc (cmdBuffer,
        LENGTH_INIT_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_INIT_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_INIT_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_RESET):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_RESET_MSG);

        cmdBuffer[LENGTH_RESET_MSG - 1] = crc (cmdBuffer,
        LENGTH_RESET_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_RESET_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_GO_ACTIVE):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_GO_ACTIVE_MSG);

        cmdBuffer[LENGTH_GO_ACTIVE_MSG - 1] = crc (cmdBuffer,
        LENGTH_GO_ACTIVE_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_GO_ACTIVE_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_GO_SLEEP):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_GO_SLEEP_MSG);

        cmdBuffer[LENGTH_GO_SLEEP_MSG - 1] = crc (cmdBuffer,
        LENGTH_GO_SLEEP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_GO_SLEEP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_GO_DEEP_SLEEP):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_GO_DEEP_SLEEP_MSG);

        cmdBuffer[LENGTH_GO_DEEP_SLEEP_MSG - 1] = crc (cmdBuffer,
        LENGTH_GO_DEEP_SLEEP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_GO_DEEP_SLEEP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_CLR_ERROR):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_CLR_ERROR_MSG);

        cmdBuffer[LENGTH_CLR_ERROR_MSG - 1] = crc (cmdBuffer,
        LENGTH_CLR_ERROR_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_CLR_ERROR_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    /*****************************************************************************/
// for osireDevice.c
    /*****************************************************************************/

    case (OSP_OSIRE_SET_SETUP):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_SET_SETUP_MSG);

        osireSetSetupData_t *p_data;
        p_data = (osireSetSetupData_t*) p_cmdInfo->p_inParameter;
        cmdBuffer[3] = p_data->data.setupData;

        cmdBuffer[LENGTH_SET_SETUP_MSG - 1] = crc (cmdBuffer,
        LENGTH_SET_SETUP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_SET_SETUP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_OSIRE_SET_SETUP_SR):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_SET_SETUP_MSG);

        osireSetSetupData_t *p_data;
        p_data = (osireSetSetupData_t*) p_cmdInfo->p_inParameter;
        cmdBuffer[3] = p_data->data.setupData;

        cmdBuffer[LENGTH_SET_SETUP_MSG - 1] = crc (cmdBuffer,
        LENGTH_SET_SETUP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_SET_SETUP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_SET_PWM):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_SET_PWM_MSG);

        osirePwmData_t *p_data;
        p_data = (osirePwmData_t*) p_cmdInfo->p_inParameter;
        cmdBuffer[8] = p_data->data.pwmData[0];
        cmdBuffer[7] = p_data->data.pwmData[1];
        cmdBuffer[6] = p_data->data.pwmData[2];
        cmdBuffer[5] = p_data->data.pwmData[3];
        cmdBuffer[4] = p_data->data.pwmData[4];
        cmdBuffer[3] = p_data->data.pwmData[5];

        cmdBuffer[LENGTH_SET_PWM_MSG - 1] = crc (cmdBuffer,
        LENGTH_SET_PWM_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_SET_PWM_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_OSIRE_SET_PWM_SR):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_SET_PWM_MSG);

        osirePwmData_t *p_data;
        p_data = (osirePwmData_t*) p_cmdInfo->p_inParameter;
        cmdBuffer[8] = p_data->data.pwmData[0];
        cmdBuffer[7] = p_data->data.pwmData[1];
        cmdBuffer[6] = p_data->data.pwmData[2];
        cmdBuffer[5] = p_data->data.pwmData[3];
        cmdBuffer[4] = p_data->data.pwmData[4];
        cmdBuffer[3] = p_data->data.pwmData[5];

        cmdBuffer[LENGTH_SET_PWM_MSG - 1] = crc (cmdBuffer,
        LENGTH_SET_PWM_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_SET_PWM_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_PWM):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_PWM_MSG);

        cmdBuffer[LENGTH_READ_PWM_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_PWM_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_PWM_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_PWM_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_OTP):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_OTP_MSG);

        //Offset in OTP memory
        cmdBuffer[3] = (*((uint8_t*) p_cmdInfo->p_inParameter));

        cmdBuffer[LENGTH_READ_OTP_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_OTP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_OTP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_OTP_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_LED_STATUS):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_LEDSTATUS_MSG);

        cmdBuffer[LENGTH_READ_LEDSTATUS_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_LEDSTATUS_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_LEDSTATUS_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_LEDSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_TEMP_STATUS):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_TEMPSTATUS_MSG);

        cmdBuffer[LENGTH_READ_TEMPSTATUS_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_TEMPSTATUS_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_TEMPSTATUS_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_TEMP):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_TEMP_MSG);

        cmdBuffer[LENGTH_READ_TEMP_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_TEMP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_TEMP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMP_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_SET_OTTH):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_SET_OTTH_MSG);

        osireOtthData_t *p_data;
        p_data = (osireOtthData_t*) p_cmdInfo->p_inParameter;
        cmdBuffer[5] = p_data->data.otthData[0];
        cmdBuffer[4] = p_data->data.otthData[1];
        cmdBuffer[3] = p_data->data.otthData[2];

        cmdBuffer[LENGTH_SET_OTTH_MSG - 1] = crc (cmdBuffer,
        LENGTH_SET_OTTH_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_SET_OTTH_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_NO_OSP_RSP; // no response expected
        p_cmdInfo->outResponseMsg = NO_OSP_RSP; // no response expected

        break;
      }

    case (OSP_OSIRE_SET_OTTH_SR):
      {

        if (p_cmdInfo->p_inParameter == NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_SET_OTTH_MSG);

        osireOtthData_t *p_data;
        p_data = (osireOtthData_t*) p_cmdInfo->p_inParameter;
        cmdBuffer[5] = p_data->data.otthData[0];
        cmdBuffer[4] = p_data->data.otthData[1];
        cmdBuffer[3] = p_data->data.otthData[2];

        cmdBuffer[LENGTH_SET_OTTH_MSG - 1] = crc (cmdBuffer,
        LENGTH_SET_OTTH_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_SET_OTTH_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_OTTH):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_OTTH_MSG);

        cmdBuffer[LENGTH_READ_OTTH_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_OTTH_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_OTTH_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_OTTH_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_GO_ACTIVE_SR):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS)
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_GO_ACTIVE_MSG);

        cmdBuffer[LENGTH_GO_ACTIVE_MSG - 1] = crc (cmdBuffer,
        LENGTH_GO_ACTIVE_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_GO_ACTIVE_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_GO_SLEEP_SR):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_GO_SLEEP_MSG);

        cmdBuffer[LENGTH_GO_SLEEP_MSG - 1] = crc (cmdBuffer,
        LENGTH_GO_SLEEP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_GO_SLEEP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_GO_DEEP_SLEEP_SR):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_GO_DEEP_SLEEP_MSG);

        cmdBuffer[LENGTH_GO_DEEP_SLEEP_MSG - 1] = crc (cmdBuffer,
        LENGTH_GO_DEEP_SLEEP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_GO_DEEP_SLEEP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_CLR_ERROR_SR):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_CLR_ERROR_MSG);

        cmdBuffer[LENGTH_CLR_ERROR_MSG - 1] = crc (cmdBuffer,
        LENGTH_CLR_ERROR_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_CLR_ERROR_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_P4ERROR_BIDIR):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_P4ERROR_MSG);

        cmdBuffer[LENGTH_P4ERROR_MSG - 1] = crc (cmdBuffer,
        LENGTH_P4ERROR_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_P4ERROR_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_P4ERROR_LOOP):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_P4ERROR_MSG);

        cmdBuffer[LENGTH_P4ERROR_MSG - 1] = crc (cmdBuffer,
        LENGTH_P4ERROR_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_P4ERROR_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_TEMPSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_SETUP):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_SETUP_MSG);

        cmdBuffer[LENGTH_READ_SETUP_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_SETUP_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_SETUP_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_SETUP_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_COM_STATUS):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_COMSTATUS_MSG);

        cmdBuffer[LENGTH_READ_COMSTATUS_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_COMSTATUS_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_COMSTATUS_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_COMSTATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    case (OSP_OSIRE_READ_STATUS):
      {

        if (p_cmdInfo->p_inParameter != NULL)
          {
            return OSP_ERROR_PARAMETER;
          }
        if ((p_cmdInfo->inDeviceAddress == BROADCAST_ADDRESS)
            || (p_cmdInfo->inDeviceAddress > MAXIMUM_ADDRESS))
          {
            return OSP_ADDRESS_ERROR;
          }

        build_header (cmdBuffer, p_cmdInfo->inDeviceAddress, p_cmdInfo->inCmdId,
        LENGTH_READ_STATUS_MSG);

        cmdBuffer[LENGTH_READ_STATUS_MSG - 1] = crc (cmdBuffer,
        LENGTH_READ_STATUS_MSG - 1);

        p_cmdInfo->outCmdBufferLength = LENGTH_READ_STATUS_MSG;
        p_cmdInfo->p_outCmdBuffer = (uint8_t*) cmdBuffer;
        p_cmdInfo->outResponseLength = LENGTH_READ_STATUS_RSP; // response expected
        p_cmdInfo->outResponseMsg = OSP_RSP; // response expected

        break;
      }

    default:
      {
        return OSP_ERROR_NOT_IMPLEMENTED;
      }

    }

  return OSP_NO_ERROR;
}
