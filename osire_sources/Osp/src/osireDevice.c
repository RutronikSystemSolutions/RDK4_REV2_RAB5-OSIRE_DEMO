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

/**
 * Include OSP Led Definitions and Data structures
 * 
 */
//#noch keine Antwort-IDs implementiert
#include <Crc/inc/crc.h>
#include <Hal/CY_SPI/inc/spiGeneral.h>
#include <Osp/inc/osireDevice.h>
#include <Osp/inc/ospCmdBuffer.h>
#include <string.h>

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_set_setup (uint16_t deviceAddress,
                                         osireSetSetupData_t data)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_OSIRE_SET_SETUP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.setupData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                          ospCmd.outCmdBufferLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_set_setup_and_sr (uint16_t deviceAddress,
                                                osireSetSetupData_t data,
                                                osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_SET_SETUP_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.setupData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }
  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_set_pwm (uint16_t deviceAddress,
                                       osirePwmData_t data)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_OSIRE_SET_PWM;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.pwmData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                          ospCmd.outCmdBufferLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/

enum OSP_ERROR_CODE osp_osire_set_pwm_and_sr (uint16_t deviceAddress,
                                              osirePwmData_t data,
                                              osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_SET_PWM_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.pwmData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_pwm (uint16_t deviceAddress,
                                        osirePwmData_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_PWM_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_PWM_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_PWM;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_PWM_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 6; i++)
    {
      p_rsp->data.pwmData[i] = rspBuffer[8 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_read_otp (uint16_t deviceAddress, uint8_t otpAddress,
                                  osireOtpData_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_OTP_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_OTP_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_OTP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &otpAddress;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_OTP_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 8; i++)
    {
      p_rsp->data.byte[i] = rspBuffer[10 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_otp_complete (uint16_t deviceAddress,
                                                 osireOtpDataComplete_t *p_rsp)
{
  osireOtpData_t opt; // OTP buffer

  for (uint8_t i = 0; i < 0x1F; i = i + 8)
    {
      enum OSP_ERROR_CODE errorCode = osp_read_otp (deviceAddress, i, &opt);

      if (errorCode != OSP_NO_ERROR)
        {
          return errorCode;
        }

      for (uint8_t j = 0; j < 8; j++)
        {
          p_rsp->data.byte[i + j] = opt.data.byte[j];
        }
    }

  p_rsp->address = opt.address;
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_ledstatus (uint16_t deviceAddress,
                                              osireLedStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_LEDSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_LEDSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_LED_STATUS;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_LEDSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.ledStatus = rspBuffer[3];

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_tempstatus (uint16_t deviceAddress,
                                               osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_TEMP_STATUS;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_temp (uint16_t deviceAddress,
                                         osireTemp_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMP_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMP_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_TEMP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMP_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.temp_value = rspBuffer[FIRST_BYTE_PAYLOAD];

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_set_otth (uint16_t deviceAddress,
                                        osireOtthData_t data)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_OSIRE_SET_OTTH;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.otthData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                          ospCmd.outCmdBufferLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_set_otth_and_sr (uint16_t deviceAddress,
                                               osireOtthData_t data,
                                               osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_SET_OTTH_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = &data.data.otthData;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_otth (uint16_t deviceAddress,
                                         osireOtthData_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_OTTH_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_OTTH_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_OTTH;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_OTTH_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 3; i++)
    {
      p_rsp->data.otthData[i] = rspBuffer[5 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_go_active_and_sr (uint16_t deviceAddress,
                                          osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_OSIRE_GO_ACTIVE_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_go_sleep_and_sr (uint16_t deviceAddress,
                                               osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_GO_SLEEP_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_go_deep_sleep_and_sr (uint16_t deviceAddress,
                                                    osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_GO_DEEP_SLEEP_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_clr_error_and_sr (uint16_t deviceAddress,
                                                osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // message buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_CLR_ERROR_SR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_p4error_bidir (uint16_t deviceAddress,
                                             osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // message buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_P4ERROR_BIDIR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}
/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_p4error_loop (uint16_t deviceAddress,
                                            osireTempStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_TEMPSTATUS_RSP]; // message buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_TEMPSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_P4ERROR_LOOP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_TEMPSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  for (uint8_t i = 0; i < 2; i++)
    {
      p_rsp->data.tempStatus[i] = rspBuffer[4 - i];
    }

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}
/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_setup (uint16_t deviceAddress,
                                          osireSetSetupData_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_SETUP_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_SETUP_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_SETUP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_SETUP_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.setupData = rspBuffer[FIRST_BYTE_PAYLOAD];

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_comstatus (uint16_t deviceAddress,
                                              osireComStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_COMSTATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_COMSTATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_COM_STATUS;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_COMSTATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.comStatus = rspBuffer[FIRST_BYTE_PAYLOAD];

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_osire_read_status (uint16_t deviceAddress,
                                           osireStatus_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_READ_STATUS_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_READ_STATUS_RSP);

  ospCmd.inCmdId = OSP_OSIRE_READ_STATUS;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,
                                                      rspBuffer,
                                                      ospCmd.outCmdBufferLength,
                                                      ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_READ_STATUS_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.status = rspBuffer[FIRST_BYTE_PAYLOAD];

  p_rsp->address = ((rspBuffer[0] & 0x0F) << 6) | ((rspBuffer[1] >> 2) & 0x3F);
  return OSP_NO_ERROR;
}

