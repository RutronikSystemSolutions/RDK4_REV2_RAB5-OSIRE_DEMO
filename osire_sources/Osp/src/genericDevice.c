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
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/ospCmdBuffer.h>
#include <string.h>
#include <Hal/CY_SPI/inc/spiGeneral.h>

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_init_bidir (uint16_t deviceAddress, ospInitRsp_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_INIT_RSP]; // response buffer *A.Heder: for SPI Slave Buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  // clear response buffer
  memset (rspBuffer, 0, LENGTH_INIT_RSP);

  //prepare message
  ospCmd.inCmdId = OSP_INIT_BIDIR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  //put message in Buffer
  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

    spiError = send_and_receive_data_over_spi_blocking(ospCmd.p_outCmdBuffer, rspBuffer, ospCmd.outCmdBufferLength, ospCmd.outResponseLength);

  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  if (crc (rspBuffer, LENGTH_INIT_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.bit.temp = rspBuffer[3];
  p_rsp->data.bit.status = rspBuffer[4];
  p_rsp->data.bit.address = ((rspBuffer[0] & 0x0F) << 6)
      | ((rspBuffer[1] >> 2) & 0x3F);

  p_rsp->address = p_rsp->data.bit.address; // return address for all cmds with rsp
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_init_loop (uint16_t deviceAddress, ospInitRsp_t *p_rsp)
{
  uint8_t rspBuffer[LENGTH_INIT_RSP]; // response buffer
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  memset (rspBuffer, 0, LENGTH_INIT_RSP);

  ospCmd.inCmdId = OSP_INIT_LOOP;
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

  if (crc (rspBuffer, LENGTH_INIT_RSP) != 0)
    {
      return OSP_ERROR_CRC;
    }

  p_rsp->data.bit.temp = rspBuffer[3];
  p_rsp->data.bit.status = rspBuffer[4];
  p_rsp->data.bit.address = ((rspBuffer[0] & 0x0F) << 6)
      | ((rspBuffer[1] >> 2) & 0x3F);

  p_rsp->address = p_rsp->data.bit.address; // return address for all cmds with rsp
  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
/**
 * @fn enum OSP_ERROR_CODE osp_reset(uint16_t)
 * @brief
 * OSP_RESET command Performs a complete reset of one or all devices. The effect is identical to
 * a power cycle.All register values are set to their default values, all error flags are cleared,
 * the communication mode detection is restarted, LED drivers are turned off, and the address is
 * set to 0x3ff. The device enters the UNINITIALIZED mode.
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 0..1000 RGBi device address (0: broadcast)
 * @return
 */
enum OSP_ERROR_CODE osp_reset (uint16_t deviceAddress)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_RESET;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

  ospErrorCode = osp_cmd_buffer (&ospCmd);
  if (ospErrorCode != OSP_NO_ERROR)
    {
      return ospErrorCode;
    }

  spiError = send_data_over_spi_blocking (ospCmd.p_outCmdBuffer,ospCmd.outCmdBufferLength);
// spiError = send_and_receive_data_over_spi_blocking (ospCmd.p_outCmdBuffer,uint8_t *p_bufferReceive,ospCmd.outCmdBufferLength,uint16_t countReceive);
  if (spiError != NO_ERROR_SPI)
    {
      return OSP_ERROR_SPI;
    }

  return OSP_NO_ERROR;
}

/*****************************************************************************/
/*****************************************************************************/
enum OSP_ERROR_CODE osp_go_active (uint16_t deviceAddress)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_GO_ACTIVE;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

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
enum OSP_ERROR_CODE osp_go_sleep (uint16_t deviceAddress)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_GO_SLEEP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

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
enum OSP_ERROR_CODE osp_go_deep_sleep (uint16_t deviceAddress)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_GO_DEEP_SLEEP;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

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
enum OSP_ERROR_CODE osp_osire_clr_error (uint16_t deviceAddress)
{
  ospCmdBuffer_t ospCmd;
  enum OSP_ERROR_CODE ospErrorCode;
  errorSpi_t spiError;

  ospCmd.inCmdId = OSP_CLR_ERROR;
  ospCmd.inDeviceAddress = deviceAddress;
  ospCmd.p_inParameter = NULL;

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
void build_header (uint8_t *p_msg, uint16_t deviceAddress, uint8_t command,
                   uint8_t lengthMsg)
{
  ospHeader_t hdr;

  hdr.bit.preample = OSP_PROTOCOL_PREAMPLE;
  hdr.bit.address = deviceAddress;

  if (lengthMsg == 12)
    {
      hdr.bit.psi = 7;
    }
  else
    {
      hdr.bit.psi = lengthMsg - 4;
    }
  hdr.bit.command = command;

  for (uint8_t i = 0; i < 3; i++)
    {
      *p_msg = hdr.buf[3 - i];
      p_msg++;
    }
}
