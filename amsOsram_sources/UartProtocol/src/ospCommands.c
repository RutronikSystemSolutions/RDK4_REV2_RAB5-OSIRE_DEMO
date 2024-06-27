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
#include <Hal/Osire/inc/osire.h>
#include <Hal/CY_Uart/inc/uart.h>
#include <Osp/inc/genericDevice.h>
#include <Osp/inc/osireDevice.h>
#include <UartProtocol/inc/ospCommands.h>
#include <UartProtocol/inc/uartProtocolHandler.h>
#include <osp2/inc/osp2.h>

void osp_commands (uint8_t *p_msg, uartHeader_t hdr)
{
  p_msg = p_msg + 4;
  uint16_t deviceAddress = *p_msg << 8;
  p_msg++;
  deviceAddress = deviceAddress + *p_msg;
  uartStatus_t uartStatus;
  uartStatus.status = 0;
  uint16_t addr;
  addr =  (hdr.bit.param1 <<8) + hdr.bit.param2;

  enum OSP_ERROR_CODE ospErrorCode;
  switch (hdr.bit.byte_3)
    {
    case OSP_RESET:
    {
      send_ack (hdr, uartStatus);
      hal_reset_osire_start ();
      ospErrorCode = osp_reset (deviceAddress);
      if (ospErrorCode != OSP_NO_ERROR)
        {
          send_nack (hdr);
          return;
        }
      hal_reset_osire_end ();
      return;

    }

    case OSP_GO_ACTIVE:
    {
      ospErrorCode = osp2_send_goactive (addr);

      if (ospErrorCode != OSP_NO_ERROR)
          send_nack (hdr);
      else
		  send_ack (hdr, uartStatus);
      return;
    }

    case OSP_GO_SLEEP:
    {
      ospErrorCode = osp2_send_gosleep (addr);
      if (ospErrorCode != OSP_NO_ERROR)
          send_nack (hdr);
      else
		  send_ack (hdr, uartStatus);
      return;
    }

    case OSP_GO_DEEP_SLEEP:
    {
      ospErrorCode = osp2_send_godeepsleep (addr);
      if (ospErrorCode != OSP_NO_ERROR)
          send_nack (hdr);
      else
		  send_ack (hdr, uartStatus);
      return;
    }

    case OSP_CLRERROR:
    {
      ospErrorCode = osp2_send_clrerror (addr);
      if (ospErrorCode != OSP_NO_ERROR)
          send_nack (hdr);
      else
		  send_ack (hdr, uartStatus);
      return;
    }

    case OSP_IDENTIFY:
    {
    	uint32_t id;
    	uint8_t idcast;
    	ospErrorCode = osp2_send_identify(addr, &id);
		idcast = (uint8_t)id;
    	uint8_t sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_ID;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

        if (ospErrorCode != OSP_NO_ERROR)
        {
            send_nack (hdr);
            sendBuffer[UART_MSG_DATA0 + 1] =  255;
        }
        else
        {
		    send_ack (hdr, uartStatus);
			sendBuffer[UART_MSG_DATA0 + 1] =  idcast;
        }

        sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;

    }

    case OSP_READSTATUS:
    {
    	uint8_t status;
    	ospErrorCode = osp2_send_readstat(addr, &status);

    	uint8_t sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_ID;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

        if (ospErrorCode != OSP_NO_ERROR)
        {
            send_nack (hdr);
            sendBuffer[UART_MSG_DATA0 + 1] =  255;
        }
        else
        {
		    send_ack (hdr, uartStatus);
			sendBuffer[UART_MSG_DATA0 + 1] =  status;
        }

        sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;

    }

    case OSP_READTEMP:
    {
    	uint8_t temp;
    	ospErrorCode = osp2_send_readtemp(addr, &temp);

        uint8_t sendBuffer[LENGTH_UART_STD + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_STD;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

        if (ospErrorCode != OSP_NO_ERROR)
        {
            send_nack (hdr);
            sendBuffer[UART_MSG_DATA0 + 1] =  255;
        }
        else
        {
		    send_ack (hdr, uartStatus);
			sendBuffer[UART_MSG_DATA0 + 1] =  temp;
        }
        sendBuffer[LENGTH_UART_STD + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_STD + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer, LENGTH_UART_STD + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
    }

    case OSP_SET_PWM:
    {
    	uint16_t red;
    	uint16_t green;
    	uint16_t blue;

    	red = (hdr.bit.param4<< 8) + hdr.bit.param5;
    	green = (hdr.bit.param6<< 8) + hdr.bit.param7;
    	blue = (hdr.bit.param8<< 8) + hdr.bit.param9;
    	ospErrorCode = osp2_send_setpwmchn(addr, hdr.bit.param3, red, green, blue);
        if (ospErrorCode != OSP_NO_ERROR)
            send_nack (hdr);
        else
		    send_ack (hdr, uartStatus);
        return;
    }

    case OSP_SET_PWM_OS:
    {
    	uint16_t red;
    	uint16_t green;
    	uint16_t blue;
    	uint8_t daytimes;

    	red = (hdr.bit.param3<< 8) + hdr.bit.param4;
    	green = (hdr.bit.param5<< 8) + hdr.bit.param6;
    	blue = (hdr.bit.param7<< 8) + hdr.bit.param8;
    	daytimes = hdr.bit.param9;
    	ospErrorCode = osp2_send_setpwm(addr, red, green, blue, daytimes);
        if (ospErrorCode != OSP_NO_ERROR)
            send_nack (hdr);
        else
		    send_ack (hdr, uartStatus);
        return;
    }

    case OSP_READ_PWM:
    {
    	uint8_t chn;
    	uint16_t red;
    	uint16_t green;
    	uint16_t blue;

    	chn = hdr.bit.param3;

    	ospErrorCode = osp2_send_readpwmchn(addr, chn, &red, &green, &blue);

    	uint8_t sendBuffer[LENGTH_UART_PWM + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
		sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
		sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
		sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_INIT;
		sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

		if (ospErrorCode != OSP_NO_ERROR)
		{
			send_nack (hdr);
			sendBuffer[UART_MSG_DATA0 + 1] = 255;
			sendBuffer[UART_MSG_DATA0 + 2] = 255;
			sendBuffer[UART_MSG_DATA0 + 3] = 255;
			sendBuffer[UART_MSG_DATA0 + 4] = 255;
			sendBuffer[UART_MSG_DATA0 + 5] = 255;
			sendBuffer[UART_MSG_DATA0 + 6] = 255;
		}
		else
		{
			send_ack (hdr, uartStatus);
			sendBuffer[UART_MSG_DATA0 + 1] = red>>8;
			sendBuffer[UART_MSG_DATA0 + 2] = ((uint8_t) red ) & 0xFF;
			sendBuffer[UART_MSG_DATA0 + 3] = green>>8;
			sendBuffer[UART_MSG_DATA0 + 4] = ((uint8_t) green ) & 0xFF;
			sendBuffer[UART_MSG_DATA0 + 5] = blue>>8;
			sendBuffer[UART_MSG_DATA0 + 6] = ((uint8_t) blue ) & 0xFF;
		}

		sendBuffer[LENGTH_UART_PWM + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_PWM + LENGTH_UART_ANS_HEADER);
		uart_send_data_blocking (sendBuffer, LENGTH_UART_PWM + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
		return;
    }

    case OSP_READTEMPSTAT:
    {
    	uint8_t temp;
       	uint8_t status;
       	ospErrorCode = osp2_send_readtempstat(addr, &temp, &status);

       	uint8_t sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
           sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
           sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
           sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_16bit;
           sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

           if (ospErrorCode != OSP_NO_ERROR)
           {
               send_nack (hdr);
               sendBuffer[UART_MSG_DATA0 + 1] =  255;
               sendBuffer[UART_MSG_DATA0 + 2] =  255;
           }
           else
           {
   		    send_ack (hdr, uartStatus);
   			sendBuffer[UART_MSG_DATA0 + 1] =  temp;
   			sendBuffer[UART_MSG_DATA0 + 2] =  status;
           }

           sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER);
           uart_send_data_blocking (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
           return;

       }

    case OSP_READADC:
    {
    	uint8_t adc;
    	ospErrorCode = osp2_send_ADCread(addr, &adc);

    	uint8_t sendBuffer[LENGTH_UART_STD + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_STD;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

        if (ospErrorCode != OSP_NO_ERROR)
        {
            send_nack (hdr);
            sendBuffer[UART_MSG_DATA0 + 1] =  255;
        }
        else
        {
		    send_ack (hdr, uartStatus);
			sendBuffer[UART_MSG_DATA0 + 1] =  adc;
        }

        sendBuffer[LENGTH_UART_STD + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_STD + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer, LENGTH_UART_STD + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
    }

    case OSP_SETADC:
    {

    	ospErrorCode = osp2_send_setADC(addr, hdr.bit.param3);
        if (ospErrorCode != OSP_NO_ERROR)
            send_nack (hdr);
        else
		    send_ack (hdr, uartStatus);
        return;
    }

    case OSP_READADCDATA:
    {
    	uint16_t adcdata;
    	ospErrorCode = osp2_send_ADCdataread(addr, &adcdata);

    	uint8_t sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_16bit;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

        if (ospErrorCode != OSP_NO_ERROR)
        {
            send_nack (hdr);
            sendBuffer[UART_MSG_DATA0 + 1] =  255;
            sendBuffer[UART_MSG_DATA0 + 2] =  255;
        }
        else
        {
		    send_ack (hdr, uartStatus);
			sendBuffer[UART_MSG_DATA0 + 1] =  adcdata >> 8;
			sendBuffer[UART_MSG_DATA0 + 2] =  ((uint8_t) adcdata ) & 0xFF;
        }

        sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
    }

    case OSP_READCURRCHANNEL:
    {
    	uint8_t flags;
    	uint8_t rcur;
		uint8_t gcur;
		uint8_t bcur;
    	ospErrorCode = osp2_send_readcurchn(addr, hdr.bit.param3, &flags, &rcur, &gcur, &bcur);

        uint8_t sendBuffer[LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_INIT;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;

        if (ospErrorCode != OSP_NO_ERROR)
        {
            send_nack (hdr);
            sendBuffer[UART_MSG_DATA0 + 1] = 255;
	        sendBuffer[UART_MSG_DATA0 + 2] = 255;
	        sendBuffer[UART_MSG_DATA0 + 3] = 255;
	        sendBuffer[UART_MSG_DATA0 + 4] = 255;
        }
        else
        {
		    send_ack (hdr, uartStatus);
	        sendBuffer[UART_MSG_DATA0 + 1] = flags;
	        sendBuffer[UART_MSG_DATA0 + 2] = rcur;
	        sendBuffer[UART_MSG_DATA0 + 3] = gcur;
	        sendBuffer[UART_MSG_DATA0 + 4] = bcur;
        }

        sendBuffer[LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer, LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
    }

    case OSP_SETCURRCHANNEL:
    {
    	// osp2_send_setcurchn(uint16_t addr, uint8_t chn, uint8_t flags, uint8_t rcur, uint8_t gcur, uint8_t bcur ) {
    	ospErrorCode =  osp2_send_setcurchn(addr, hdr.bit.param3, hdr.bit.param4,hdr.bit.param5, hdr.bit.param6, hdr.bit.param7);
        if (ospErrorCode != OSP_NO_ERROR)
            send_nack (hdr);
        else
		    send_ack (hdr, uartStatus);
        return;
    }



    case OSP_INIT_BIDIR:
      {
        send_ack (hdr, uartStatus);
        ospInitRsp_t initRsp;
        ospErrorCode = osp_init_bidir (deviceAddress, &initRsp);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }


        uint8_t sendBuffer[LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_INIT;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = initRsp.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = initRsp.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = initRsp.data.bit.status;
        sendBuffer[UART_MSG_DATA0 + 4] = initRsp.data.bit.temp;
        sendBuffer[LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER] = crc (sendBuffer,
        LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }

    case OSP_INIT_LOOP:
      {
        send_ack (hdr, uartStatus);
        ospInitRsp_t initRsp;
        ospErrorCode = osp_init_loop (deviceAddress, &initRsp);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_INIT;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = initRsp.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = initRsp.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = initRsp.data.bit.status;
        sendBuffer[UART_MSG_DATA0 + 4] = initRsp.data.bit.temp;
        sendBuffer[LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER] = crc (sendBuffer,
        LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_INIT + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }

//    case OSP_TESTPW:
//      {
//
//    	  if(hdr.bit.param3 == 1)
//			  ospErrorCode =  osp2_send_testpw(addr, 0x4247525F4143);
//    	  else
//			  ospErrorCode =  osp2_send_testpw(addr, 0b00001);
//
//          if (ospErrorCode != OSP_NO_ERROR)
//            send_nack (hdr);
//          else
//  		    send_ack (hdr, uartStatus);
//          return;
//      }
//
//    case OSP_TESTDATASET:
//      {
//    	  uint16_t payload_data;
//
//    	  payload_data = (hdr.bit.param3 & 0x1F) | (hdr.bit.param4 << 5);
//
//
//		  ospErrorCode =  osp2_send_settestdata(addr, payload_data);
//
//          if (ospErrorCode != OSP_NO_ERROR)
//            send_nack (hdr);
//          else
//  		    send_ack (hdr, uartStatus);
//          return;
//      }
//
//    case OSP_SETI2CCFG:
//      {
//    	  uint8_t flags;
//    	  uint8_t speed;
//
//    	  flags = hdr.bit.param3;
//    	  speed = hdr.bit.param4;
//
//    	  ospErrorCode =  osp2_send_seti2ccfg(addr, flags, speed);
//
//          if (ospErrorCode != OSP_NO_ERROR)
//            send_nack (hdr);
//          else
//  		    send_ack (hdr, uartStatus);
//          return;
//      }
//
//    case OSP_READI2CCFG:
//      {
//    	  uint8_t flags;
//    	  uint8_t speed;
//
//    	  ospErrorCode =  osp2_send_readi2ccfg(addr, &flags, &speed);
//
//
//          uint8_t sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC] = {0};
//          sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
//          sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
//          sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_16bit;
//          sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
//
//          if (ospErrorCode != OSP_NO_ERROR)
//          {
//              send_nack (hdr);
//              sendBuffer[UART_MSG_DATA0 + 1] =  255;
//              sendBuffer[UART_MSG_DATA0 + 2] =  255;
//          }
//          else
//          {
//  		    send_ack (hdr, uartStatus);
//  	        sendBuffer[UART_MSG_DATA0 + 1] = flags;
//  	        sendBuffer[UART_MSG_DATA0 + 2] = speed;
//
//          }
//
//          sendBuffer[LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER] = crc (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER);
//          uart_send_data_blocking (sendBuffer, LENGTH_UART_16bit + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
//          return;
//      }
//
//    case OSP_WRITEI2C:
//      {
//    	  uint8_t daddr7;
//    	  uint8_t raddr;
//    	  uint8_t buf_i2c[8];
//    	  int size;
//
//    	  daddr7 = hdr.bit.param3;
//    	  raddr = hdr.bit.param4;
//    	  buf_i2c[0] = hdr.bit.param5;
//    	  buf_i2c[1] = hdr.bit.param6;
//    	  buf_i2c[2] = hdr.bit.param7;
//    	  buf_i2c[3] = hdr.bit.param8;
//    	  buf_i2c[4] = hdr.bit.param9;
//    	  buf_i2c[5] = hdr.bit.param10;
//    	  buf_i2c[6] = hdr.bit.param11;
//    	  buf_i2c[7] = hdr.bit.param12;
//    	  size = hdr.bit.param13;
//
//    	  ospErrorCode =  osp2_send_i2cwrite8(addr, daddr7, raddr, buf_i2c, size);
//
//          if (ospErrorCode != OSP_NO_ERROR)
//            send_nack (hdr);
//          else
//  		    send_ack (hdr, uartStatus);
//          return;
//      }
//
//    case OSP_READI2C:
//      {
//    	  uint8_t daddr7;
//    	  uint8_t raddr;
//    	  uint8_t num;
//
//    	  daddr7 = hdr.bit.param3;
//    	  raddr = hdr.bit.param4;
//    	  num = hdr.bit.param5;
//
//    	  ospErrorCode =  osp2_send_i2cread8(addr, daddr7, raddr, num );
//
//          if (ospErrorCode != OSP_NO_ERROR)
//            send_nack (hdr);
//          else
//  		    send_ack (hdr, uartStatus);
//          return;
//      }

    case OSP_OSIRE_GO_ACTIVE_SR:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_go_active_and_sr (deviceAddress, &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);

        return;
      }

    case OSP_OSIRE_SET_PWM:
      {
        send_ack (hdr, uartStatus);
        p_msg++;
        osirePwmData_t pwmData;
        pwmData.data.bit.red_pwm = *p_msg << 8;
        p_msg++;
        pwmData.data.bit.red_pwm = pwmData.data.bit.red_pwm + *p_msg;
        p_msg++;
        pwmData.data.bit.green_pwm = *p_msg << 8;
        p_msg++;
        pwmData.data.bit.green_pwm = pwmData.data.bit.green_pwm + *p_msg;
        p_msg++;
        pwmData.data.bit.blue_pwm = *p_msg << 8;
        p_msg++;
        pwmData.data.bit.blue_pwm = pwmData.data.bit.blue_pwm + *p_msg;
        p_msg++;
        if (*p_msg == 1)
          {
            pwmData.data.bit.red_curr = 1;
            pwmData.data.bit.green_curr = 1;
            pwmData.data.bit.blue_curr = 1;
          }
        else
          {
            pwmData.data.bit.red_curr = 0;
            pwmData.data.bit.green_curr = 0;
            pwmData.data.bit.blue_curr = 0;
          }
        ospErrorCode = osp_osire_set_pwm (deviceAddress, pwmData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        return;
      }
    case OSP_OSIRE_READ_PWM:
      {
        send_ack (hdr, uartStatus);
        osirePwmData_t pwmData;
        ospErrorCode = osp_osire_read_pwm (deviceAddress, &pwmData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_PWM_READ + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_PWM_READ;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = pwmData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = pwmData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = pwmData.data.bit.red_pwm >> 8;
        sendBuffer[UART_MSG_DATA0 + 4] = pwmData.data.bit.red_pwm & 0xff;
        sendBuffer[UART_MSG_DATA0 + 5] = pwmData.data.bit.green_pwm >> 8;
        sendBuffer[UART_MSG_DATA0 + 6] = pwmData.data.bit.green_pwm & 0xff;
        sendBuffer[UART_MSG_DATA0 + 7] = pwmData.data.bit.blue_pwm >> 8;
        sendBuffer[UART_MSG_DATA0 + 8] = pwmData.data.bit.blue_pwm & 0xff;
        if (pwmData.data.bit.red_curr == 1)
          {
            sendBuffer[UART_MSG_DATA0 + 9] = 1;
          }
        else
          {
            sendBuffer[UART_MSG_DATA0 + 9] = 0;
          }
        sendBuffer[LENGTH_UART_PWM_READ + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_PWM_READ + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_PWM_READ + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);

        return;
      }
    case OSP_OSIRE_SET_SETUP:
      {
        send_ack (hdr, uartStatus);
        p_msg = p_msg + 1;
        osireSetSetupData_t setupData;
        setupData.data.setupData = *p_msg;
        ospErrorCode = osp_osire_set_setup (deviceAddress, setupData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        return;
      }
    case OSP_OSIRE_READ_SETUP:
      {
        send_ack (hdr, uartStatus);
        osireSetSetupData_t setupData;
        ospErrorCode = osp_osire_read_setup (deviceAddress, &setupData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_SETUP_READ + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];

        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_SETUP_READ;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = setupData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = setupData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = setupData.data.setupData;

        sendBuffer[LENGTH_UART_SETUP_READ + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_SETUP_READ + LENGTH_UART_ANS_HEADER);

        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_SETUP_READ + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }
    case OSP_OSIRE_READ_OTP:
      {
        send_ack (hdr, uartStatus);
        p_msg++;
        uint8_t otpAddress = *p_msg;
        osireOtpData_t otpData;
        ospErrorCode = osp_read_otp (deviceAddress, otpAddress, &otpData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_OTP_READ + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_OTP_READ;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = otpData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = otpData.address & 0xff;
        for (uint8_t i = 0; i < LENGTH_UART_OTP_READ - 3; i++)
          {
            sendBuffer[i + UART_MSG_DATA0 + 3] = otpData.data.byte[i];
          }
        sendBuffer[LENGTH_UART_OTP_READ + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_OTP_READ + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_OTP_READ + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }
    case OSP_OSIRE_READ_OTP_COMPLETE:
      send_ack (hdr, uartStatus);
      osireOtpDataComplete_t otpDataComplete;
      ospErrorCode = osp_osire_read_otp_complete (deviceAddress,
                                                  &otpDataComplete);
      if (ospErrorCode != OSP_NO_ERROR)
        {
          send_nack (hdr);
          return;
        }

      uint8_t sendBuffer[LENGTH_UART_OTP_COMPLETE + LENGTH_UART_ANS_HEADER
          + LENGTH_UART_ANS_CRC];
      sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
      sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
      sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_OTP_COMPLETE;
      sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
      sendBuffer[UART_MSG_DATA0 + 1] = otpDataComplete.address >> 8;
      sendBuffer[UART_MSG_DATA0 + 2] = otpDataComplete.address & 0xff;
      for (uint8_t i = 0; i < LENGTH_UART_OTP_COMPLETE - 3; i++)
        {
          sendBuffer[i + UART_MSG_DATA0 + 3] = otpDataComplete.data.byte[i];
        }
      sendBuffer[LENGTH_UART_OTP_COMPLETE + LENGTH_UART_ANS_HEADER] = crc (
          sendBuffer,
          LENGTH_UART_OTP_COMPLETE + LENGTH_UART_ANS_HEADER);
      uart_send_data_blocking (sendBuffer,
      LENGTH_UART_OTP_COMPLETE + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
      return;

    case OSP_OSIRE_SET_SETUP_SR:
      {
        send_ack (hdr, uartStatus);
        p_msg = p_msg + 1;
        osireSetSetupData_t setupData;
        osireTempStatus_t tempStatus;
        setupData.data.setupData = *p_msg;
        ospErrorCode = osp_osire_set_setup_and_sr (deviceAddress, setupData,
                                                   &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);

        return;
      }

    case OSP_OSIRE_SET_PWM_SR:
      {
        send_ack (hdr, uartStatus);
        p_msg++;
        osirePwmData_t pwmData;
        pwmData.data.bit.red_pwm = *p_msg << 8;
        p_msg++;
        pwmData.data.bit.red_pwm = pwmData.data.bit.red_pwm + *p_msg;
        p_msg++;
        pwmData.data.bit.green_pwm = *p_msg << 8;
        p_msg++;
        pwmData.data.bit.green_pwm = pwmData.data.bit.green_pwm + *p_msg;
        p_msg++;
        pwmData.data.bit.blue_pwm = *p_msg << 8;
        p_msg++;
        pwmData.data.bit.blue_pwm = pwmData.data.bit.blue_pwm + *p_msg;
        p_msg++;
        if (*p_msg == 1)
          {
            pwmData.data.bit.red_curr = 1;
            pwmData.data.bit.green_curr = 1;
            pwmData.data.bit.blue_curr = 1;
          }
        else
          {
            pwmData.data.bit.red_curr = 0;
            pwmData.data.bit.green_curr = 0;
            pwmData.data.bit.blue_curr = 0;
          }

        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_set_pwm_and_sr (deviceAddress, pwmData,
                                                 &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);

        return;
      }
    case OSP_OSIRE_READ_STATUS:
      {
        send_ack (hdr, uartStatus);
        osireStatus_t statusData;
        ospErrorCode = osp_osire_read_status (deviceAddress, &statusData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_STATUS + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];

        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_STATUS;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = statusData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = statusData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = statusData.data.status;

        sendBuffer[LENGTH_UART_STATUS + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_STATUS + LENGTH_UART_ANS_HEADER);

        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_STATUS + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }
    case OSP_OSIRE_READ_TEMP:
      {
        send_ack (hdr, uartStatus);
        osireTemp_t tempData;
        ospErrorCode = osp_osire_read_temp (deviceAddress, &tempData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_TEMP + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];

        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_TEMP;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = tempData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = tempData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = tempData.data.temp_value;

        sendBuffer[LENGTH_UART_TEMP + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_TEMP + LENGTH_UART_ANS_HEADER);

        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_TEMP + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }

    case OSP_OSIRE_SET_OTTH:
      {
        send_ack (hdr, uartStatus);

        p_msg++;
        osireOtthData_t otthData;
        otthData.data.otthData[0] = *p_msg;
        p_msg++;
        otthData.data.otthData[1] = *p_msg;
        p_msg++;
        otthData.data.otthData[2] = *p_msg;

        ospErrorCode = osp_osire_set_otth (deviceAddress, otthData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        return;
      }

    case OSP_OSIRE_SET_OTTH_SR:
      {
        send_ack (hdr, uartStatus);
        p_msg++;
        osireOtthData_t otthData;
        otthData.data.otthData[0] = *p_msg;
        p_msg++;
        otthData.data.otthData[1] = *p_msg;
        p_msg++;
        otthData.data.otthData[2] = *p_msg;

        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_set_otth_and_sr (deviceAddress, otthData,
                                                  &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);

        return;
      }

    case OSP_OSIRE_READ_OTTH:
      {
        send_ack (hdr, uartStatus);
        osireOtthData_t otthData;
        ospErrorCode = osp_osire_read_otth (deviceAddress, &otthData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_OTTH_READ + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];
        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_OTTH_READ;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = otthData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = otthData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = otthData.data.otthData[0];
        sendBuffer[UART_MSG_DATA0 + 4] = otthData.data.otthData[1];
        sendBuffer[UART_MSG_DATA0 + 5] = otthData.data.otthData[2];

        sendBuffer[LENGTH_UART_OTTH_READ + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_OTTH_READ + LENGTH_UART_ANS_HEADER);
        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_OTTH_READ + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);

        return;
      }

    case OSP_OSIRE_READ_COM_STATUS:
      {
        send_ack (hdr, uartStatus);
        osireComStatus_t comStatusData;
        ospErrorCode = osp_osire_read_comstatus (deviceAddress, &comStatusData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGHT_UART_COM_STATUS + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];

        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGHT_UART_COM_STATUS;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = comStatusData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = comStatusData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = comStatusData.data.comStatus;

        sendBuffer[LENGHT_UART_COM_STATUS + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGHT_UART_COM_STATUS + LENGTH_UART_ANS_HEADER);

        uart_send_data_blocking (sendBuffer,
        LENGHT_UART_COM_STATUS + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }

    case OSP_OSIRE_READ_LED_STATUS:
      {
        send_ack (hdr, uartStatus);
        osireLedStatus_t ledStateData;
        ospErrorCode = osp_osire_read_ledstatus (deviceAddress, &ledStateData);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        uint8_t sendBuffer[LENGTH_UART_LED_STATUS + LENGTH_UART_ANS_HEADER
            + LENGTH_UART_ANS_CRC];

        sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
        sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
        sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_LED_STATUS;
        sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
        sendBuffer[UART_MSG_DATA0 + 1] = ledStateData.address >> 8;
        sendBuffer[UART_MSG_DATA0 + 2] = ledStateData.address & 0xff;
        sendBuffer[UART_MSG_DATA0 + 3] = ledStateData.data.ledStatus;

        sendBuffer[LENGTH_UART_LED_STATUS + LENGTH_UART_ANS_HEADER] = crc (
            sendBuffer, LENGTH_UART_LED_STATUS + LENGTH_UART_ANS_HEADER);

        uart_send_data_blocking (sendBuffer,
        LENGTH_UART_LED_STATUS + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
        return;
      }
    case OSP_OSIRE_READ_TEMP_STATUS:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_read_tempstatus (deviceAddress, &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);
        return;
      }


    case OSP_OSIRE_GO_SLEEP_SR:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_go_sleep_and_sr (deviceAddress, &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);

        return;
      }
    case OSP_OSIRE_GO_DEEP_SLEEP_SR:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_go_deep_sleep_and_sr (deviceAddress,
                                                       &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);

        return;
      }
    case OSP_OSIRE_P4ERROR_BIDIR:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_p4error_bidir (deviceAddress, &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);
        return;
      }
    case OSP_OSIRE_P4ERROR_LOOP:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_p4error_loop (deviceAddress, &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);
        return;
      }
    case OSP_CLR_ERROR:
      send_ack (hdr, uartStatus);
      ospErrorCode = osp_osire_clr_error (deviceAddress);
      if (ospErrorCode != OSP_NO_ERROR)
        {
          send_nack (hdr);
          return;
        }

      return;
    case OSP_OSIRE_CLR_ERROR_SR:
      {
        send_ack (hdr, uartStatus);
        osireTempStatus_t tempStatus;
        ospErrorCode = osp_osire_clr_error_and_sr (deviceAddress, &tempStatus);
        if (ospErrorCode != OSP_NO_ERROR)
          {
            send_nack (hdr);
            return;
          }

        send_uart_temp_status (hdr, tempStatus);
        return;
      }
    default:
      uartStatus.bit.cmd_not_implemented = 1;
      send_ack (hdr, uartStatus);
      return;
    }
}

void send_uart_temp_status (uartHeader_t hdr, osireTempStatus_t tempStatus)
{
  uint8_t sendBuffer[LENGTH_UART_TEMP_STATUS + LENGTH_UART_ANS_HEADER
      + LENGTH_UART_ANS_CRC];

  sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
  sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
  sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_TEMP_STATUS;
  sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
  sendBuffer[UART_MSG_DATA0 + 1] = tempStatus.address >> 8;
  sendBuffer[UART_MSG_DATA0 + 2] = tempStatus.address & 0xff;
  sendBuffer[UART_MSG_DATA0 + 3] = tempStatus.data.tempStatus[0];
  sendBuffer[UART_MSG_DATA0 + 4] = tempStatus.data.tempStatus[1];

  sendBuffer[LENGTH_UART_TEMP_STATUS + LENGTH_UART_ANS_HEADER] = crc (
      sendBuffer, LENGTH_UART_TEMP_STATUS + LENGTH_UART_ANS_HEADER);
  uart_send_data_blocking (sendBuffer,
  LENGTH_UART_TEMP_STATUS + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
}

void send_uart_identify (uartHeader_t hdr, uint8_t id)
{
  uint8_t sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER
      + LENGTH_UART_ANS_CRC];

  sendBuffer[UART_MSG_DEVICEID] = hdr.bit.deviceID;
  sendBuffer[UART_MSG_GROUP] = GROUP_OSPCOMMANDS;
  sendBuffer[UART_MSG_LENGTH] = LENGTH_UART_ID;
  sendBuffer[UART_MSG_DATA0] = hdr.bit.byte_3;
  sendBuffer[UART_MSG_DATA0 + 1] =  id;


  sendBuffer[LENGTH_UART_ID + LENGTH_UART_ANS_HEADER] = crc (
      sendBuffer, LENGTH_UART_ID + LENGTH_UART_ANS_HEADER);
  uart_send_data_blocking (sendBuffer,
		  LENGTH_UART_ID + LENGTH_UART_ANS_HEADER + LENGTH_UART_ANS_CRC);
}

