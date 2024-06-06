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
#ifndef OSP_INC_GENERIC_DEVICE_H_
#define OSP_INC_GENERIC_DEVICE_H_

#ifdef __cplusplus
extern "C"
  {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * Brief OSP Library - Generic Device
 *
 * Customer API for devices that are communicating with
 * OSP Generic Device Protocol
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 * 
 */

/**
 * Defines OSP Generic Device commands
 */
/*****************************************************************************/
/*********************** defines *********************************************/
#define OSP_PROTOCOL_PREAMPLE 0x0A
#define FIRST_BYTE_PAYLOAD 3

/*****************************************************************************/
/*********************** defines *********************************************/
#define BROADCAST_ADDRESS 0
#define MAXIMUM_ADDRESS 1002

/*****************************************************************************/
/*********************** defines *********************************************/
#define LENGTH_INIT_MSG 4
#define LENGTH_INIT_RSP 6
/*****************************************************************************/
#define LENGTH_RESET_MSG 4
/*****************************************************************************/
#define LENGTH_GO_ACTIVE_MSG 4
/*****************************************************************************/
#define LENGTH_GO_SLEEP_MSG 4
/*****************************************************************************/
#define LENGTH_GO_DEEP_SLEEP_MSG 4
/*****************************************************************************/
#define LENGTH_CLR_ERROR_MSG 4
/*****************************************************************************/
#define NO_OSP_RSP false
#define OSP_RSP !NO_OSP_RSP
#define LENGTH_NO_OSP_RSP 0
/*****************************************************************************/
/**
 * Enumeration OSP Generic Device commands
 */
enum OSP_GENERIC_DEVICE_CMDS
{
  OSP_RESET = 0x00, /**< implemented */
  OSP_CLR_ERROR = 0x01, /**< implemented */
  OSP_INIT_BIDIR = 0x02, /**< implemented */
  OSP_INIT_LOOP = 0x03, /**< implemented */
  OSP_GO_SLEEP = 0x04, /**< implemented */
  OSP_GO_ACTIVE = 0x05, /**< implemented */
  OSP_GO_DEEP_SLEEP = 0x06, /**< implemented */
};

/*****************************************************************************/
/*****************************************************************************/
/**
 * Enumeration OSP Generic Device error codes
 */
enum OSP_ERROR_CODE
{
  OSP_NO_ERROR = 0x00, /**< no error */
  OSP_ADDRESS_ERROR, /**< invalid device address*/
  OSP_ERROR_INITIALIZATION, /**< error while initializing */
  OSP_ERROR_CRC, /**< incorrect CRC of OSP command */

  OSP_ERROR_SPI, /**< SPI interface error */
  OSP_ERROR_PARAMETER, /**< invalid parameter error */
  OSP_ERROR_NOT_IMPLEMENTED /**< CMD not implemented error */
};

/*****************************************************************************/
/*****************************************************************************/
/**
 * Enumeration OSP Generic Device error codes
 */
typedef struct InitRsp_t
{
  union
  {
    uint8_t rsp[10]; /**< response buffer for OSP_INIT_BIDIR command */
    struct
    {
      uint8_t temp; /**< temperature */
      uint8_t status; /**< current status */
      uint16_t address :10; /**< device address */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} ospInitRsp_t;

/*****************************************************************************/
/*****************************************************************************/
typedef union
{
  uint8_t buf[4]; /**< header buffer */
  struct
  {
    uint32_t reserved :8; /**< reserved */
    uint32_t command :7; /**< OSP or OSP_OSIRE command */
    uint32_t psi :3; /**< payload in bytes */
    uint32_t address :10; /**< device address */
    uint32_t preample :4; /**< OSP_PROTOCOL_PREAMPLE 0x0A */
  } bit;
} ospHeader_t;

/**
 * @brief OSP_INIT_BIDIR command
 * Initiates the automatic addressing of the chain and sets the communication
 * direction to bidirectional. The command shall be addressed to the first
 * unit in the chain always with address 0x001. The last unit in the
 * chain (indicated by the EOL mode) returns its address to the master
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress start address of 1st device, shall be 1
 * @param p_rsp response data from device
 * 
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_init_bidir (uint16_t deviceAddress, ospInitRsp_t *p_rsp);

/**
 * @brief OSP_RESET command
 * Performs a complete reset of one or all devices. The effect is
 * identical to a power cycle. All register values are set to their
 * default values, all error flags are cleared, the communication mode
 * detection is restarted, LED drivers are turned off, and the address is
 * set to 0x3ff. The device enters the UNINITIALIZED mode.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceId  0..1000 RGBi device address (0: broadcast)
 * 
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_reset (uint16_t deviceAddress);

/**
 * @brief OSP_GO_ACTIVE command.
 * Puts device into ACTIVE state: The LED drivers are enabled. The last PWM
 * parameters are used to create LED PWM signals. An PWM parameter update via
 * SETPWM() command is possible. During ACTIVE mode, diagnostic is possible
 * and readable.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress  0..1000 RGBi device address (0: broadcast)
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_go_active (uint16_t deviceAddress);

/**
 * @brief OSP_INIT_LOOP command
 * Same as INITBIDIR but sets the communication mode to loop-back.
 * The response to the master is sent in the forward direction.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide"
 * 
 * @param deviceAddress start address of 1st device, shall be 1
 * @param p_rsp response data from device
 * 
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_init_loop (uint16_t deviceAddress, ospInitRsp_t *p_rsp);

/**
 * @brief OSP_GO_SLEEP command
 * Sends one or all devices into SLEEP state.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress  0..1000 RGBi device address (0: broadcast)
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_go_sleep (uint16_t deviceAddress);

/**
 * @brief OSP_GO_DEEP_SLEEP command
 * Sends one or all devices into DEEPSLEEP state.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress  0..1000 RGBi device address (0: broadcast)
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_go_deep_sleep (uint16_t deviceAddress);

/**
 * @brief OSP_CLEAR_ERROR command
 * This function will clear all error flags,
 * if an error still exists for example short/open the error flag is set again.
 *
 * @param deviceAddress  0..1000 RGBi device address (0: broadcast)
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_clr_error (uint16_t deviceAddress);

/**
 * @brief Internal function for OSP header creation
 *
 * @param p_msg message buffer to create the OSP header
 * @param deviceAddress 0..1000 RGBi device address (0: broadcast)
 * @param command OSP or OSP_OSIRE command
 * @param lengthMsg length of the buffer that is used
 * lengthMsg
 *
 * @return error communication or command parameter error
 */
void build_header (uint8_t *p_msg, uint16_t deviceAddress, uint8_t command,
                   uint8_t lengthMsg);

#ifdef __cplusplus
}
#endif

#endif  // OSP_INC_GENERIC_DEVICE_H_
