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
#ifndef OSP_INC_OSIRE_DEVICE_H_
#define OSP_INC_OSIRE_DEVICE_H_

#ifdef __cplusplus
extern "C"
  {
#endif

/**
 * Brief OSP Library
 *
 * Customer API for RGBi LED Stripe communication
 * 
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 */

/**
 * Include OSP generic definitions and data structures
 */
#include <Osp/inc/genericDevice.h>

/*****************************************************************************/
/*********************** defines *********************************************/
#define LENGTH_SET_SETUP_MSG 5
/*****************************************************************************/
#define LENGTH_SET_PWM_MSG 10
/*****************************************************************************/
#define LENGTH_SET_OTTH_MSG 7
/*****************************************************************************/
#define LENGTH_READ_PWM_MSG 4
#define LENGTH_READ_PWM_RSP 10
/*****************************************************************************/
#define LENGTH_READ_OTP_MSG 5
#define LENGTH_READ_OTP_RSP 12
/*****************************************************************************/
#define LENGTH_READ_SETUP_MSG 4
#define LENGTH_READ_SETUP_RSP 5
/*****************************************************************************/
#define LENGTH_READ_TEMPSTATUS_MSG 4
#define LENGTH_READ_TEMPSTATUS_RSP 6
/*****************************************************************************/
#define LENGTH_READ_TEMP_MSG 4
#define LENGTH_READ_TEMP_RSP 5
/*****************************************************************************/
#define LENGTH_READ_OTTH_MSG 4
#define LENGTH_READ_OTTH_RSP 7
/*****************************************************************************/
#define LENGTH_READ_COMSTATUS_MSG 4
#define LENGTH_READ_COMSTATUS_RSP 5
/*****************************************************************************/
#define LENGTH_READ_STATUS_MSG 4
#define LENGTH_READ_STATUS_RSP 5
/*****************************************************************************/
#define LENGTH_READ_LEDSTATUS_MSG 4
#define LENGTH_READ_LEDSTATUS_RSP 5

/*****************************************************************************/
#define LENGTH_P4ERROR_MSG 4

/*****************************************************************************/
/*****************************************************************************/
/**
 * Enumeration OSP OSire Device commands
 */
enum OSP_OSIRE_DEVICE_CMDS
{
  OSP_OSIRE_P4ERROR_BIDIR = 0x08, /**< implemented */
  OSP_OSIRE_CLR_ERROR_SR = 0x21, /**< implemented */
  OSP_OSIRE_GO_SLEEP_SR = 0x24, /**< implemented */
  OSP_OSIRE_GO_ACTIVE_SR = 0x25, /**< implemented */
  OSP_OSIRE_GO_DEEP_SLEEP_SR = 0x26, /**< implemented */
  OSP_OSIRE_READ_STATUS = 0x40, /**< implemented */
  OSP_OSIRE_READ_TEMP_STATUS = 0x42, /**<implemented */
  OSP_OSIRE_READ_COM_STATUS = 0x44, /**< implemented */
  OSP_OSIRE_READ_LED_STATUS = 0x46, /**< implemented */
  OSP_OSIRE_READ_TEMP = 0x48, /**< implemented */
  OSP_OSIRE_READ_OTTH = 0x4A, /**< implemented */
  OSP_OSIRE_SET_OTTH = 0x4B, /**< implemented */
  OSP_OSIRE_SET_OTTH_SR = 0x6B, /**< implemented */
  OSP_OSIRE_READ_SETUP = 0x4C, /**< implemented */
  OSP_OSIRE_SET_SETUP = 0x4D, /**< implemented */
  OSP_OSIRE_SET_SETUP_SR = 0x6D, /**< implemented */
  OSP_OSIRE_READ_PWM = 0x4E, /**< implemented */
  OSP_OSIRE_SET_PWM = 0x4F, /**< implemented */
  OSP_OSIRE_SET_PWM_SR = 0x6F, /**< implemented */
  OSP_OSIRE_READ_OTP = 0x58, /**< implemented */

  OSP_OSIRE_P4ERROR_LOOP = 0x09 /**< implemented; precondition: OSP_INIT_LOOP */
};

/*****************************************************************************/
/*****************************************************************************/
typedef struct SetSetupData_t
{
  union
  {
    uint8_t setupData; /**< SETUP DATA register */
    struct
    {
      uint8_t uv_fsave :1; /**< undervoltage error is detected */
      uint8_t ot_fsave :1; /**< overtemperature error is detected */
      uint8_t los_fsave :1; /**< open/short error is detected */
      uint8_t ce_fsave :1; /**< communication error is detected */
      uint8_t tempck_sel :1; /**< Update rate of the temperature sensor */
      uint8_t crc_en :1; /**< CRC check */
      uint8_t com_inv :1; /**< CLK polarity for MCU mode */
      uint8_t fast_pwm :1; /**< PWM frequency and dynamic range */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} osireSetSetupData_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */

/*****************************************************************************/
/*****************************************************************************/
typedef struct PwmData_t
{
  union
  {
    uint8_t pwmData[6]; /**< PWM data buffer */
    struct
    {
      uint16_t blue_pwm :15; /**< blue PWM value */
      uint16_t blue_curr :1; /**< day or night mode */
      uint16_t green_pwm :15; /**< green PWM value */
      uint16_t green_curr :1; /**< day or night mode */
      uint16_t red_pwm :15; /**< red PWM value */
      uint16_t red_curr :1; /**< day or night mode */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} osirePwmData_t;

/*****************************************************************************/
/*****************************************************************************/
typedef struct OtpData_t
{
  union
  {
    uint8_t byte[8]; /**< buffer for one otp read block */
  } data;
  uint16_t address :10; /**< device address */
} osireOtpData_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */

/*****************************************************************************/
/*****************************************************************************/
typedef struct OtpDataComplete_t
{
  union
  {
    uint8_t byte[32]; /**< max. buffer for full otp read memory */
  } data;
  uint16_t address :10; /**< device address */
} osireOtpDataComplete_t;

/*****************************************************************************/
/*****************************************************************************/
typedef struct LedStatus_t
{
  union
  {
    uint8_t ledStatus; /**< LED STATUS register. Cleared after readout. */
    struct
    {
      uint8_t blue_short :1; /**< BLUE channel short fault flag */
      uint8_t green_short :1; /**< GREEN channel short fault flag */
      uint8_t red_short :1; /**< RED channel short fault flag */
      uint8_t ledStatus_reserved_1 :1; /**< reserved */
      uint8_t blue_open :1; /**< BLUE channel open fault flag. */
      uint8_t green_open :1; /**< GREEN channel open fault flag. */
      uint8_t red_open :1; /**< RED channel open fault flag. */
      uint8_t ledStatus_reserved_2 :1; /**< reserved */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} osireLedStatus_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */

/*****************************************************************************/
/*****************************************************************************/
typedef struct osireTemp_t
{
  union
  {
    uint8_t temp_value; /**< TEMP register */
  } data;
  uint16_t address :10; /**< device address */
} osireTemp_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */

/*****************************************************************************/
/*****************************************************************************/
typedef struct ComStatus_t
{
  union
  {
    uint8_t comStatus; /**< COM STATUS register */
    struct
    {
      uint8_t sio1_state :2; /**< Communication mode of SIO1 */
      uint8_t sio2_state :2; /**< Communication mode of SIO2 */
      uint8_t reserved :4; /**< reserved */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} osireComStatus_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */

/*****************************************************************************/
/*****************************************************************************/
typedef struct Status_t
{
  union
  {
    uint8_t status; /**< STATUS register */
    struct
    {
      uint8_t uv_flag :1; /**< undervoltage fault flag */
      uint8_t ot_flag :1; /**< overtemperature fault flag */
      uint8_t los_flag :1; /**< LED fault flag */
      uint8_t ce_flag :1; /**< communication fault flag */
      uint8_t com_mode :1; /**< communication direction */
      uint8_t otpcrc_flag :1; /**< OTP error flag */
      uint8_t state :2; /**< device state */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} osireStatus_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */

/*****************************************************************************/
/*****************************************************************************/
typedef struct TempStatus_t
{
  union
  {
    uint8_t tempStatus[2]; /**< STATUS + TEMP registers */
    struct
    {
      uint8_t Status;
      uint8_t Temp;
    } byte;
  } data;
  uint16_t address :10; /**< device address */
} osireTempStatus_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */
/*****************************************************************************/
/*****************************************************************************/
typedef struct OtthData_t
{
  union
  {
    uint8_t otthData[3]; /**< STATUS register */
    struct
    {
      uint8_t ot_high_value :8; /**< overtemperature fault threshold */
      uint8_t ot_low_value :8; /**< overtemperature fault release threshold */
      uint8_t or_cycle :2; /**< overtemperature detection low pass filter cycle length */
      uint8_t otth_reserved :6; /**< reserved */
    } bit;
  } data;
  uint16_t address :10; /**< device address */
} osireOtthData_t; /**<For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf" */
/*****************************************************************************/
/*****************************************************************************/

/**
 * @brief OSP_OSIRE_SET_SETUP command
 *
 * Writes the SETUP register. See READSETUP for the payload format.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress  0..1000 RGBi device address (0: broadcast)
 * @param data SETUP register values
 * 
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_set_setup (uint16_t deviceAddress,
                                         osireSetSetupData_t data);

/**
 * @brief OSP_OSIRE_SET_SETUP_SR command and reads status and temperature
 *
 * Writes the SETUP register. See READSETUP for the payload format.
 * Additionally the STATUS and TEMP register is read.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress, 0..1000 RGBi device address (0: broadcast)
 * @param data SETUP register values
 * @param p_data STATUS and TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_set_setup_and_sr (uint16_t deviceAddress,
                                                osireSetSetupData_t data,
                                                osireTempStatus_t *p_data);

/**
 * @brief OSP_OSIRE_SET_PWM command
 *
 * Writes the PWM register. See READPWM for the payload format.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 0..1000 RGBi device address (0: broadcast)
 * @param data PWM register value
 *
 * @return error, communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_set_pwm (uint16_t deviceAddress,
                                       osirePwmData_t data);

/**
 * @brief OSP_OSIRE_SET_PWM_SR command and reads status and temperature
 *
 * Writes the PWM register. See READPWM for the payload format.
 * Additionally the STATUS and TEMP register is read.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 0..1000 RGBi device address (0: broadcast)
 * @param data PWM register value
 * @param p_data STATUS and TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_set_pwm_and_sr (uint16_t deviceAddress,
                                              osirePwmData_t data,
                                              osireTempStatus_t *p_data);

/**
 * @brief OSP_OSIRE_READ_PWM command

 * This function reads the PWM register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data PWM register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_pwm (uint16_t deviceAddress,
                                        osirePwmData_t *p_data);

/**
 * @brief OSP_OSIRE_READ_OTP command
 * Reads 8 bytes from OTP memory, from otpAddress.
 * If readout address is beyond OTP address range, 0x00 will be delivered
 * for these addresses.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param otpAddress address of the first read byte of OTP
 * @param p_data OTP response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_read_otp (uint16_t deviceAddress, uint8_t otpAddress,
                                  osireOtpData_t *p_data);

/**
 * @brief Read complete OTP memory command
 * Reads all bytes from OTP memory. By use of several consecutive
 * OSP_OSIRE_READ_OTP commands
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data full OTP memory data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_otp_complete (uint16_t deviceAddress,
                                                 osireOtpDataComplete_t *p_data);

/**
 * @brief OSP_OSIRE_READ_LED_STATUS command
 * This function reads the LED STATUS register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data LED STATUS register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_ledstatus (uint16_t deviceAddress,
                                              osireLedStatus_t *p_data);

/**
 * @brief OSP_OSIRE_READ_TEMP_STATUS command
 * This function reads the STATUS and TEMP register in a single 2-byte
 * payload of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data STATUS and TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_tempstatus (uint16_t deviceAddress,
                                               osireTempStatus_t *p_data);

/**
 * @brief OSP_OSIRE_READ_TEMP command
 * This function reads LED TEMP register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data LED TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_temp (uint16_t deviceAddress,
                                         osireTemp_t *p_data);

/**
 * @brief OSP_OSIRE_SET_OTTH command
 *
 * Writes the OTTH register. See READOTTH for the payload format.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 0..1000 RGBi device address (0: broadcast)
 * @param data PWM register value
 *
 * @return error, communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_set_otth (uint16_t deviceAddress,
                                        osireOtthData_t data);

/**
 * @brief OSP_OSIRE_SET_OTTH_SR command and reads status and temperature
 *
 * Writes the OTTH register. See READOTTH for the payload format.
 * Additionally the STATUS and TEMP register is read.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param data PWM register value
 * @param p_data STATUS and TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_set_otth_and_sr (uint16_t deviceAddress,
                                               osireOtthData_t data,
                                               osireTempStatus_t *p_data);

/**
 * @brief OSP_OSIRE_READ_OTTH command

 * This function reads the OTTH register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data PWM register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_otth (uint16_t deviceAddress,
                                         osireOtthData_t *p_data);

/**
 * @brief OSP_OSIRE_GO_SLEEP_SR command and read status and temperature
 * This function sends one or all devices into SLEEP state
 * Additionally the STATUS and TEMP register is read.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *

 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data STATUS and TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_go_sleep_and_sr (uint16_t deviceAddress,
                                               osireTempStatus_t *p_data);

/**
 * @brief OSP_OSIRE_GO_DEEP_SLEEP_SR command and read status and temperature
 * This function sends one or all devices into DEEP_SLEEP state
 * Additionally the STATUS and TEMP register is read.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *

 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data STATUS and TEMP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_go_deep_sleep_and_sr (uint16_t deviceAddress,
                                                    osireTempStatus_t *p_data);

/**
 * @brief OSP_CLEAR_ERROR command and read status and temperature
 * This function will clear all error flags,
 * if an error still exists for example short/open the error flag is set again and LED TEMPSTAT Register will be sent.
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data, pointer to response data from RGBi LED
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_clr_error_and_sr (uint16_t deviceAddress,
                                                osireTempStatus_t *p_data);

/**
 * @brief OSP_PING_FOR_ERROR_BIDIR command
 * Addressed or initialized device (typ. 1st.), checks if error flag occurs.
 * It will check only the selected flag bits which leads to SLEEP state (setup register).
 * If yes: answer to master (state + temperature) if not:
 * same command will be forward to next device with a new address field (STARTADDRESS+1).
 * If no error flag bit in chain. Last device sends status + temperature register.
 * Note:
 * The command shall use the address of the first device only.
 * Using an address from a unit in the middle of the chain might lead to unpredictable behavior
 * of the chain if another unit saw a reset condition (e.g. power loss).
 *
 * @param deviceAddress of first RGBi device (typ. 1)
 * @param p_data, pointer to response data from RGBi LED
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_p4error_bidir (uint16_t deviceAddress,
                                             osireTempStatus_t *p_data);

/**
 * @brief OSP_PING_FOR_ERROR_LOOP command
 * Addressed or initialized device (typ. 1st.), checks if error flag occurs.
 * It will check only the selected flag bits which leads to SLEEP state (setup register).
 * If yes: answer to master (state + temperature) if not:
 * same command will be forward to next device with a new address field (STARTADDRESS+1).
 * If no error flag bit in chain. Last device sends status + temperature register.
 * Note:
 * The command shall use the address of the first device only.
 * Using an address from a unit in the middle of the chain might lead to unpredictable behavior
 * of the chain if another unit saw a reset condition (e.g. power loss).
 *
 * @param deviceAddress of first RGBi device (typ. 1)
 * @param p_data, pointer to response data from RGBi LED
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_p4error_loop (uint16_t deviceAddress,
                                            osireTempStatus_t *p_data);

/**
 * @brief OSP_GO_ACTIVE command and read status and temperature
 * Puts device into ACTIVE state: The LED drivers are enabled. The last PWM
 * parameters are used to create LED PWM signals. An PWM parameter update via
 * SETPWM() command is possible. During ACTIVE mode, diagnostic is possible
 * and readable.
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress  1..1000 RGBi device address
 * @param p_data, pointer to response data from RGBi LED
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_go_active_and_sr (uint16_t deviceAddress,
                                          osireTempStatus_t *p_rsp);

/**
 * @brief OSP_CLEAR_ERROR command and read status and temperature
 * This function will clear all error flags,
 * if an error still exists for example short/open the error flag is set again and LED TEMPSTAT Register will be sent.
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data, pointer to response data from RGBi LED
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_clr_error_and_sr (uint16_t deviceAddress,
                                                osireTempStatus_t *p_data);

/**
 * @brief OSP_PING_FOR_ERROR_BIDIR command
 * Addressed or initialized device (typ. 1st.), checks if error flag occurs.
 * It will check only the selected flag bits which leads to SLEEP state (setup register).
 * If yes: answer to master (state + temperature) if not:
 * same command will be forward to next device with a new address field (STARTADDRESS+1).
 * If no error flag bit in chain. Last device sends status + temperature register.
 * Note:
 * The command shall use the address of the first device only.
 * Using an address from a unit in the middle of the chain might lead to unpredictable behavior
 * of the chain if another unit saw a reset condition (e.g. power loss).
 *
 * @param deviceAddress of first RGBi device (typ. 1)
 * @param p_data, pointer to response data from RGBi LED
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_p4error_bidir (uint16_t deviceAddress,
                                             osireTempStatus_t *p_data);

/**
 * @brief OSP_OSIRE_READ_SETUP command
 * This function reads SETUP register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data SETUP register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_setup (uint16_t deviceAddress,
                                          osireSetSetupData_t *p_data);

/**
 * @brief OSP_OSIRE_READ_COM_STATUS command
 * This function reads COM STATUS register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data pointer to response data from RGBi LED
 *
 * @return error COM STATUS register response data
 */
enum OSP_ERROR_CODE osp_osire_read_comstatus (uint16_t deviceAddress,
                                              osireComStatus_t *p_data);

/**
 * @brief OSP_OSIRE_READ_STATUS command
 * This function reads STATUS register of the device
 *
 * For further details refer to "OSIRE_E3731i_Start_Up_Guide.pdf"
 *
 * @param deviceAddress 1..1000 RGBi device address
 * @param p_data STATUS register response data
 *
 * @return error communication or command parameter error
 */
enum OSP_ERROR_CODE osp_osire_read_status (uint16_t deviceAddress,
                                           osireStatus_t *p_data);

#ifdef __cplusplus
}
#endif

#endif  // OSP_INC_OSIRE_DEVICE_H_
