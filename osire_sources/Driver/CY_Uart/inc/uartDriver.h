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

#ifndef DRIVER_UART_DRIVER_H_
#define DRIVER_UART_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
//#include "SDK/platform/devices/status.h"

//status wrapper
typedef enum
{
	STATUS_SUCCESS 	= 0x00,
	STATUS_BUSY 	= 0x01,
	STATUS_TIMEOUT 	= 0x02,
	STATUS_ERROR	= 0x03

}status_t;

#define UART_DRIVER_BUFFER_SIZE 264  // Buffer Size > Longest Possible Message

/* Allocate context for UART operation */
extern cy_stc_scb_uart_context_t KITPROG_UART_Context;

/**
 * @brief Initializes Uart driver with lpuart1_InitConfig0 values.
 *
 * @return STATUS_SUCCESS if initialization was successful, error code
 *                        otherwise.
 */
cy_en_scb_uart_status_t uart_driver_init (void);

/**
 * @brief Getter for the uart status.
 *
 * @return uart status.
 */
status_t uart_driver_get_status (void);

/**
 * @brief This function sends data out through the LPUART module using
 *        blocking method.
 *
 * @param txBuff Data for transfer.
 * @param txSize Size of the transfer data in bytes.
 * @return STATUS_SUCCESS if send was success, error code otherwise.
 */
uint8_t uart_driver_send_data (const uint8_t *p_txBuff, uint32_t txSize);

/**
 * @brief This function receives data from LPUART module using non-blocking
 *        method. This function returns immediately after initiating the
 *        receive function.
 *
 * @return STATUS_SUCCESS if send was success, error code otherwise.
 */
status_t uart_driver_receive_data (void);

/**
 * @brief Getter for the data from the receiving buffer.
 *
 * @param rxBuff Destination buffer.
 * @param rxSize Size of the data which should be retrieved in bytes.
 * @return True if the data is retrieved successful, false otherwise.
 */
bool uart_driver_get_rx_data (uint8_t *p_rxBuff, uint32_t rxSize);

/**
 * @brief Completes the non blocking receiving function, and disables the interrupts.
 *
 * @return STATUS_SUCCESS if complete was success, error code otherwise.
 */
uint8_t uart_driver_complete_receive_data_using_int (void);

/**
 * @brief Completes the non blocking sending function, and disables the interrupts.
 */
void uart_driver_complete_send_data_using_int (void);

/**
 * @brief Getter for the fill level of the receiving buffer used for
 *        non-blocking data receiving.
 *
 * @return Number of used bytes in receiving buffer.
 */
uint16_t uart_driver_get_rx_buffer_level (void);

/**
 * @brief Getter for the fill level of the transmitting buffer used for
 *        non-blocking data receiving.
 *
 * @return Number of used bytes in transmitting buffer.
 */
uint16_t uart_driver_get_tx_buffer_level (void);

#endif  // DRIVER_UART_DRIVER_H_
