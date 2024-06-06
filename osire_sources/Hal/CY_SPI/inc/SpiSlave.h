/******************************************************************************
* File Name: SpiSlave.h
*
* Description: This file contains all the function prototypes required for
*              SPI Slave implemented using Serial Communication Block (SCB)
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#ifndef SOURCE_SPISLAVE_H_
#define SOURCE_SPISLAVE_H_

#include "cy_pdl.h"
#include "cycfg.h"

#include <Hal/CY_SPI/inc/spiGeneral.h>

#define MAX_MESSAGE_LENGTH 13
#define COUNT_MESSAGE 5
#define OSP_PREAMBLE 0xA
#define MAX_TIME_DELAY_MS 1
#define NEGATIVE_MAX_TIME_DELAY_MS   (MAX_TIME_DELAY_MS*(-1))

typedef enum
{
  SPI_NEW_MESSAGE_OK, SPI_NO_NEW_MESSAGE, SPI_ERROR_DATA_CORRUPTION

} errorCodeSpiNewMessage_t;

typedef enum
{
  SPI_RECEIVE_DEFAULT,
  SPI_RECEIVE_WAITING,
  SPI_RECEIVE_FINISHED,
  SPI_RECEIVE_ERROR_TO_SLOW,
  SPI_RECEIVE_ERROR_CORRUPT_DATA

} spiReceiveStatusSlave_t;

typedef union
{
  uint8_t data[3];
  struct
  {
    uint8_t address_high :4; //Byte 1 bit 0 -> 3
    uint8_t preamble :4;    //Byte 1 bit 4 -> 7
    uint8_t psi_high :2;     //Byte 2 bit 0 -> 1
    uint8_t address_low :6;  //Byte 2 bit 2 -> 7
    uint8_t command :7;      //Byte 3 bit 0 -> 6
    uint8_t psi_low :1;      //Byte 3 bit 7
  } bit;

} osireHeader_t;


/*******************************************************************************
*         Function Prototypes
*******************************************************************************/
uint32_t CY_init_SPI_Slave(void);
uint32_t read_packet(uint8_t *, uint32_t);
errorSpi_t hal_spi_slave_receive_for_blocking (uint8_t *bufferReceiveIn,uint16_t count);
errorCodeSpiNewMessage_t hal_check_for_bytes_received_for_blocking (void);
spiReceiveStatusSlave_t hal_spi_receive_control (void);
uint8_t* hal_get_new_message (errorCodeSpiNewMessage_t *error);
void hal_reset_spi_slave (void);
void hal_spi_receive_reset_buffer (void);
#endif /* SOURCE_SPISLAVE_H_ */

/* [] END OF FILE */
