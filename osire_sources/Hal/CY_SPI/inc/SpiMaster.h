/******************************************************************************
* File Name:   main.c
*
* Description: This file contains all the function prototypes required for
*              SPI Master implemented using Serial Communication Block (SCB)
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef SOURCE_SPIMASTER_H_
#define SOURCE_SPIMASTER_H_

#include "cy_pdl.h"
#include "cycfg.h"

extern cy_stc_scb_spi_context_t mSPI_context;

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Initialization status */
//#define INIT_SUCCESS            (0)
//#define INIT_FAILURE            (1)

/* Element index in the packet */
//#define PACKET_SOP_POS          (0UL)
//#define PACKET_CMD_POS          (1UL)
//#define PACKET_EOP_POS          (2UL)

/* TX Packet Head and Tail */
//#define PACKET_SOP          (0x01UL)
//#define PACKET_EOP          (0x17UL)

/* Assign SPI interrupt priority */
#define mSPI_INTR_PRIORITY  (3U)

/***************************************
*         Function Prototypes
****************************************/
cy_en_scb_spi_status_t CY_init_SPI_Master(void);
cy_en_scb_spi_status_t hal_spi_master_send_blocking(uint8_t *txBuffer, uint32_t transferSize);
cy_en_scb_spi_status_t hal_spi_master_send_non_blocking(uint8_t *p_bufferSend, uint8_t count, uint32_t delay);

#endif /* SOURCE_SPIMASTER_H_ */

/* [] END OF FILE */
