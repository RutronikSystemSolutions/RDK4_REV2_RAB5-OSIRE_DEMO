/******************************************************************************
* File Name:   SpiSlave.c
*
* Description: This file contains function definitions for SPI Slave.
*
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

#include "SpiSlave.h"
#include "Interface.h"
#include <Driver/Spi/inc/spiSlaveDriver.h>
#include <Hal/CY_Uart/inc/uart.h>


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
cy_stc_scb_spi_context_t sSPI_context;

static uint8_t messageBuffer[COUNT_MESSAGE][MAX_MESSAGE_LENGTH];
static volatile bool messageRead[COUNT_MESSAGE];
static volatile uint8_t messageCounter = 0;
static volatile uint8_t lastReadBytePosition = SIZE_OF_SPI_RECEIVE_BUFFER - 1;
static volatile uint8_t lastReadMessage = 0;
static volatile bool firstMessage = true;

static uint8_t *p_bufferReceive;
static uint8_t lengthBuffer;

/* Assign SPI interrupt number and priority */
#define sSPI_INTR_PRIORITY   (3U)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static void SPI_Isr(void);

/*******************************************************************************
 * Function Name: sSPI_Interrupt
 *******************************************************************************
 *
 * Invokes the Cy_SCB_SPI_Interrupt() PDL driver function.
 *
 *******************************************************************************/
static void SPI_Isr(void)
{
    Cy_SCB_SPI_Interrupt(sSPI_HW, &sSPI_context);
}

/*******************************************************************************
* Function Name: init_slave
********************************************************************************
*
* Summary:
*  This function initializes the SPI Slave based on the
*  configuration done in design.modus file.
*
* Parameters:
*  None
*
* Return:
*  (uint32) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32_t CY_init_SPI_Slave(void)
{
    cy_en_scb_spi_status_t spi_status;
    cy_en_sysint_status_t intr_status;

    /* Configure the SPI block */
    spi_status = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &sSPI_context);

    /* If the initialization fails, return failure status */
    if(spi_status != CY_SCB_SPI_SUCCESS)
    {
        return(INIT_FAILURE);
    }

    /* Set active slave select to line 0 */
    //Cy_SCB_SPI_SetActiveSlaveSelect(sSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    /* Populate configuration structure */
    const cy_stc_sysint_t spi_intr_config =
    {
        .intrSrc      = sSPI_IRQ,
        .intrPriority = sSPI_INTR_PRIORITY,
    };

    /* Hook interrupt service routine and enable interrupt */
    intr_status = Cy_SysInt_Init(&spi_intr_config, &SPI_Isr);

    if(intr_status != CY_SYSINT_SUCCESS)
    {
        return(INIT_FAILURE);
    }

    NVIC_EnableIRQ(sSPI_IRQ);

    /* Enable the SPI Slave block */
    Cy_SCB_SPI_Enable(sSPI_HW);

    /* Initialization completed */
    return(INIT_SUCCESS);
}


/******************************************************************************
* Function Name: read_packet
*******************************************************************************
*
* Summary:
*  This function reads the data received by the slave. Note that
*  the below function is blocking until the required number of
*  bytes is received by the slave.
*
* Parameters:
*  - (uint8_t *) rxBuffer - Pointer to the receive buffer where data
*                          needs to be stored
*  - (uint32_t) transferSize - Number of bytes to be received
*
* Return:
*  - (uint32_t) - Returns TRANSFER_COMPLETE if SPI transfer is completed or
*                 returns TRANSFER_FAILURE if SPI tranfer is not successfull
*
******************************************************************************/
uint32_t read_packet(uint8_t *rxBuffer, uint32_t transferSize)
    {
    uint32_t slave_status;
    cy_en_scb_spi_status_t status;

    /* Prepare for a transfer. */
    status = Cy_SCB_SPI_Transfer(sSPI_HW, NULL, rxBuffer, transferSize, &sSPI_context);

    if (status == CY_SCB_SPI_SUCCESS)
	{
	/* Blocking wait for transfer completion */
	while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE &\
 Cy_SCB_SPI_GetTransferStatus(sSPI_HW, &sSPI_context)))
	    {
	    }

	/* Check start and end of packet markers */
	if ((rxBuffer[PACKET_SOP_POS] == PACKET_SOP) && (rxBuffer[PACKET_EOP_POS] == PACKET_EOP))
	    {
	    /* Data received correctly */
	    slave_status = TRANSFER_COMPLETE;
	    }
	else
	    {
	    /* Data was not received correctly */
	    slave_status = TRANSFER_FAILURE;
	    }
	}
    else
	{
	/* SPI transfer not initiated */
	slave_status = TRANSFER_FAILURE;
	}

    return slave_status;
    }

/**
 * @fn errorSpi_t hal_spi_slave_receive_for_blocking(uint8_t*, uint16_t)
 * @brief
 *
 * @param bufferReceiveIn
 * @param count
 * @return
 */
errorSpi_t hal_spi_slave_receive_for_blocking (uint8_t *bufferReceiveIn,uint16_t count)
{
  spiHalError = NO_ERROR_SPI;
  p_bufferReceive = bufferReceiveIn;
  lengthBuffer = count;
//*start Änderungen A.Heder
//  cy_en_scb_spi_status_t slaveStatus;

  Cy_SCB_SPI_Enable(sSPI_HW);
  /* Initiate SPI Master write transaction. */
  /*slaveStatus = */(void)Cy_SCB_SPI_Transfer(sSPI_HW, NULL, p_bufferReceive,count, &sSPI_context);
//*End
  return (0);
}
errorCodeSpiNewMessage_t hal_check_for_bytes_received_for_blocking (void)
{
  errorCodeSpiNewMessage_t err = SPI_NO_NEW_MESSAGE;
  uint8_t *p_buffer;
  spiReceiveStatusSlave_t status = hal_spi_receive_control ();
  p_buffer = hal_get_new_message (&err);

  if (err != SPI_NO_NEW_MESSAGE)
    {
      memcpy (p_bufferReceive, p_buffer, lengthBuffer);
    }
  else if (status == SPI_RECEIVE_ERROR_CORRUPT_DATA)
    {
      err = SPI_ERROR_DATA_CORRUPTION;
    }

  return (err);
}

spiReceiveStatusSlave_t hal_spi_receive_control(void)
    {
    bool complete = false;
    bool new_data = false;
    uint8_t length = 0;
    uint8_t neededLength = 0;
    spiReceiveStatusSlave_t status = SPI_RECEIVE_DEFAULT;

    uint8_t psi = 0;
    osireHeader_t *p_header;
    uint8_t tempBufferHeader[4];

    uint8_t lastReceivedBytePosition = get_buffer_position();

    //-1 because the position in the receive buffer is at the next position which is free in the receive buffer
    if (lastReceivedBytePosition == 0)
	{
	lastReceivedBytePosition = SIZE_OF_SPI_RECEIVE_BUFFER - 1;
	}
    else
	{
	lastReceivedBytePosition--;
	}

    uint8_t pos = 0;
    uint32_t timeStampPreviousReceivedByte;
    uint32_t timeStampLastReceivedByte;
    int32_t timeStampDiff;

    if (lastReceivedBytePosition != lastReadBytePosition)
	{

	pos = lastReceivedBytePosition;

	int16_t readCount = lastReceivedBytePosition - lastReadBytePosition;

	if (readCount < 0)
	    {
	    readCount = SIZE_OF_SPI_RECEIVE_BUFFER - 1 + readCount; //readCount is negative!
	    }

	for (int8_t i = 0; i < readCount; i++)
	    {
	    timeStampLastReceivedByte = *(get_buffer_time_stamp() + pos);

	    if (pos == 0)
		{
		pos = SIZE_OF_SPI_RECEIVE_BUFFER - 1;
		}
	    else
		{
		pos--;
		}

	    timeStampPreviousReceivedByte = *(get_buffer_time_stamp() + pos);

	    timeStampDiff = timeStampLastReceivedByte - timeStampPreviousReceivedByte;

	    if ((timeStampDiff > MAX_TIME_DELAY_MS) || (timeStampDiff < NEGATIVE_MAX_TIME_DELAY_MS))
		{
		lastReadBytePosition = pos;
		break;
		}

	    }

	}

    if (lastReceivedBytePosition < lastReadBytePosition) // we have wrapped around
	{
	length = get_max_size_buffer() + lastReceivedBytePosition + 1;
	length = length - lastReadBytePosition;
	}
    else
	{
	length = lastReceivedBytePosition - lastReadBytePosition;
	}

    if (length >= 3)
	{
	new_data = true;

	if (length > MAX_MESSAGE_LENGTH)
	    {
	    //we have a problem! //we are to slow!
	    status = SPI_RECEIVE_ERROR_TO_SLOW;
	    }
	}

    if (new_data == true)
	{

	if ((lastReadBytePosition < (SIZE_OF_SPI_RECEIVE_BUFFER - 3)) || (lastReadBytePosition == (SIZE_OF_SPI_RECEIVE_BUFFER - 1)))
	    {
	    if (lastReadBytePosition != (SIZE_OF_SPI_RECEIVE_BUFFER - 1))
		{
		p_header = (osireHeader_t*) (get_buffer() + lastReadBytePosition + 1);
		}
	    else
		{
		p_header = (osireHeader_t*) (get_buffer());
		}
	    }
	else //header is split in the buffer!
	    {

	    uint8_t i = 0, j = 0;
	    for (; i < (SIZE_OF_SPI_RECEIVE_BUFFER - lastReadBytePosition - 1); i++)
		{
		tempBufferHeader[i] = *(get_buffer() + lastReadBytePosition + 1 + i);
		}
	    for (; i < 3; i++)
		{
		tempBufferHeader[i] = *(get_buffer() + j);
		j++;
		}

	    p_header = (osireHeader_t*) tempBufferHeader;
	    }

	status = SPI_RECEIVE_WAITING;

	if (p_header->bit.preamble == OSP_PREAMBLE)
	    {
	    //check length

	    psi = (uint8_t) (p_header->bit.psi_high << 1);
	    psi = psi + p_header->bit.psi_low;

	    if (psi == 0x7)
		{
		neededLength = 12; //this is fix!
		}
	    else
		{
		neededLength = psi + 4;
		}

	    if (neededLength <= length)
		{
		complete = true;
		}
	    else
		{
		complete = false;
		}
	    }
	else
	    {
	    //we have a problem! -> we have corrupted Data!
	    status = SPI_RECEIVE_ERROR_CORRUPT_DATA;
	    }
	}

    if (complete == true)
	{
	messageRead[messageCounter] = false;
	uint8_t position = 0;

	position = lastReadBytePosition + 1;
	if (position >= get_max_size_buffer())
	    {
	    position = 0;
	    }

	uint8_t byteStored = 0;

	for (uint8_t i = 0; i < MAX_MESSAGE_LENGTH; i++)
	    {
	    messageBuffer[messageCounter][i] = 0xAA;
	    }

	for (uint8_t i = 0; i < neededLength; i++)
	    {
	    //copy data
	    messageBuffer[messageCounter][i] = *(get_buffer() + position);
	    position++;
	    byteStored++;
	    if (position >= get_max_size_buffer())
		{
		position = 0;
		}
	    }

	messageCounter++;
	if (messageCounter >= COUNT_MESSAGE)
	    {
	    messageCounter = 0;
	    }

	lastReadBytePosition = lastReadBytePosition + byteStored;

	if (lastReadBytePosition >= get_max_size_buffer())
	    {
	    lastReadBytePosition = lastReadBytePosition - get_max_size_buffer();
	    }

	status = SPI_RECEIVE_FINISHED;
	}

    if (status == SPI_RECEIVE_ERROR_CORRUPT_DATA)
	{
	hal_reset_spi_slave();
	}

    return (status);
    }
/**
 * @fn void hal_reset_spi_slave(void)
 * @brief
 *
 */
void hal_reset_spi_slave (void)
{
  hal_spi_receive_reset_buffer ();
  dri_spi_s_init ();
}
/**
 * @fn uint8_t hal_get_new_message*(errorCodeSpiNewMessage_t*)
 * @brief
 *
 * @param p_error
 * @return
 */
uint8_t* hal_get_new_message (errorCodeSpiNewMessage_t *p_error)
{
  uint8_t i = 0, j = 0;
  uint8_t *p_return = NULL;
  //lastReadMessage;

  *p_error = SPI_NO_NEW_MESSAGE;

  //j = lastReadMessage;

  for (; i < COUNT_MESSAGE; i++)
    {
      if (j >= COUNT_MESSAGE)
        {
          j = 0;
        }

      if (messageRead[j] == false)
        {
          //lastReadMessage = j;
          messageRead[j] = true;
          *p_error = SPI_NEW_MESSAGE_OK;
          p_return = messageBuffer[j];
          firstMessage = false;
          break;
        }
      j++;
    }

  return (p_return);
}
/**
 * @fn void hal_spi_receive_reset_buffer(void)
 * @brief
 *
 */
void hal_spi_receive_reset_buffer (void)
{
  messageCounter = 0;
  lastReadBytePosition = SIZE_OF_SPI_RECEIVE_BUFFER - 1;
  lastReadMessage = 0;

  for (uint8_t i = 0; i < COUNT_MESSAGE; i++)
    {
      messageRead[i] = true;
    }

  firstMessage = true;

  dri_spi_s_reset_buffer ();
}


