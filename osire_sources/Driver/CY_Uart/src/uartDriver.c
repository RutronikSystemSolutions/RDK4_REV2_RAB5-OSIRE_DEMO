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
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdlib.h>
#include <Common/inc/osireEvkDefines.h>
#include <Driver/CY_Uart/inc/ringBuffer.h>
#include "Driver/CY_Uart/inc/uartDriver.h"

// ---- PRIVATE FUNCTIONS: ----------------------------------------------------
void uart_driver_set_baud_rate (uint32_t desiredBaudRate);
void uart_driver_irq_handler (void);
void uart_driver_rx_irq_handler (void);
void uart_driver_tx_complete_irq_handler (void);
void uart_driver_tx_empty_irq_handler (void);
void uart_driver_error_irq_handler (void);

void UART_Isr(void);
// ---- VARIABLES: ------------------------------------------------------------

static status_t uartStatus = STATUS_SUCCESS;

 uint8_t receiveRingBuffer[UART_DRIVER_BUFFER_SIZE] =
  { 0 };
 uint8_t transmitRingBuffer[UART_DRIVER_BUFFER_SIZE] =
  { 0 };

ringBuffer_t receiveRingBufferHandle =
  { .p_buffer = receiveRingBuffer, .bufferSize = UART_DRIVER_BUFFER_SIZE,
      .readIndex = 0, .writeIndex = 0 };

ringBuffer_t transmitRingBufferHandle =
  { .p_buffer = transmitRingBuffer, .bufferSize = UART_DRIVER_BUFFER_SIZE,
      .readIndex = 0, .writeIndex = 0 };

cy_stc_sysint_t KITPROG_UART_INT_config =
{
    .intrSrc      = KITPROG_UART_IRQ,
    .intrPriority = 0u, // cm0 = ? , cm4 = 7
};

// ---- IMPLEMENTATIONS: ------------------------------------------------------
cy_en_scb_uart_status_t uart_driver_init(void)
{
	cy_en_scb_uart_status_t cy_uart_status = CY_SCB_UART_SUCCESS;

	/*UART Initialisation*/
	cy_uart_status = Cy_SCB_UART_Init(KITPROG_UART_HW, &KITPROG_UART_config, &KITPROG_UART_Context);

	/* Hook interrupt service routine and enable interrupt */
	(void) Cy_SysInt_Init(&KITPROG_UART_INT_config, &UART_Isr);
	Cy_SCB_UART_StartRingBuffer(KITPROG_UART_HW, receiveRingBuffer, UART_DRIVER_BUFFER_SIZE, &KITPROG_UART_Context);
	Cy_SCB_UART_Enable(KITPROG_UART_HW);
	NVIC_EnableIRQ(KITPROG_UART_IRQ);

	return cy_uart_status;
}
/**
 * @brief
 *
 */
void UART_Isr(void)
{
    Cy_SCB_UART_Interrupt(KITPROG_UART_HW, &KITPROG_UART_Context);
}

/**
 * @fn status_t uart_driver_get_status(void)
 * @brief
 *
 * @return
 */
status_t uart_driver_get_status(void)
{
	if (0 != (CY_SCB_UART_TRANSMIT_ACTIVE & Cy_SCB_UART_GetTransmitStatus(KITPROG_UART_HW, &KITPROG_UART_Context)))
	{
		uartStatus = STATUS_BUSY;
	}
	else
	{
		uartStatus = STATUS_SUCCESS;
	}
	return uartStatus;
}


uint8_t uart_driver_send_data(const uint8_t *p_txBuff, uint32_t txSize)
{
	cy_en_scb_uart_status_t driver_status;

	/*Check UART status machine*/
	if (uartStatus == STATUS_BUSY)
	{
		return STATUS_BUSY;
	}

	driver_status = Cy_SCB_UART_Transmit(KITPROG_UART_HW, (void*)p_txBuff, txSize, &KITPROG_UART_Context);

	if (driver_status != CY_SCB_UART_SUCCESS)
	{
		return STATUS_ERROR;
	}
	else
	{
		return STATUS_SUCCESS;
	}
}

status_t uart_driver_receive_data(void)
{
	if (uartStatus == STATUS_BUSY)
	{
		return STATUS_BUSY;
	}
	else
	{
		uartStatus = STATUS_BUSY;
	}

	return STATUS_SUCCESS;
}

uint16_t uart_driver_get_rx_buffer_level (void)
{
	uint16_t nbr_bytes = 0;

	/*Get the received data from the ring buffer first*/
	nbr_bytes = Cy_SCB_UART_GetNumInRingBuffer(KITPROG_UART_HW, &KITPROG_UART_Context);
	/*Data in the FIFO also counts*/
	nbr_bytes += Cy_SCB_UART_GetNumInRxFifo(KITPROG_UART_HW);

	return nbr_bytes;
}

bool uart_driver_get_rx_data (uint8_t *p_rxBuff, uint32_t rxSize)
{
    cy_en_scb_uart_status_t status = 0;

    status = Cy_SCB_UART_Receive(KITPROG_UART_HW, p_rxBuff, rxSize,&KITPROG_UART_Context);

    if(status != CY_SCB_UART_SUCCESS )
    {
    	return false;
    }

    return true;
}

uint16_t uart_driver_get_tx_buffer_level (void)
{
  return get_ring_buffer_fill_level (&transmitRingBufferHandle);
}

void uart_driver_complete_send_data_using_int (void)
{
  uartStatus = STATUS_SUCCESS;
}

uint8_t uart_driver_complete_receive_data_using_int (void)
{
  return uartStatus;
}

// ---- PRIVATE FUNCTIONS IMPLEMENTATION: -------------------------------------
/**
 * @fn void uart_driver_set_baud_rate(uint32_t)
 * @brief  wird in Device Configurator eingestellt und dementsprechend initialisiert
 *
 * @param desiredBaudRate
 */
void uart_driver_set_baud_rate (uint32_t desiredBaudRate)
{
}

/**
 * @fn void uart_driver_rx_irq_handler(void)
 * @brief wird nicht mehr benötigt --> CY PDL Ringbuffer für RX used
 *
 */
void uart_driver_rx_irq_handler (void)
{
}

void uart_driver_tx_complete_irq_handler (void)
{
  if (get_ring_buffer_fill_level (&transmitRingBufferHandle) == 0)
    {
      uart_driver_complete_send_data_using_int ();
    }

}

void uart_driver_tx_empty_irq_handler (void)
{
}

void uart_driver_irq_handler (void)
{
}

void uart_driver_error_irq_handler (void)
{
}
