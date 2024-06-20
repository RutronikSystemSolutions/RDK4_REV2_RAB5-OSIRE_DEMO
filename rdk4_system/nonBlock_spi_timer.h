/*
 * nonBlock_spi_timer.h
 *
 *  Created on: 2024-02-08
 *      Author: GDR
 */

#ifndef NONBLOCK_SPI_TIMER_H_
#define NONBLOCK_SPI_TIMER_H_

#include <stdio.h>
#include <stdlib.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

extern cyhal_timer_t one_shot_spi_timer_obj;
extern uint8_t *pBuffSPImaster;
extern uint8_t SPImaster_count;
extern _Bool transfer_delay_complete;

cy_rslt_t one_shot_spi_timer_init (void);
cy_rslt_t one_shot_timer_spi_start (uint32_t delay_ms);

#endif /* NONBLOCK_SPI_TIMER_H_ */
