/*
 * button_timer.h
 *
 *  Created on: 2024-02-13
 *      Author: GDR
 */

#ifndef BUTTON_TIMER_H_
#define BUTTON_TIMER_H_

#include <stdio.h>
#include <stdlib.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#define BUTTON_IND_HOLD_MS			250u
extern _Bool button_ind_complete;

cy_rslt_t button_ind_timer_init (void);
cy_rslt_t button_ind_timer_start (uint32_t delay_ms);

#endif /* BUTTON_TIMER_H_ */
