/*
 * sys_timer.h
 *
 *  Created on: 2024-01-29
 *      Author: GDR
 */

#ifndef SYS_TIMER_H_
#define SYS_TIMER_H_

#include "cy_pdl.h"
#include "cybsp.h"


_Bool sys_timer_init(void);
uint32_t get_system_time_ms(void);
uint32_t *get_system_time_pointer_ms(void);


#endif /* SYS_TIMER_H_ */
