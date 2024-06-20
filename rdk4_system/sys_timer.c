/*
 * sys_timer.c
 *
 *  Created on: 2024-01-29
 *      Author: GDR
 */

#include "sys_timer.h"

static uint32_t sys_time_ms = 0;
static void systic_callback(void)
{
	sys_time_ms++;
}

_Bool sys_timer_init(void)
{
    Cy_SysTick_EnableInterrupt();
    Cy_SysTick_Enable();


    /* Find unused callback slot. */
    for (uint32_t i = 0u; i < CY_SYS_SYST_NUM_OF_CALLBACKS; ++i)
    {
        if (Cy_SysTick_GetCallback(i) == NULL)
        {
            /* Set callback */
        	Cy_SysTick_SetCallback(i, systic_callback);
            return true;
        }
    }

	return false;
}

uint32_t get_system_time_ms(void)
{
	return sys_time_ms;
}

uint32_t *get_system_time_pointer_ms(void)
{
	return &sys_time_ms;
}
