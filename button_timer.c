/*
 * button_timer.c
 *
 *  Created on: 2024-02-13
 *      Author: GDR
 */

#include "button_timer.h"
#include <osire_sources/Hal/CY_Gpios/inc/pin.h>

#define BUTTON_TIMER_FREQ_HZ			1000u

static void button_ind_timer_isr(void *callback_arg, cyhal_timer_event_t event);

/* Timer object used */
cyhal_timer_t button_ind_timer_obj;

cyhal_timer_cfg_t button_ind_timer_cfg =
{
    .compare_value = 0,
    .period = 0xFFFFFFFF,
    .direction = CYHAL_TIMER_DIR_UP,
    .is_compare = true,
    .is_continuous = false,
    .value = 0
};


_Bool button_ind_complete = true;

cy_rslt_t button_ind_timer_init (void)
{
	cy_rslt_t rslt = CY_RSLT_SUCCESS;

    /* Initialize the timer object */
    rslt = cyhal_timer_init(&button_ind_timer_obj, NC, NULL);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /* Set the frequency of timer to 1000 Hz  */
    rslt = cyhal_timer_set_frequency(&button_ind_timer_obj, BUTTON_TIMER_FREQ_HZ);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&button_ind_timer_obj, button_ind_timer_isr, NULL);

    cyhal_timer_enable_event(&button_ind_timer_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE, 3, false);

	return rslt;
}

cy_rslt_t button_ind_timer_start (uint32_t delay_ms)
{
	cy_rslt_t rslt = CY_RSLT_SUCCESS;

	/*Delay begins from here*/
	button_ind_complete = false;

    /* Reset configuration */
    rslt = cyhal_timer_reset(&button_ind_timer_obj);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /* Apply timer configuration */
    button_ind_timer_cfg.compare_value = delay_ms;
    rslt = cyhal_timer_configure(&button_ind_timer_obj, &button_ind_timer_cfg);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /*Enable Timer Compare Interrupt*/
    cyhal_timer_enable_event(&button_ind_timer_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE, 3, true);

    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&button_ind_timer_obj);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

	return rslt;
}

/*Delay ends with this Interrupt*/
static void button_ind_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /*Stop the Timer and Disable Timer Compare Interrupt*/
    cyhal_timer_stop(&button_ind_timer_obj);
    cyhal_timer_enable_event(&button_ind_timer_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE, 3, false);

    /*Turn OFF LEDs here*/
    set_led_red (0);
    set_led_green (0);
    set_led_blue (0);

    button_ind_complete = true;
}



