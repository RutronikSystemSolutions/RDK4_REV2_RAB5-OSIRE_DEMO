/*
 * nonBlock_spi_timer.c
 *
 *  Created on: 2024-02-08
 *      Author: GDR
 */


#include "nonBlock_spi_timer.h"
#include <CY_Gpios/inc/pin.h>
#include "SpiMaster.h"

#define ONE_SHOT_TIMER_FREQ_HZ			1000u

static void one_shot_spi_timer_isr(void *callback_arg, cyhal_timer_event_t event);

/* Timer object used */
cyhal_timer_t one_shot_spi_timer_obj;

cyhal_timer_cfg_t one_shot_spi_timer_cfg =
{
    .compare_value = 0,
    .period = 0xFFFFFFFF,
    .direction = CYHAL_TIMER_DIR_UP,
    .is_compare = true,
    .is_continuous = false,
    .value = 0
};

uint8_t *pBuffSPImaster = NULL;
uint8_t SPImaster_count = 0;
_Bool transfer_delay_complete = true;

cy_rslt_t one_shot_spi_timer_init (void)
{
	cy_rslt_t rslt = CY_RSLT_SUCCESS;

    /* Initialize the timer object */
    rslt = cyhal_timer_init(&one_shot_spi_timer_obj, NC, NULL);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /* Set the frequency of timer to 1000 Hz  */
    rslt = cyhal_timer_set_frequency(&one_shot_spi_timer_obj, ONE_SHOT_TIMER_FREQ_HZ);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&one_shot_spi_timer_obj, one_shot_spi_timer_isr, NULL);

    cyhal_timer_enable_event(&one_shot_spi_timer_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE, 3, false);

	return rslt;
}

cy_rslt_t one_shot_timer_spi_start (uint32_t delay_ms)
{
	cy_rslt_t rslt = CY_RSLT_SUCCESS;

	/*Delay begins from here*/
	transfer_delay_complete = false;

    /* Reset configuration */
    rslt = cyhal_timer_reset(&one_shot_spi_timer_obj);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /* Apply timer configuration */
    one_shot_spi_timer_cfg.compare_value = delay_ms;
    rslt = cyhal_timer_configure(&one_shot_spi_timer_obj, &one_shot_spi_timer_cfg);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

    /*Enable Timer Compare Interrupt*/
    cyhal_timer_enable_event(&one_shot_spi_timer_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE, 3, true);

    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&one_shot_spi_timer_obj);
    if (rslt != CY_RSLT_SUCCESS)
    {
    	return rslt;
    }

	return rslt;
}

/*Delay ends with this Interrupt*/
static void one_shot_spi_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /*Stop the Timer and Disable Timer Compare Interrupt*/
    cyhal_timer_stop(&one_shot_spi_timer_obj);
    cyhal_timer_enable_event(&one_shot_spi_timer_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE, 3, false);

    Cy_SCB_SPI_Enable(mSPI_HW);

    /*Non blocking transfer starts here*/
    Cy_SCB_SPI_Transfer(mSPI_HW, pBuffSPImaster, NULL, SPImaster_count, &mSPI_context);
}

