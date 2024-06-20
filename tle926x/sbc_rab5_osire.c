/*
 * sbc_rab5_osire.c
 *
 *  Created on: 2024-06-10
 *      Author: GDR
 */

#define WDT_FEED_INTERVAL_MS (500U)

#include "sbc_rab5_osire.h"
#include <amsOsram_sources/Hal/CY_SPI/inc/SpiMaster.h>
#include "sys_timer.h"

/*Priority for SBC interrupts*/
#define SBC_IRQ_PRIORITY		1

static void sbc_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/*TLE9262 SBC Interrupt Data*/
cyhal_gpio_callback_data_t sbc_int_data =
{
		.callback = sbc_interrupt_handler,
		.callback_arg = NULL,
};

/*SBC Power Supply Variables*/
sbc_vcc3_on_t vcc3_supp;
sbc_vcc2_on_t vcc2_supp;

uint32_t feed_time = 0;
uint32_t current_time = 0;

_Bool scb_spi_enabled = false;

void sbc_rab5_osire_spi_en(void)
{
	sbc_spi_init();
	scb_spi_enabled = true;
}

void sbc_rab5_osire_spi_dis(void)
{
	sbc_spi_deinit();
	scb_spi_enabled = false;
}

void sbc_rab5_osire_init(void)
{
    cy_rslt_t result;
    SBC_ErrorCode sbc_err;

    /*Enable the interrupts to be able to initialise the SBC*/
    __enable_irq();

    /*Initialize SBC Interrupt Pin*/
    result = cyhal_gpio_init(INT_SBC, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    /*Register callback functions */
    cyhal_gpio_register_callback(INT_SBC, &sbc_int_data);
    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(INT_SBC, CYHAL_GPIO_IRQ_RISE, SBC_IRQ_PRIORITY, true);

    /*SBC Initializations*/
    sbc_err = sbc_init();
    if(sbc_err.flippedBitsMask)
    {
    	CY_ASSERT(0);
    }

    /*Turn ON the 5V Power Supply VCC2 */
    vcc2_supp = VCC2_ON_ALWAYS;
    sbc_err = sbc_switch_vcc2(vcc2_supp);
    if(sbc_err.flippedBitsMask)
    {
    	CY_ASSERT(0);
    }

    /*Turn ON the 3.3V Power Supply VCC3 for the Arduino Shield(s) */
    vcc3_supp = VCC3_ENABLED;
    sbc_err = sbc_switch_vcc3(vcc3_supp);
    if(sbc_err.flippedBitsMask)
    {
    	CY_ASSERT(0);
    }

    /*Enter Normal Mode*/
    sbc_err = sbc_mode_normal();
    if(sbc_err.flippedBitsMask)
    {
    	CY_ASSERT(0);
    }

    /*SBC Watchdog Configuration*/
    sbc_err = sbc_configure_watchdog(TIME_OUT_WD, NO_WD_AFTER_CAN_LIN_WAKE, WD_1000MS);
    if(sbc_err.flippedBitsMask)
    {
    	CY_ASSERT(0);
    }

    /*Disable the SBC SPI and interrupts for further initialisations*/
	sbc_spi_deinit();
	__disable_irq();
}

void sbc_rab5_osire_check_wdt(void)
{
	current_time = get_system_time_ms();

	if(current_time >= (feed_time + WDT_FEED_INTERVAL_MS))
	{
		CY_deinit_SPI_Master();

		sbc_rab5_osire_spi_en();
     	/*Feed the watchdog*/
    	sbc_wd_trigger();
		sbc_rab5_osire_spi_dis();

		CY_init_SPI_Master();

		feed_time = get_system_time_ms();
	}
}

/* SBC Interrupt handler callback function */
static void sbc_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    if(scb_spi_enabled)
    {
        SBC_ISR();
    }
}


