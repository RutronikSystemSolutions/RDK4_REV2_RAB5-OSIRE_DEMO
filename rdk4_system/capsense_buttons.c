/*
 * capsense_buttons.c
 *
 *  Created on: 2024-02-07
 *      Author: GDR
 */

#include "capsense_buttons.h"

static void capsense_msc1_isr(void);

capsense_data_t cbuttons =
		{
				.csb1_status = false,
				.csb3_status = false
		};

/*******************************************************************************
* Function Name: capsense_msc1_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense MSC0 block.
*
*******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configures the CapSense
*  interrupt.
*
*******************************************************************************/
void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC1_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
void process_touch(void)
{
    uint32_t button1_status;
    uint32_t button3_status;
    static _Bool capsense_start = false;


    /* Start the first scan */
    if(!capsense_start)
    {
    	capsense_start = true;
    	Cy_CapSense_ScanAllSlots(&cy_capsense_context);
    	return;
    }

	if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
    {
        /* Process all widgets */
        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

	    /* Get button 1 status */
	    button1_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_CSB1_WDGT_ID,CY_CAPSENSE_CSB1_SNS0_ID,&cy_capsense_context);
	    if(button1_status)
	    {
	    	cbuttons.csb1_status = true;
	    }
	    else
	    {
	    	cbuttons.csb1_status = false;
	    }

	    /* Get button 3 status */
	    button3_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_CSB3_WDGT_ID,CY_CAPSENSE_CSB3_SNS0_ID,&cy_capsense_context);
	    if(button3_status)
	    {
	    	cbuttons.csb3_status = true;
	    }
	    else
	    {
	    	cbuttons.csb3_status = false;
	    }

        /* Initiate next scan */
        Cy_CapSense_ScanAllSlots(&cy_capsense_context);
    }
}
