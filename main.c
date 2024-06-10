/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK4 RAB5-OSIRE Demonstration
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2024-02-13
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include <CY_System/inc/initSystem.h>
#include "Demos/DemoControl/inc/demoControl.h"
#include <UartProtocol/inc/uartProtocolHandler.h>
#include <Hal/Button/inc/button.h>
#include "sbc_rab5_osire.h"
#include <osireDevice.h>

int main(void)
{
//	ospInitRsp_t rsp;
//	osirePwmData_t dataPwm;
//
//	dataPwm.data.bit.blue_curr = 0;
//	dataPwm.data.bit.green_curr = 0;
//	dataPwm.data.bit.red_curr = 0;
//	dataPwm.data.bit.blue_pwm  = 0x3fff;
//	dataPwm.data.bit.green_pwm  = 0x3fff;
//	dataPwm.data.bit.red_pwm  = 0x3fff;

	/*Hardware initialisation*/
	init_sys();

//	osp_init_bidir (1, &rsp);
//	osp_go_active(0);
//  osp_osire_set_pwm (1, dataPwm);
//	osp_osire_set_pwm (2, dataPwm);
//	osp_said_set_curr (3);
//	osp_said_set_pwm (3, dataPwm);

	/*Main loop*/
	for (;;)
	{
		button_polling();
		demo_control();
		uart_receive_new_msg();
		sbc_rab5_osire_check_wdt();
//		Cy_SysLib_Delay(100);
//		osp_said_set_pwm (3, dataPwm);
//		osp_said_set_pwm (3, dataPwm);
	}
}

/* [] END OF FILE */
