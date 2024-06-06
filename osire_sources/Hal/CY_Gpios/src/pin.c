

#include <CY_Gpios/inc/pin.h>
#include "cybsp.h"
#include "cy_pdl.h"
#include "cyhal.h"


/*****************************************************************************/
/*****************************************************************************/
void hal_init_pin (void)
{

}

/*****************************************************************************/
/*****************************************************************************/
void set_led_CS_Slave (uint8_t val) //0 = low, 1 = high,
{
	Cy_GPIO_Write(ARD_IO2_PORT, ARD_IO2_PIN, val);
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t hal_check_SW2 (void)
{
    return(false);
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t hal_check_SW3 (void)
{
    return(false);
}

/*****************************************************************************/
/*****************************************************************************/
//---------------- DEBUG LEDs--------------------------------//
void set_led_red (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
  if (val == 0) //Set high = OFF
    {
	Cy_GPIO_Set(USER_LED_RED_PORT,USER_LED_RED_PIN);
    }
  else if (val == 1) //Set low = ON
    {
	Cy_GPIO_Clr(USER_LED_RED_PORT,USER_LED_RED_PIN);
    }
  else if (val == 2)
    {
	Cy_GPIO_Inv(USER_LED_RED_PORT, USER_LED_RED_PIN);
    }
}

/*****************************************************************************/
/*****************************************************************************/
void set_led_green(uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
	if (val == 0) //Set high = OFF
			{
		Cy_GPIO_Set(USER_LED_GREEN_PORT, USER_LED_GREEN_PIN);
	} else if (val == 1) //Set low = ON
			{
		Cy_GPIO_Clr(USER_LED_GREEN_PORT, USER_LED_GREEN_PIN);
	} else if (val == 2)
	{
		//PINS_GPIO_TogglePins (LED_GREEN_PORT, (1 << LED_GREEN_PIN));
		Cy_GPIO_Inv(USER_LED_GREEN_PORT, USER_LED_GREEN_PIN);
	}
}

/*****************************************************************************/
/*****************************************************************************/
void set_led_blue (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
	  if (val == 0) //Set high = OFF
	    {
		Cy_GPIO_Set(USER_LED_BLUE_PORT,USER_LED_BLUE_PIN);
	    }
	  else if (val == 1) //Set low = ON
	    {
		Cy_GPIO_Clr(USER_LED_BLUE_PORT,USER_LED_BLUE_PIN);
	    }
	  else if (val == 2)
	    {
		Cy_GPIO_Inv(USER_LED_BLUE_PORT, USER_LED_BLUE_PIN);
	    }
}
/**
 * @fn void set_ext_pull_up_valid(void)
 * @brief //set to low --> pull up is on
 * 18.10.23 Andreas Heder
 */
void set_ext_pull_up_valid (void)
{

}
void set_ext_pull_up_invalid (void)
{

}



/*****************************************************************************/
/*****************************************************************************/
//---------------- DEBUG Pins --------------------------------//
void set_debug_1 (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
//  if (val == 1) //Set high
//    {
//      PINS_GPIO_SetPins (DEBUG_1_PORT, (1 << DEBUG_1_PIN));
//    }
//  else if (val == 0) //Set low
//    {
//      PINS_GPIO_ClearPins (DEBUG_1_PORT, (1 << DEBUG_1_PIN));
//    }
//  else if (val == 2)
//    {
//      PINS_GPIO_TogglePins (DEBUG_1_PORT, (1 << DEBUG_1_PIN));
//    }
}

/*****************************************************************************/
/*****************************************************************************/
void set_debug_2 (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
//  if (val == 1) //Set high
//    {
//      PINS_GPIO_SetPins (DEBUG_2_PORT, (1 << DEBUG_2_PIN));
//    }
//  else if (val == 0) //Set low
//    {
//      PINS_GPIO_ClearPins (DEBUG_2_PORT, (1 << DEBUG_2_PIN));
//    }
//  else if (val == 2)
//    {
//      PINS_GPIO_TogglePins (DEBUG_2_PORT, (1 << DEBUG_2_PIN));
//    }
}

/*****************************************************************************/
/*****************************************************************************/
void set_debug_3 (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
//  if (val == 1) //Set high
//    {
//      PINS_GPIO_SetPins (DEBUG_3_PORT, (1 << DEBUG_3_PIN));
//    }
//  else if (val == 0) //Set low
//    {
//      PINS_GPIO_ClearPins (DEBUG_3_PORT, (1 << DEBUG_3_PIN));
//    }
//  else if (val == 2)
//    {
//      PINS_GPIO_TogglePins (DEBUG_3_PORT, (1 << DEBUG_3_PIN));
//    }
}

/*****************************************************************************/
/*****************************************************************************/
void set_debug_4 (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
//  if (val == 1) //Set high
//    {
//      PINS_GPIO_SetPins (DEBUG_4_PORT, (1 << DEBUG_4_PIN));
//    }
//  else if (val == 0) //Set low
//    {
//      PINS_GPIO_ClearPins (DEBUG_4_PORT, (1 << DEBUG_4_PIN));
//    }
//  else if (val == 2)
//    {
//      PINS_GPIO_TogglePins (DEBUG_4_PORT, (1 << DEBUG_4_PIN));
//    }
}

/*****************************************************************************/
/*****************************************************************************/
void set_debug_5 (uint8_t val) //0 = OFF, 1 = ON, 2 = Toggle
{
//  if (val == 1) //Set high
//    {
//      PINS_GPIO_SetPins (DEBUG_5_PORT, (1 << DEBUG_5_PIN));
//    }
//  else if (val == 0) //Set low
//    {
//      PINS_GPIO_ClearPins (DEBUG_5_PORT, (1 << DEBUG_5_PIN));
//    }
//  else if (val == 2)
//    {
//      PINS_GPIO_TogglePins (DEBUG_5_PORT, (1 << DEBUG_5_PIN));
//    }
}
