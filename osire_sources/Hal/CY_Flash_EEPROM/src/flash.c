/*
 * flash.c
 *
 *  Created on: 08.06.2023
 *      Author: EDE
 */


#include <CY_Flash_EEPROM/inc/flash.h>
#include "cy_pdl.h"
#include "cybsp.h"
#include <Feature/ColorCorrection/inc/colorDefinition.h>

#define EM_EEPROM_SIZE              (1u*10240u)
#define BLOCKING_WRITE              (1u)
#define REDUNDANT_COPY              (0u)
#define WEAR_LEVELLING_FACTOR       (1u)
#define SIMPLE_MODE                 (1u)

#define EM_EEPROM_PHYSICAL_SIZE     (CY_EM_EEPROM_GET_PHYSICAL_SIZE(EM_EEPROM_SIZE, SIMPLE_MODE, WEAR_LEVELLING_FACTOR, REDUNDANT_COPY))

#define LED_XYZ_START_ADDRESS  0 //logical Address Start with index 0

cy_stc_eeprom_context_t em_eeprom_context;

/* Emulated EEPROM configuration and context structure. */

cy_stc_eeprom_config_t em_eeprom_config =
{
    .eepromSize         = EM_EEPROM_SIZE,           /* in bytes */
    .blockingWrite      = BLOCKING_WRITE,           /* Blocking writes enabled */
    .redundantCopy      = REDUNDANT_COPY,           /* Redundant copy enabled */
    .wearLevelingFactor = WEAR_LEVELLING_FACTOR,    /* Wear levelling factor of 2 */
    .simpleMode         = SIMPLE_MODE,              /* Simple mode disabled */
};

/* EEPROM storage Emulated EEPROM flash. */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)
const uint8_t em_eeprom_storage[EM_EEPROM_PHYSICAL_SIZE] = {0u};


/**
 * @brief
 *
 * @return cy_en_em_eeprom_status_t
 */
cy_en_em_eeprom_status_t hal_init_flash(void)
{
	cy_en_em_eeprom_status_t em_eeprom_status;
	// Initialize the flash start address in Emulated EEPROM configuration structure
	em_eeprom_config.userFlashStartAddr = (uint32_t) em_eeprom_storage;

	/* Initialize Emulated EEPROM */
	em_eeprom_status = Cy_Em_EEPROM_Init(&em_eeprom_config, &em_eeprom_context);
	if(em_eeprom_status != CY_EM_EEPROM_SUCCESS)
	{
		CY_ASSERT(0);
	}

	return em_eeprom_status;
}

/**
 * @fn cy_en_em_eeprom_status_t hal_erase_led_xyz_data_from_flash(void)
 * @brief
 *
 * @return
 */
cy_en_em_eeprom_status_t hal_erase_led_xyz_data_from_flash (void)
{
	return Cy_Em_EEPROM_Erase(&em_eeprom_context);
}

/**
 * @fn cy_en_em_eeprom_status_t hal_write_single_led_xyz_struct_to_flash(uint16_t, const uint8_t*, uint32_t)
 * @brief
 *
 * @param ledIndex
 * @param p_bufSrc
 * @param length
 * @return
 */
cy_en_em_eeprom_status_t hal_write_single_led_xyz_struct_to_flash (uint16_t ledIndex,const uint8_t *p_bufSrc,uint32_t length)
{
	cy_en_em_eeprom_status_t ret = 0;

  // Check if the length matches the expected one
  if (length != sizeof(DN_RGB_XYZ_t))
    {
      return CY_EM_EEPROM_BAD_PARAM;
    }
  if (ledIndex >= hal_get_maximal_number_of_led_xyz_structures ())
    {
      return CY_EM_EEPROM_BAD_PARAM;
    }
  uint32_t xyzSturctAddr = hal_get_led_xyz_sturct_address_in_flash (ledIndex);

  ret = hal_write_to_flash (xyzSturctAddr, p_bufSrc, length);

  return ret;
}

/**
 * @brief Getter for maximum number of LED xyz structures in flash.
 *
 * @return Maximum number of LED xyz structures in flash.
 */
uint32_t hal_get_maximal_number_of_led_xyz_structures (void)
{
  return (EM_EEPROM_SIZE / sizeof(DN_RGB_XYZ_t));
}

/**
 * @fn uint32_t hal_get_led_xyz_sturct_address_in_flash(uint16_t)
 * @brief Getter for the LED xyz structure address in flash.
 * @param ledIndex  Index of selected LED structure (starts from 0).
 * @return Address of indexed structure, 0 if index is out of range.
 */
uint32_t hal_get_led_xyz_sturct_address_in_flash (uint16_t ledIndex)
{
  if (ledIndex >= hal_get_maximal_number_of_led_xyz_structures ())
    {
      return 0;
    }
  return LED_XYZ_START_ADDRESS + ledIndex * sizeof(DN_RGB_XYZ_t);
}

/**
 * @fn cy_en_em_eeprom_status_t hal_write_to_flash(uint32_t, const uint8_t*, uint32_t)
 * @brief  Writes provided data to selected address in flash.
 *
 * @param address Address in flash where the data should be written.
 * @param p_bufSrc Pointer at source data which should be written.
 * @param length Length of provided data in bytes. Note: must be aligned to
FEATURE_FLS_PF_BLOCK_WRITE_UNIT_SIZE.
 * @return STATUS_SUCCESS if write operation was successful, error code otherwise.
 */
cy_en_em_eeprom_status_t hal_write_to_flash (uint32_t address, const uint8_t *p_bufSrc,uint32_t length)
{
	return Cy_Em_EEPROM_Write(address, (void *)&p_bufSrc, length, &em_eeprom_context);
}
