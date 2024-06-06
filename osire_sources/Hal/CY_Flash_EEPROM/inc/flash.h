/*
 * flash.h
 *
 *  Created on: 08.06.2023
 *      Author: EDE
 */

#ifndef OSIRE_SOURCES_HAL_CY_FLASH_EEPROM_INC_FLASH_H_
#define OSIRE_SOURCES_HAL_CY_FLASH_EEPROM_INC_FLASH_H_



#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_em_eeprom.h"


#ifdef __cplusplus
extern "C" {
#endif

extern cy_stc_eeprom_context_t em_eeprom_context;
extern cy_stc_eeprom_config_t em_eeprom_config;

cy_en_em_eeprom_status_t hal_init_flash(void);
cy_en_em_eeprom_status_t hal_erase_led_xyz_data_from_flash (void);
cy_en_em_eeprom_status_t hal_write_single_led_xyz_struct_to_flash (uint16_t ledIndex,const uint8_t *p_bufSrc,uint32_t length);
uint32_t hal_get_maximal_number_of_led_xyz_structures (void);
uint32_t hal_get_led_xyz_sturct_address_in_flash (uint16_t ledIndex);
cy_en_em_eeprom_status_t hal_write_to_flash (uint32_t address, const uint8_t *p_bufSrc,uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* OSIRE_SOURCES_HAL_CY_FLASH_EEPROM_INC_FLASH_H_ */
