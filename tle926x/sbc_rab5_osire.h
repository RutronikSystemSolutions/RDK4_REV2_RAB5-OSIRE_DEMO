/*
 * sbc_rab5_osire.h
 *
 *  Created on: 2024-06-10
 *      Author: GDR
 */

#ifndef TLE926X_SBC_RAB5_OSIRE_H_
#define TLE926X_SBC_RAB5_OSIRE_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "TLE926x.h"

void sbc_rab5_osire_init(void);
void sbc_rab5_osire_spi_en(void);
void sbc_rab5_osire_spi_dis(void);
void sbc_rab5_osire_check_wdt(void);

#endif /* TLE926X_SBC_RAB5_OSIRE_H_ */
