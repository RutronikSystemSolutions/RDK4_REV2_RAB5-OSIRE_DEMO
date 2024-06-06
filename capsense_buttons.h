/*
 * capsense_buttons.h
 *
 *  Created on: 2024-02-07
 *      Author: GDR
 */

#ifndef CAPSENSE_BUTTONS_H_
#define CAPSENSE_BUTTONS_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg_capsense.h"

#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)

typedef struct capsense_data
{
    _Bool csb1_status;
    _Bool csb3_status;
}capsense_data_t;

extern capsense_data_t cbuttons;

void initialize_capsense(void);
void process_touch(void);

#endif /* CAPSENSE_BUTTONS_H_ */
