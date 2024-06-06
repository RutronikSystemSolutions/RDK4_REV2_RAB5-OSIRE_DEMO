/*
 * demoControl.h
 *
 *  Created on: 07.06.2023
 *      Author: EDE
 */

#ifndef OSIRE_SOURCES_DEMOS_DEMOCONTROL_INC_DEMOCONTROL_H_
#define OSIRE_SOURCES_DEMOS_DEMOCONTROL_INC_DEMOCONTROL_H_



#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"



#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 *
 */
typedef enum
{
  MINIMAL_RGBI,
  XYZ_TO_FLASH,
  COLOR_CORRECTION,
  RUNNING_LIGHT,
  UPDATE_RATE_EXAMPLE,
  UART_MODE
} demoState_t;

/**
 * @fn void reset_modi(void)
 * @brief
 *
 */
void reset_modi (void);

/**
 * @fn void start_uart_mode(void)
 * @brief
 *
 */
void start_uart_mode (void);
/**
 * @fn void start_minimal_rbgi_mode(void)
 * @brief
 *
 */
void start_minimal_rbgi_mode (void);

/**
 * @fn void start_colorCorrection_mode(void)
 * @brief
 *
 */
void start_colorCorrection_mode (void);

/**
 * @fn bool start_runningLight_mode(uint8_t)
 * @brief
 *
 * @param version
 * @return
 */
bool start_runningLight_mode (uint8_t version);

/**
 * @fn void start_update_example_mode(uint8_t, float, float, float)
 * @brief
 *
 * @param modeInt
 * @param cx
 * @param cy
 * @param mcd
 */
void start_update_example_mode (uint8_t modeInt, float cx, float cy, float mcd);

/**
 * @fn void demo_control(void)
 * @brief
 *
 */
void demo_control (void);


#ifdef __cplusplus
}
#endif

#endif /* OSIRE_SOURCES_DEMOS_DEMOCONTROL_INC_DEMOCONTROL_H_ */
