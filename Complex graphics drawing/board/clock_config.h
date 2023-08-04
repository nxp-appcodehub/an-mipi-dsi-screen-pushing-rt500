/*
 * Copyright 2019-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_SYSOSC_SETTLING_US 100U   /*!< Board System oscillator settling time in us */
#define BOARD_XTAL32K_CLK_HZ 32768U     /*!< Board xtal32K frequency in Hz */
#if (BOARD_EVK_MIMXRT595 == 1)
#define BOARD_XTAL_SYS_CLK_HZ 24000000U /*!< Board xtal frequency in Hz */
#elif (BOARD_CUSTOMER_HB1 == 1)
#define BOARD_XTAL_SYS_CLK_HZ 16000000U /*!< Board xtal frequency in Hz */
#endif
/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes default configuration of clocks.
 *
 */
void BOARD_InitBootClocks(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockRUN configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 198000000U /*!< Core clock frequency: 198000000Hz */

/*******************************************************************************
 * API for BOARD_BootClockRUN configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockRUN(int8_t xip);
void BOARD_BootClockRUN_SysPLL(int8_t xip);
void BOARD_BootClockRUN_NoPLL(int8_t xip);
void BOARD_BootClockRUN_EXT_XTAL(int8_t xip);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _CLOCK_CONFIG_H_ */
