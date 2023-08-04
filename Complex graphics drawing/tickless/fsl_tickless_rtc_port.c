/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "FreeRTOS.h"

#if configUSE_TICKLESS_IDLE == 2
#include "fsl_common.h"
#include "fsl_rtc.h"

/*!
 * @brief Fuction of LPT timer.
 *
 * This function to return LPT timer base address
 */

RTC_Type *vPortGetRtcBase(void)
{
    return RTC;
}

/*!
 * @brief Fuction of LPT timer.
 *
 * This function to return LPT timer interrupt number
 */

IRQn_Type vPortGetRtcIrqn(void)
{
    return RTC_IRQn;
}

/*!
 * @brief Interrupt service fuction of LPT timer.
 *
 * This function to call vPortRtcIsr
 */
void RTC_IRQHandler(void)
{
    vPortRtcIsr();
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

#endif
