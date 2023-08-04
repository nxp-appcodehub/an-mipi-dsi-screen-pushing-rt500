/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2019, 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* Compiler includes. */
#if defined(__ICCARM__)
#include <intrinsics.h>
#endif

#include "board.h"
#include "fsl_power.h"
#include "fsl_gpio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#if configUSE_TICKLESS_IDLE == 2
#include "fsl_rtc.h"
#else
#include "fsl_device_registers.h"
#endif

#include "fsl_tickless_rtc.h"


# define APP_BUFFER_COUNT  2
   
#define DSI_OFF
#define DPHY_OFF
#define NON_PDSLEEPCFG3
extern uint32_t SystemCoreClock; /* in Kinetis SDK, this contains the system core clock speed */

#if(APP_BUFFER_COUNT == 1)
volatile int32_t deepsleep_allowed = 0;
#else
volatile int32_t deepsleep_allowed[2] = {0};
#endif

int32_t deepsleep_tickless_mode = 0;
deepsleep_user_resume_t deep_user_resume_func = NULL;

/*
 * LPT timer base address and interrupt number
 */
#if configUSE_TICKLESS_IDLE == 2
extern RTC_Type *vPortGetRtcBase(void);
extern IRQn_Type vPortGetRtcIrqn(void);
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The number of SysTick increments that make up one tick period.
 */
#if configUSE_TICKLESS_IDLE == 2
static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if configUSE_TICKLESS_IDLE == 2
static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The number of LPTIMER increments that make up one tick period.
 */
#if configUSE_TICKLESS_IDLE == 2
static uint32_t ulLPTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The flag of LPTIMER is occurs or not.
 */
#if configUSE_TICKLESS_IDLE == 2
static volatile bool ulLPTimerInterruptFired = false;
#endif /* configUSE_TICKLESS_IDLE */

#if configUSE_TICKLESS_IDLE == 2

void vPortRtcIsr(void)
{
    ulLPTimerInterruptFired = true;
    RTC_ClearStatusFlags(RTC, kRTC_WakeupFlag);
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t ulReloadValue, ulCompleteTickPeriods;
    TickType_t xModifiableIdleTime;
    RTC_Type *pxRtcBase;

    pxRtcBase = vPortGetRtcBase();
    if (pxRtcBase == 0)
        return;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }
    if (xExpectedIdleTime == 0)
        return;
    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods. */
    ulReloadValue = ulLPTimerCountsForOneTick * (xExpectedIdleTime - 1UL);

    /* Stop the RTC and systick momentarily.  The time the RTC and systick is stopped for
    is accounted for as best it can be, but using the tickless mode will
    inevitably result in some tiny drift of the time maintained by the
    kernel with respect to calendar time. */
    RTC_StopTimer(pxRtcBase);
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();
    __DSB();
    __ISB();

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep)
    {
        /* Restart from whatever is left in the count register to complete
        this tick period. */
        SysTick->LOAD = SysTick->VAL;

        /* Restart SysTick. */
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

        /* Reset the reload register to the value required for normal tick
        periods. */
        SysTick->LOAD = ulTimerCountsForOneTick - 1UL;

        /* Re-enable interrupts - see comments above __disable_irq()
        call above. */
        __enable_irq();
    }
    else
    {
        /* Set the new reload value. */
        RTC_SetWakeupCount(pxRtcBase, ulReloadValue);
        /* Enable RTC. */
        RTC_StartTimer(pxRtcBase);

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
		
        configPRE_SLEEP_PROCESSING(xModifiableIdleTime);
        if (xModifiableIdleTime > 0)
        {
            __DSB();
#if(APP_BUFFER_COUNT == 1)
            if (deepsleep_allowed) {
#else
            if ((deepsleep_allowed[0]) && (deepsleep_allowed[1])) {
#endif
#ifdef NON_PDSLEEPCFG3
                uint32_t pd_exclude[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0};
#else
                uint32_t pd_exclude[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
#endif

                if (deepsleep_tickless_mode == 0) {
                    __WFI();
                } else if (deepsleep_tickless_mode == 1) {
//#ifdef GPIO_DEBUG
//                GPIO_PinWrite(GPIO, GPIO_DEBUG_PORT_1, GPIO_DEBUG_PIN_1, 1);
//#endif
                    /* MIPI */
                    pd_exclude[0] = (SYSCTL0_PDSLEEPCFG0_RBBSRAM_PD_MASK | SYSCTL0_PDSLEEPCFG0_RBB_PD_MASK | SYSCTL0_PDSLEEPCFG0_LPOSC_PD_MASK);
#ifdef DSI_OFF
                    pd_exclude[1] = (SYSCTL0_PDSLEEPCFG1_SMARTDMA_SRAM_APD_MASK /*| SYSCTL0_PDSLEEPCFG1_MIPIDSI_SRAM_APD_MASK*/
                                   | SYSCTL0_PDSLEEPCFG1_FLEXSPI0_SRAM_APD_MASK | SYSCTL0_PDSLEEPCFG1_SRAM_SLEEP_MASK
                                   | SYSCTL0_PDSLEEPCFG1_FLEXSPI1_SRAM_APD_MASK /*| SYSCTL0_PDSLEEPCFG1_MIPIDSI_PD_MASK*/);
#else
                    pd_exclude[1] = (SYSCTL0_PDSLEEPCFG1_SMARTDMA_SRAM_APD_MASK | SYSCTL0_PDSLEEPCFG1_MIPIDSI_SRAM_APD_MASK
                                   | SYSCTL0_PDSLEEPCFG1_FLEXSPI0_SRAM_APD_MASK | SYSCTL0_PDSLEEPCFG1_SRAM_SLEEP_MASK
                                   | SYSCTL0_PDSLEEPCFG1_FLEXSPI1_SRAM_APD_MASK | SYSCTL0_PDSLEEPCFG1_MIPIDSI_PD_MASK);
#endif
                    
                    
#ifdef DPHY_OFF
                    DSI_DeinitDphy(MIPI_DSI_HOST);
                    MIPI_DSI_HOST ->DPHY_PD_DPHY = 1U;
#endif
                    POWER_EnterDeepSleep(pd_exclude);
#ifdef DPHY_OFF
                    MIPI_DSI_HOST ->DPHY_PD_DPHY = 0U;
#endif
//                    POWER_EnterSleep();
//#ifdef GPIO_DEBUG
//                GPIO_PinWrite(GPIO, GPIO_DEBUG_PORT_1, GPIO_DEBUG_PIN_1, 0);
//#endif                    
                } else if (deepsleep_tickless_mode == 2) {
                    /* Sensor */
                    pd_exclude[0] = (SYSCTL0_PDSLEEPCFG0_RBBSRAM_PD_MASK | SYSCTL0_PDSLEEPCFG0_RBB_PD_MASK);
                    pd_exclude[1] = (SYSCTL0_PDSLEEPCFG1_FLEXSPI0_SRAM_APD_MASK | SYSCTL0_PDSLEEPCFG1_FLEXSPI1_SRAM_APD_MASK | 
						SYSCTL0_PDSLEEPCFG1_SRAM_SLEEP_MASK | SYSCTL0_PDSLEEPCFG1_MIPIDSI_PD_MASK);					
                    POWER_EnterDeepSleep(pd_exclude);
                }

                if (deep_user_resume_func)
                    deep_user_resume_func();
            }
            else {
                __WFI();
            }
            __ISB();
        }
        configPOST_SLEEP_PROCESSING(xExpectedIdleTime);

        ulLPTimerInterruptFired = false;

        /* Re-enable interrupts - see comments above __disable_irq()
        call above. */
        __enable_irq();
        __NOP();
        if (ulLPTimerInterruptFired)
        {
            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods   = xExpectedIdleTime - 1UL;
            ulLPTimerInterruptFired = false;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
            Work out how long the sleep lasted rounded to complete tick
            periods (not the ulReload value which accounted for part
            ticks). */
            ulCompleteTickPeriods = xExpectedIdleTime - 1 - RTC_GetWakeupCount(pxRtcBase) / ulLPTimerCountsForOneTick;
        }

        /* Stop RTC when CPU waked up then set SysTick->LOAD back to its standard
        value.  The critical section is used to ensure the tick interrupt
        can only execute once in the case that the reload register is near
        zero. */
        RTC_StopTimer(pxRtcBase);

        portENTER_CRITICAL();
        {
            SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
            vTaskStepTick(ulCompleteTickPeriods);
            SysTick->LOAD = ulTimerCountsForOneTick - 1UL;
        }
        portEXIT_CRITICAL();
    }
}

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void vPortSetupTimerInterrupt(void)
{
    /* Calculate the constants required to configure the tick interrupt. */
    ulTimerCountsForOneTick   = (configCPU_CLOCK_HZ / configTICK_RATE_HZ);
    ulLPTimerCountsForOneTick = (configRTC_CLOCK_HZ / configTICK_RATE_HZ);
    if (ulLPTimerCountsForOneTick != 0)
    {
        xMaximumPossibleSuppressedTicks = portMAX_16_BIT_NUMBER / ulLPTimerCountsForOneTick;
    }
    else
    {
        /* ulLPTimerCountsForOneTick is zero, not allowed state */
        while (1)
            ;
    }
    NVIC_EnableIRQ(vPortGetRtcIrqn());

    /* Configure SysTick to interrupt at the requested rate. */
    SysTick->LOAD = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}
#endif /* configUSE_TICKLESS_IDLE */
