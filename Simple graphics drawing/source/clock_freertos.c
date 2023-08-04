/*
 * Copyright 2019, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"

#include "board.h"
#include "fsl_common.h"
#include "fsl_power.h"
#include "fsl_inputmux.h"
#include "fsl_pint.h"
#include "fsl_usart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "pmic_support.h"
#include "fsl_pca9420.h"
#include "fsl_clock.h"
#include "fsl_iopctl.h"

#include "vglite_support.h"
#include "vglite_window.h"

/*-----------------------------------------------------------*/
#include "vg_lite.h"
#include "elm.h"

#include "clock_analog.h"
#include "hour_needle.h"
#include "minute_needle.h"

#include "display_support.h"
#include "fsl_fbdev.h"

#include "FreeRTOS.h"

//#define ENABLE_PSRAM  1

//#define CONTROL_FRAME_RATE 1
//#define NON_DEEPSLEEP_WAKEUP 1
#if (configUSE_TICKLESS_IDLE == 2)
#include "fsl_rtc.h"
#include "fsl_tickless_rtc.h"
#endif

#include "fsl_utick.h"
#if (CONTROL_FRAME_RATE == 1)
#define UTICK_TIME_1S (1000000UL)
static void UTickCallback(void);
SemaphoreHandle_t xSemaphore_Timer;
#endif
#define APP_BUFFER_COUNT 2
#define DEFAULT_SIZE     256.0f;

typedef struct elm_render_buffer
{
    ElmBuffer handle;
    vg_lite_buffer_t *buffer;
} ElmRenderBuffer;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void vglite_task(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static vg_lite_display_t display;
static vg_lite_window_t window;

static vg_lite_matrix_t matrix;

static ElmHandle analogClockHandle  = ELM_NULL_HANDLE;
static ElmHandle hourNeedleHandle   = ELM_NULL_HANDLE;
static ElmHandle minuteNeedleHandle = ELM_NULL_HANDLE;
static ElmRenderBuffer elmFB[APP_BUFFER_COUNT];

extern unsigned int ClockAnalog_evo_len;
extern unsigned char ClockAnalog_evo[];

extern unsigned int HourNeedle_evo_len;
extern unsigned char HourNeedle_evo[];

extern unsigned int MinuteNeedle_evo_len;
extern unsigned char MinuteNeedle_evo[];

extern clock_sys_pll_config_t g_sysPllConfig_BOARD_BootClockRUN;

AT_QUICKACCESS_SECTION_CODE(void BOARD_SetFlexspiClock(FLEXSPI_Type *qspi, uint32_t flexspiClockSrc, uint32_t divider));

void BOARD_ConfigPMICModes(pca9420_modecfg_t *cfg, uint32_t num)
{
    assert(cfg);

    /* Deep sleep core voltage decreased to 0.6V. */
    cfg[1].sw1OutVolt = kPCA9420_Sw1OutVolt0V600;

    /* Configuration PMIC mode to align with power lib like below:
     *  0b00    run mode, no special.
     *  0b01    deep sleep mode, no special.
     *  0b10    deep powerdown mode, vddcore off.
     *  0b11    full deep powerdown mode vdd1v8 and vddcore off. */

    /* Mode 2: VDDCORE off. */
    cfg[2].enableSw1Out = false;

    /* Mode 3: VDDCORE, VDD1V8 and VDDIO off. */
    cfg[3].enableSw1Out  = false;
    cfg[3].enableSw2Out  = false;
    cfg[3].enableLdo2Out = false;
}


AT_QUICKACCESS_SECTION_CODE(void shutoff_clocks(int8_t xip))
{
    /* CLKCTL0 PSCCTL0 */
    CLOCK_DisableClock(kCLOCK_Dsp);
    CLOCK_DisableClock(kCLOCK_RomCtrlr);
//    CLOCK_DisableClock(kCLOCK_AxiSwitch);
//    CLOCK_DisableClock(kCLOCK_AxiCtrl);
    CLOCK_DisableClock(kCLOCK_PowerQuad);
    CLOCK_DisableClock(kCLOCK_Casper);
    CLOCK_DisableClock(kCLOCK_HashCrypt);
    CLOCK_DisableClock(kCLOCK_Puf);
    CLOCK_DisableClock(kCLOCK_Rng);
    if (xip == 0) {
        CLOCK_DisableClock(kCLOCK_Flexspi0);
    }
    CLOCK_DisableClock(kCLOCK_OtpCtrl);
	
#if (ENABLE_PSRAM == 0)
    CLOCK_DisableClock(kCLOCK_Flexspi1);
#endif
    CLOCK_DisableClock(kCLOCK_UsbhsPhy);
    CLOCK_DisableClock(kCLOCK_UsbhsDevice);
    CLOCK_DisableClock(kCLOCK_UsbhsHost);
    CLOCK_DisableClock(kCLOCK_UsbhsSram);
    CLOCK_DisableClock(kCLOCK_Sct);
//    CLOCK_DisableClock(kCLOCK_Gpu);
    CLOCK_DisableClock(kCLOCK_DisplayCtrl);
//    CLOCK_DisableClock(kCLOCK_MipiDsiCtrl);
//    CLOCK_DisableClock(kCLOCK_Smartdma);

    /* CLKCTL0 PSCCTL2 */
    CLOCK_DisableClock(kCLOCK_Sdio0);
    CLOCK_DisableClock(kCLOCK_Sdio1);
    CLOCK_DisableClock(kCLOCK_Acmp0);
    CLOCK_DisableClock(kCLOCK_Adc0);
    CLOCK_DisableClock(kCLOCK_ShsGpio0);

    /* CLKCTL0 PSCCTL1 */
#if (CONTROL_FRAME_RATE == 0)	
    CLOCK_DisableClock(kCLOCK_Utick0);
#endif
    CLOCK_DisableClock(kCLOCK_Wwdt0);
//    CLOCK_DisableClock(kCLOCK_Pmc);

    /* CLKCTL1 PSCCTL0 */
    CLOCK_DisableClock(kCLOCK_Flexcomm0);
    CLOCK_DisableClock(kCLOCK_Flexcomm1);
    CLOCK_DisableClock(kCLOCK_Flexcomm2);
    CLOCK_DisableClock(kCLOCK_Flexcomm3);
    CLOCK_DisableClock(kCLOCK_Flexcomm4);
    CLOCK_DisableClock(kCLOCK_Flexcomm5);
    CLOCK_DisableClock(kCLOCK_Flexcomm6);
    CLOCK_DisableClock(kCLOCK_Flexcomm7);
    CLOCK_DisableClock(kCLOCK_Flexcomm8);
    CLOCK_DisableClock(kCLOCK_Flexcomm9);
    CLOCK_DisableClock(kCLOCK_Flexcomm10);
    CLOCK_DisableClock(kCLOCK_Flexcomm11);
    CLOCK_DisableClock(kCLOCK_Flexcomm12);
    CLOCK_DisableClock(kCLOCK_Flexcomm13);
    CLOCK_DisableClock(kCLOCK_Flexcomm14);
    CLOCK_DisableClock(kCLOCK_Flexcomm15);
    CLOCK_DisableClock(kCLOCK_Flexcomm16);
    CLOCK_DisableClock(kCLOCK_Dmic0);
    CLOCK_DisableClock(kCLOCK_OsEventTimer);
    CLOCK_DisableClock(kCLOCK_Flexio);

    /* CLKCTL1 PSCCTL1 */
#ifndef GPIO_DEBUG
    CLOCK_DisableClock(kCLOCK_HsGpio0);
#endif
    CLOCK_DisableClock(kCLOCK_HsGpio1);
    CLOCK_DisableClock(kCLOCK_HsGpio2);
 //   CLOCK_DisableClock(kCLOCK_HsGpio3);
    CLOCK_DisableClock(kCLOCK_HsGpio4);
    CLOCK_DisableClock(kCLOCK_HsGpio5);
    CLOCK_DisableClock(kCLOCK_HsGpio6);
    CLOCK_DisableClock(kCLOCK_HsGpio7);
    CLOCK_DisableClock(kCLOCK_Crc);
    CLOCK_DisableClock(kCLOCK_Dmac0);
    CLOCK_DisableClock(kCLOCK_Dmac1);
    CLOCK_DisableClock(kCLOCK_Mu);
    CLOCK_DisableClock(kCLOCK_Sema);
    CLOCK_DisableClock(kCLOCK_Freqme);

    /* CLKCTL1 PSCCTL2 */
    CLOCK_DisableClock(kCLOCK_Ct32b0);
    CLOCK_DisableClock(kCLOCK_Ct32b1);
    CLOCK_DisableClock(kCLOCK_Ct32b2);
    CLOCK_DisableClock(kCLOCK_Ct32b3);
    CLOCK_DisableClock(kCLOCK_Ct32b4);
//    CLOCK_DisableClock(kCLOCK_Rtc);
    CLOCK_DisableClock(kCLOCK_Mrt0);
    CLOCK_DisableClock(kCLOCK_Wwdt1);
    CLOCK_DisableClock(kCLOCK_I3c0);
    CLOCK_DisableClock(kCLOCK_I3c1);
    CLOCK_DisableClock(kCLOCK_Pint);
    CLOCK_DisableClock(kCLOCK_InputMux);
}


AT_QUICKACCESS_SECTION_CODE(void set_resets(int8_t xip))
{
    /* RSTCTL0 - PRSTCTL0*/
    RESET_SetPeripheralReset(kDSP_RST_SHIFT_RSTn);
//    RESET_SetPeripheralReset(kAXI_SWITCH_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kPOWERQUAD_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kCASPER_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kHASHCRYPT_RST_SHIFT_RSTn);
    if (xip == 0) {
        RESET_SetPeripheralReset(kFLEXSPI0_RST_SHIFT_RSTn);
    }
	
#if (ENABLE_PSRAM == 0)
    RESET_SetPeripheralReset(kFLEXSPI1_RST_SHIFT_RSTn);
#endif

    RESET_SetPeripheralReset(kUSBHS_PHY_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kUSBHS_DEVICE_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kSCT_RST_SHIFT_RSTn);
//    RESET_SetPeripheralReset(kGPU_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kDISP_CTRL_RST_SHIFT_RSTn);
//    RESET_SetPeripheralReset(kMIPI_DSI_CTRL_RST_SHIFT_RSTn);
//    RESET_SetPeripheralReset(kMIPI_DSI_PHY_RST_SHIFT_RSTn);
//    RESET_SetPeripheralReset(kSMART_DMA_RST_SHIFT_RSTn);

    /* RSTCTL0 - PRSTCTL1 */
    RESET_SetPeripheralReset(kSDIO0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kSDIO1_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kACMP0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kADC0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kSHSGPIO0_RST_SHIFT_RSTn);

    /* RSTCTL0 - PRSTCTL2 */	
    RESET_SetPeripheralReset(kUTICK0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kWWDT0_RST_SHIFT_RSTn);

    /* RSTCTL1 - PRSTCTL0 */
    RESET_SetPeripheralReset(kFC0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC1_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC2_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC3_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC4_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC5_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC6_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC7_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC8_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC9_RST_SHIFT_RSTn);	
    RESET_SetPeripheralReset(kFC10_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC11_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC12_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC13_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC14_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC15_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kDMIC_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFC16_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kOSEVENT_TIMER_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kFLEXIO_RST_SHIFT_RSTn);

    /* RSTCTL1 - PRSTCTL1 */
#ifndef GPIO_DEBUG
    RESET_SetPeripheralReset(kHSGPIO0_RST_SHIFT_RSTn);
#endif
    RESET_SetPeripheralReset(kHSGPIO1_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kHSGPIO2_RST_SHIFT_RSTn);
//    RESET_SetPeripheralReset(kHSGPIO3_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kHSGPIO4_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kHSGPIO5_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kHSGPIO6_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kHSGPIO7_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kDMAC0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kDMAC1_RST_SHIFT_RSTn);	
    RESET_SetPeripheralReset(kMU_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kSEMA_RST_SHIFT_RSTn);	
    RESET_SetPeripheralReset(kFREQME_RST_SHIFT_RSTn);

    /* RSTCTL1 - PRSTCTL2 */
    RESET_SetPeripheralReset(kCT32B0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kCT32B1_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kCT32B2_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kCT32B3_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kCT32B4_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kMRT0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kWWDT1_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kI3C0_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kI3C1_RST_SHIFT_RSTn);	
    RESET_SetPeripheralReset(kPINT_RST_SHIFT_RSTn);
    RESET_SetPeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
}

AT_QUICKACCESS_SECTION_CODE(void poweroff(int8_t xip))
{
//    CLKCTL0->SYSTICKFCLKSEL = CLKCTL0_SYSTICKFCLKSEL_SEL(7);
	CLKCTL0->WDT0FCLKSEL = CLKCTL0_WDT0FCLKSEL_SEL(7);
//	  CLKCTL0->SYSPLL0CLKSEL = CLKCTL0_SYSPLL0CLKSEL_SEL(7);
    CLKCTL1->FLEXCOMM[0].FRGCLKSEL = CLKCTL1_FRGCLKSEL_SEL(7);

    /* PDRCFG0 */
    POWER_EnablePD(kPDRUNCFG_LP_VDD_COREREG);
    POWER_EnablePD(kPDRUNCFG_LP_PMCREF);
    POWER_EnablePD(kPDRUNCFG_PD_HVD1V8);
//    POWER_EnablePD(kPDRUNCFG_LP_PORCORE);
    POWER_EnablePD(kPDRUNCFG_LP_LVDCORE);
    POWER_EnablePD(kPDRUNCFG_LP_LVDCORE);
    POWER_EnablePD(kPDRUNCFG_PD_HVDCORE);
    /* RBB/FBB Power Gate is controlled by BIAS API */

//    POWER_EnablePD(kPDRUNCFG_PD_LPOSC);
    POWER_EnablePD(kPDRUNCFG_PD_RBBSRAM);
    /* If FFRO is not used  as main clock source */
//    if (!((CLKCTL0->MAINCLKSELB == 0) && ((CLKCTL0->MAINCLKSELA == 1) || (CLKCTL0->MAINCLKSELA == 3))))
    {
    	/* MIPI Clock source */
//        POWER_EnablePD(kPDRUNCFG_PD_FFRO);
    }    

//    CLKCTL0->SYSPLL0PFD |= CLKCTL0_SYSPLL0PFD_PFD0_CLKGATE_MASK;
//    CLKCTL0->SYSPLL0PFD |= CLKCTL0_SYSPLL0PFD_PFD1_CLKGATE_MASK;
//    CLKCTL0->SYSPLL0PFD |= CLKCTL0_SYSPLL0PFD_PFD2_CLKGATE_MASK;
//    CLKCTL0->SYSPLL0PFD |= CLKCTL0_SYSPLL0PFD_PFD3_CLKGATE_MASK;
//	  CLKCTL0->SYSPLL0CTL0 |= CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK | CLKCTL0_SYSPLL0CTL0_RESET_MASK;
    /* Power down System PLL*/
//    SYSCTL0->PDRUNCFG0_SET = SYSCTL0_PDRUNCFG0_SYSPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_SYSPLLANA_PD_MASK;

    CLKCTL1->AUDIOPLL0PFD |= (CLKCTL1_AUDIOPLL0PFD_PFD0_CLKGATE_MASK << (8 * kCLOCK_Pfd0));
    /* Set Audio PLL Reset & HOLDRINGOFF_ENA */
    CLKCTL1->AUDIOPLL0CTL0 |= CLKCTL1_AUDIOPLL0CTL0_HOLDRINGOFF_ENA_MASK | CLKCTL1_AUDIOPLL0CTL0_RESET_MASK;
    /* Power down Audio PLL */
    SYSCTL0->PDRUNCFG0_SET = SYSCTL0_PDRUNCFG0_AUDPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_AUDPLLANA_PD_MASK;

//    if (!(((CLKCTL0->MAINCLKSELB == 0) && ((CLKCTL0->MAINCLKSELA == 2))) || (CLKCTL0->MAINCLKSELB == 1)))
//    {
//        POWER_EnablePD(kPDRUNCFG_PD_SYSXTAL);
//    }

    POWER_EnablePD(kPDRUNCFG_PD_ADC);
    POWER_EnablePD(kPDRUNCFG_LP_ADC);
    POWER_EnablePD(kPDRUNCFG_PD_ADC_TEMPSNS);
    POWER_EnablePD(kPDRUNCFG_PD_PMC_TEMPSNS);
    POWER_EnablePD(kPDRUNCFG_PD_ACMP);
    if(xip == 0)
    {
        POWER_EnablePD(kPDRUNCFG_LP_HSPAD_FSPI0_VDET);
        POWER_EnablePD(kPDRUNCFG_PD_HSPAD_FSPI0_REF);
    }
    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_SDIO0_VDET);
    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_SDIO0_REF);
	
#if (ENABLE_PSRAM == 0)
    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_FSPI1_VDET);
    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_FSPI1_REF);
#endif

    /* PDRCFG1 */
    POWER_EnablePD(kPDRUNCFG_PPD_PQ_SRAM);
    if(xip == 0)
    {
        POWER_EnablePD(kPDRUNCFG_APD_FLEXSPI0_SRAM);
        POWER_EnablePD(kPDRUNCFG_PPD_FLEXSPI0_SRAM);
    }
#if (ENABLE_PSRAM == 0)
    POWER_EnablePD(kPDRUNCFG_APD_FLEXSPI1_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_FLEXSPI1_SRAM);
#endif
    POWER_EnablePD(kPDRUNCFG_APD_USBHS_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_USBHS_SRAM);
    POWER_EnablePD(kPDRUNCFG_APD_USDHC0_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_USDHC0_SRAM);
    POWER_EnablePD(kPDRUNCFG_APD_USDHC1_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_USDHC1_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_CASPER_SRAM);
//    POWER_EnablePD(kPDRUNCFG_APD_GPU_SRAM);
//    POWER_EnablePD(kPDRUNCFG_PPD_GPU_SRAM);
//    POWER_EnablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
//    POWER_EnablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
//    POWER_EnablePD(kPDRUNCFG_APD_MIPIDSI_SRAM);
//    POWER_EnablePD(kPDRUNCFG_PPD_MIPIDSI_SRAM);
    POWER_EnablePD(kPDRUNCFG_APD_DCNANO_SRAM);
    POWER_EnablePD(kPDRUNCFG_PPD_DCNANO_SRAM);
    POWER_EnablePD(kPDRUNCFG_PD_DSP);
//    POWER_EnablePD(kPDRUNCFG_PD_MIPIDSI);
    POWER_EnablePD(kPDRUNCFG_PD_OTP);
    POWER_EnablePD(kPDRUNCFG_PD_ROM);
    POWER_EnablePD(kPDRUNCFG_LP_HSPAD_SDIO1_VDET);
    POWER_EnablePD(kPDRUNCFG_PD_HSPAD_SDIO1_REF);
//  POWER_EnablePD(kPDRUNCFG_SRAM_SLEEP);

    POWER_ApplyPD();
}

AT_QUICKACCESS_SECTION_CODE(void poweroff_memory(int32_t array, int32_t peripheral))
{
    SYSCTL0->PDRUNCFG2_SET = array;
    SYSCTL0->PDRUNCFG3_SET = peripheral;

    POWER_ApplyPD();
}

AT_QUICKACCESS_SECTION_CODE(void recoverdefaultpad(int8_t xip))
{
    int8_t pin, port;
    /* Pin used */
    /*  */
	/* PSRAM Reset - P0_28, extenal pull-up (VDDIO_1) */
	/* FLEXSPI CLK/DATA0/DATA1/DATA2/DATA3/DQS/SS0_B - P4_11/P4_12/P4_13/P4_14/P4_15/P4_16/P4_18 (VDDIO_1) */
	/* FLEXSPI DATA4/DATA5/DATA6/DATA7 - P5_15/P5_16/P5_17/P5_18 (VDDIO_1) */
	/* MIPI Display - P3_15 PWR (VDDIO_4), P0_12 Backlight (VDDIO_0), P3_21 Reset(VDDIO_4), P3_18 TE (VDDIO_4) 
       P3_19 LCM_CTP_INT(VDDIO_4), P4_4 LCM_CTP_RST(VDDIO0_0), P0_29(VDDIO_1), P0_30(VDDIO_1) - FC4 TP */
    if(xip == 0) {
        for(port = 0; port <= 5; port++){
            for(pin = 0; pin < 32; pin++) {				
                if(IOPCTL->PIO[port][pin] == 0)
                    continue;

                if (port == 3 && pin == 18)
                    continue;
#if (ENABLE_PSRAM == 1)
                if((port == 4) && ((pin == 11) || (pin == 12) || (pin == 13) 
                    || (pin == 14) || (pin == 15) || (pin == 16) || (pin == 18))) {
                    continue;
                }

                if ((port == 5) && ((pin == 15) || (pin == 16) || (pin ==17) || (pin == 18))) {
                    continue;
                }
#endif
                IOPCTL_PinMuxSet(IOPCTL, port, pin, 0);
            }
        }
    } else {
        for(pin = 0; pin < 32; pin++) {
            if(IOPCTL->PIO[0][pin] == 0)
                continue;
            IOPCTL_PinMuxSet(IOPCTL, 0, pin, 0);
        }

        for(pin = 0; pin < 16; pin++) {
            if(IOPCTL->PIO[1][pin] == 0)
                continue;
            IOPCTL_PinMuxSet(IOPCTL, 1, pin, 0);
        }
        IOPCTL_PinMuxSet(IOPCTL, 1, 30, 0);
        IOPCTL_PinMuxSet(IOPCTL, 1, 31, 0);
		
        for(pin = 0; pin < 32; pin++) {
            if(IOPCTL->PIO[2][pin] == 0)
                continue;

            IOPCTL_PinMuxSet(IOPCTL, 2, pin, 0);
    	}

        for(pin = 0; pin < 32; pin++) {
            if(IOPCTL->PIO[3][pin] == 0)
                continue;

            if (pin == 18)
                continue;

            IOPCTL_PinMuxSet(IOPCTL, 3, pin, 0);
    	}

        for(pin = 0; pin < 32; pin++) {
            if(IOPCTL->PIO[4][pin] == 0)
                continue;
#if (ENABLE_PSRAM == 1)
            if ((pin == 11) || (pin == 12) || (pin == 13) 
                || (pin == 14) || (pin == 15) || (pin == 16) || (pin == 18)) {
                continue;
            }
#endif
            IOPCTL_PinMuxSet(IOPCTL, 4, pin, 0);
    	}
		
        for(pin = 0; pin < 32; pin++) {
            if(IOPCTL->PIO[5][pin] == 0)
                continue;
#if (ENABLE_PSRAM == 1)
            if ((pin == 15) || (pin == 16) || (pin ==17) || (pin == 18)) {
                continue;
            }
#endif
            IOPCTL_PinMuxSet(IOPCTL, 5, pin, 0);
    	}
    }

    IOPCTL->FC15_I2C_SCL = 0;
    IOPCTL->FC15_I2C_SDA = 0;
}

extern clock_audio_pll_config_t g_audioPllConfig_BOARD_BootClockRUN;
extern volatile int32_t deepsleep_allowed;
extern int32_t deepsleep_tickless_mode;

int32_t wakeup_flag = 0;

void deepsleep_tickless_mode1_wakeup(void)
{
    status_t status;

    wakeup_flag = 1;
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    pca9420_modecfg_t pca9420ModeCfg[4];
    status_t status;
    uint32_t i;
    void *p = (void *)main;
    uint32_t bbMode;
    volatile int32_t delay = 10000;
    int8_t xip_execution = 0;
    pca9420_sw1_out_t sw1_out_volt;

    if((uint32_t)p < 0x8000000) {
        xip_execution = 0;
    } else {
        xip_execution = 1;
    }

    /* BE CAUTIOUS TO SET CORRECT VOLTAGE RANGE ACCORDING TO YOUR BOARD/APPLICATION. PAD SUPPLY BEYOND THE RANGE DO
       HARM TO THE SILICON. */
    power_pad_vrange_t vrange = {.Vdde0Range = kPadVol_171_198,
                                 .Vdde1Range = kPadVol_171_198,
                                 /* SD0 voltage is switchable, but in power_manager demo, it's fixed 3.3V. */
                                 .Vdde2Range = kPadVol_171_198,
                                 .Vdde3Range = kPadVol_300_360,
                                 .Vdde4Range = kPadVol_171_198};
    BOARD_InitPins();
#if (CONTROL_FRAME_RATE == 1)
    xSemaphore_Timer = xSemaphoreCreateBinary();
    if (xSemaphore_Timer == NULL)
    {
        PRINTF("xSemaphore_Timer creation failed.\r\n");
        vTaskSuspend(NULL);
    }
#endif

#if (ENABLE_PSRAM == 1)
    BOARD_InitPsRamPins();
#endif

#if (DEMO_PANEL_RM67162 == DEMO_PANEL)
    GPIO_PortInit(GPIO, BOARD_MIPI_TE_PORT);
#endif
    
#if (DEMO_PANEL_TFT_PROTO_5 == DEMO_PANEL)
    BOARD_InitFlexIOPanelPins();
	
    GPIO_PortInit(GPIO, BOARD_SSD1963_RST_PORT);
    GPIO_PortInit(GPIO, BOARD_SSD1963_CS_PORT);
    GPIO_PortInit(GPIO, BOARD_SSD1963_RS_PORT);
#else
    BOARD_InitMipiPanelPins();
	
    GPIO_PortInit(GPIO, BOARD_MIPI_POWER_PORT);
    GPIO_PortInit(GPIO, BOARD_MIPI_BL_PORT);
    GPIO_PortInit(GPIO, BOARD_MIPI_RST_PORT);
#endif

    POWER_SetDeepSleepClock(kDeepSleepClk_LpOsc);
	
    BOARD_BootClockRUN_NoPLL(xip_execution);
    CLOCK_InitSysPll(&g_sysPllConfig_BOARD_BootClockRUN);

    /* Use GPIO to make sure core is alive */
#ifdef GPIO_DEBUG
    gpio_pin_config_t gpio_config;
		
    gpio_config.outputLogic = 0;
    gpio_config.pinDirection = kGPIO_DigitalOutput;
    CLOCK_EnableClock(kCLOCK_HsGpio0);
    RESET_ClearPeripheralReset(kHSGPIO0_RST_SHIFT_RSTn);
    GPIO_PinInit(GPIO, GPIO_DEBUG_PORT_0, GPIO_DEBUG_PIN_0, &gpio_config);
    GPIO_PinInit(GPIO, GPIO_DEBUG_PORT_1, GPIO_DEBUG_PIN_1, &gpio_config);
#endif

    BOARD_InitDebugConsole();
    PRINTF("Booting GPU\n");
    
    if(xip_execution == 0) {
        PRINTF("Power Manager Demo RAM.\r\n");
    } else {
        PRINTF("Power Manager Demo FLASH.\r\n");
    }
    PRINTF("CPU Frequency %d\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Main Clock %d\n", CLOCK_GetFreq(kCLOCK_BusClk));
    PRINTF("FLEXSPI0 FREQ %d\n", CLOCK_GetFreq(kCLOCK_Flexspi0Clk));
    PRINTF("CLKCTL0 0x%x, 0x%x, 0x%x\n", CLKCTL0->PSCCTL0, CLKCTL0->PSCCTL1, CLKCTL0->PSCCTL2);
    PRINTF("CLKCTL1 0x%x, 0x%x, 0x%x\n", CLKCTL1->PSCCTL0, CLKCTL1->PSCCTL1, CLKCTL1->PSCCTL2);
    PRINTF("PDRUNCFG 0x%x, 0x%x, 0x%x, 0x%x\n", SYSCTL0->PDRUNCFG0, SYSCTL0->PDRUNCFG1, SYSCTL0->PDRUNCFG2, SYSCTL0->PDRUNCFG3);

    sw1_out_volt = BOARD_CalcVoltLevel(CLOCK_GetFreq(kCLOCK_CoreSysClk), 0U, CLOCK_GetFreq(kCLOCK_CoreSysClk)/*CLOCK_GetFreq(kCLOCK_GpuClk)*/);
    PRINTF("sw1_out_volt = %d",sw1_out_volt);
    PRINTF("PMC0 0x%x, 0x%x, 0x%x, 0x%x\n", PMC->CTRL, PMC->RUNCTRL, PMC->SLEEPCTRL, PMC->LVDCORECTRL);
    PRINTF("PMC1 0x%x, 0x%x\n", PMC->AUTOWKUP, PMC->PMICCFG);

    PMC->LVDCORECTRL = 0;
//    PMC->BBCTRL = 0;
    POWER_ApplyPD();

    /* PMIC PCA9420 */
    BOARD_InitPmic();
    for (i = 0; i < ARRAY_SIZE(pca9420ModeCfg); i++)
    {
        PCA9420_GetDefaultModeConfig(&pca9420ModeCfg[i]);
    }
    BOARD_ConfigPMICModes(pca9420ModeCfg, ARRAY_SIZE(pca9420ModeCfg));
    
    PRINTF("Active Core Voltage %d mV\n", sw1_out_volt * 25 + 500);
    PRINTF("Deep Sleep Core Voltage %d mV\n", pca9420ModeCfg[1].sw1OutVolt * 25 + 500);

    PCA9420_WriteModeConfigs(&pca9420Handle, kPCA9420_Mode0, &pca9420ModeCfg[0], ARRAY_SIZE(pca9420ModeCfg));
    BOARD_SetPmicVoltageForFreq(CLOCK_GetFreq(kCLOCK_CoreSysClk), 0U, 1U); //For informative

    /* Indicate to power library that PMIC is used. */
    POWER_UpdatePmicRecoveryTime(1);

    POWER_SetPadVolRange(&vrange);
    /* Determine the power mode before bring up. */
    if ((POWER_GetEventFlags() & PMC_FLAGS_DEEPPDF_MASK) != 0)
    {
        PRINTF("Board wake up from deep or full deep power down mode.\r\n");
        POWER_ClearEventFlags(PMC_FLAGS_DEEPPDF_MASK);
    }

#if (ENABLE_PSRAM == 1)
    CLOCK_InitSysPfd(kCLOCK_Pfd2, 24); /* Enable AUX0 PLL clock */

    status = BOARD_InitPsRam(kAUX0_PLL_to_FLEXSPI1_CLK);
    if (status != kStatus_Success)
    {
        assert(false);
    }
#endif
    NVIC_SetPriority(GPU_IRQn, 4);

#if (configUSE_TICKLESS_IDLE == 2)
    /* Enable 32KHz Oscillator clock */
    CLOCK_EnableOsc32K(true);
    /* Initialize RTC timer */
    RTC_Init(RTC);

    /* enable RTC interrupt */
    RTC_EnableInterrupts(RTC, RTC_CTRL_WAKE1KHZ_MASK);
    NVIC_SetPriority(RTC_IRQn, 4);
    EnableDeepSleepIRQ(RTC_IRQn);
#endif
#ifndef GPIO_DEBUG
//    recoverdefaultpad(xip_execution);
#endif

    poweroff(xip_execution);
    shutoff_clocks(xip_execution);
    set_resets(xip_execution);
#if (CONTROL_FRAME_RATE == 1)
     CLOCK_AttachClk(kLPOSC_to_UTICK_CLK);
     UTICK_Init(UTICK0);
     NVIC_SetPriority(UTICK0_IRQn, 4);
     EnableDeepSleepIRQ(UTICK0_IRQn);
#endif
    deepsleep_tickless_mode = 1;
    deep_user_resume_func = deepsleep_tickless_mode1_wakeup;

    if (xTaskCreate(vglite_task, "vglite_task", configMINIMAL_STACK_SIZE + 2000, NULL, configMAX_PRIORITIES - 1, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    vTaskStartScheduler();

    for(;;);
}


#if (DEMO_PANEL_RM67162 == DEMO_PANEL)
void GPIO_INTA_IRQHandler(void)
{
    uint32_t intStat;

    intStat = GPIO_PortGetInterruptStatus(GPIO, BOARD_MIPI_TE_PORT, 0);

    GPIO_PortClearInterruptFlags(GPIO, BOARD_MIPI_TE_PORT, 0, intStat);

    if (intStat & (1U << BOARD_MIPI_TE_PIN))
    {
        BOARD_DisplayTEPinHandler();
    }
}
#endif




static ELM_BUFFER_FORMAT _buffer_format_to_Elm(vg_lite_buffer_format_t format)
{
    switch (format)
    {
        case VG_LITE_RGB565:
            return ELM_BUFFER_FORMAT_RGB565;
            break;
        case VG_LITE_BGR565:
            return ELM_BUFFER_FORMAT_BGR565;
            break;
        case VG_LITE_BGRX8888:
            return ELM_BUFFER_FORMAT_BGRX8888;
            break;
        default:
            return ELM_BUFFER_FORMAT_RGBA8888;
            break;
    }
}

static void cleanup(void)
{
    vg_lite_close();
}

static int load_clock(void)
{    
    if (ClockAnalog_evo_len != 0)
	{
        analogClockHandle = ElmCreateObjectFromData(ELM_OBJECT_TYPE_EGO, (void *)ClockAnalog_evo, ClockAnalog_evo_len);
    }
    return (analogClockHandle != ELM_NULL_HANDLE);
}

static int load_hour(void)
{
    if (HourNeedle_evo_len != 0)
    {
        hourNeedleHandle = ElmCreateObjectFromData(ELM_OBJECT_TYPE_EGO, (void *)HourNeedle_evo, HourNeedle_evo_len);
    }
	return (hourNeedleHandle != ELM_NULL_HANDLE);
}

static int load_minute(void)
{    
    if (MinuteNeedle_evo_len != 0)
    {
        minuteNeedleHandle =
            ElmCreateObjectFromData(ELM_OBJECT_TYPE_EGO, (void *)MinuteNeedle_evo, MinuteNeedle_evo_len);
    }

    return (minuteNeedleHandle != ELM_NULL_HANDLE);
}

static int load_texture(void)
{
    int ret = 0;
    ret     = load_clock();
    if (ret < 0)
    {
        PRINTF("load_clock\r\n");
        return ret;
    }
    ret = load_hour();
    if (ret < 0)
    {
        PRINTF("load_hour\r\n");
        return ret;
    }
    ret = load_minute();
    if (ret < 0)
    {
        PRINTF("load_minute\r\n");
    }
    return ret;
}

static vg_lite_error_t init_vg_lite(void)
{
    vg_lite_error_t error = VG_LITE_SUCCESS;
    int ret               = 0;

    error = VGLITE_CreateDisplay(&display);
    if (error)
    {
        PRINTF("VGLITE_CreateDisplay failed: VGLITE_CreateDisplay() returned error %d\n", error);
        return error;
    }
    // Initialize the window.
    error = VGLITE_CreateWindow(&display, &window);
    if (error)
    {
        PRINTF("VGLITE_CreateWindow failed: VGLITE_CreateWindow() returned error %d\n", error);
        return error;
    }
    // Initialize the draw.

    ret = ElmInitalize(128, 128);
    if (!ret)
    {
        PRINTF("ElmInitalize failed\n");
        cleanup();
        return VG_LITE_OUT_OF_MEMORY;
    }

    // Setup a scale at center of buffer.
    vg_lite_identity(&matrix);
    vg_lite_translate(window.width / 2.0f, window.height / 2.0f, &matrix);
    // load the texture;
    ret = load_texture();
    if (ret < 0)
    {
        PRINTF("load_texture error\r\n");
        return VG_LITE_OUT_OF_MEMORY;
    }

    return error;
}

static ElmBuffer get_elm_buffer(vg_lite_buffer_t *buffer)
{
    for (int i = 0; i < APP_BUFFER_COUNT; i++)
    {
        if (elmFB[i].buffer == NULL)
        {
            elmFB[i].buffer = buffer;
            elmFB[i].handle = ElmWrapBuffer(buffer->width, buffer->height, buffer->stride, buffer->memory,
                                            buffer->address, _buffer_format_to_Elm(buffer->format));
            vg_lite_clear(buffer, NULL, 0x0);
            return elmFB[i].handle;
        }
        if (elmFB[i].buffer == buffer)
            return elmFB[i].handle;
    }
    return 0;
}
static int render(vg_lite_buffer_t *buffer, ElmHandle object)
{
    int status                = 0;
    ElmBuffer elmRenderBuffer = get_elm_buffer(buffer);
    status                    = ElmDraw(elmRenderBuffer, object);
    if (!status)
    {
        status = -1;
        return status;
    }
    ElmFinish();
    return status;
}

static void redraw()
{
    int status = 0;

    vg_lite_buffer_t *rt = VGLITE_GetRenderTarget(&window);
    if (rt == NULL)
    {
        PRINTF("vg_lite_get_renderTarget error\r\n");
        while (1)
            ;
    }
    static float angle = 0;
#ifdef GPIO_DEBUG
    GPIO_PinWrite(GPIO, GPIO_DEBUG_PORT_0, GPIO_DEBUG_PIN_0, 1);
#endif

    deepsleep_allowed = 0;

    // Draw the path using the matrix.
    status = render(rt, analogClockHandle);
    if (status == -1)
    {
        PRINTF("ELM Render analogClockHandle Failed");
        return;
    }

    ElmReset(hourNeedleHandle, ELM_PROP_TRANSFER_BIT);
    ElmTransfer(hourNeedleHandle, 200.0, 200.0);
//    ElmRotate(hourNeedleHandle, angle);

    status = render(rt, hourNeedleHandle);
    if (status == -1)
    {
        PRINTF("ELM Render hourNeedleHandle Failed");
    }

    ElmReset(minuteNeedleHandle, ELM_PROP_TRANSFER_BIT);
    ElmTransfer(minuteNeedleHandle, 200.0, 200.0);
    ElmRotate(minuteNeedleHandle, -angle);

    status = render(rt, minuteNeedleHandle);
    if (status == -1)
    {
        PRINTF("ELM Render minuteNeedleHandle Failed");
    }

#if (CONTROL_FRAME_RATE == 1)
    angle += 6;
#else
    angle += 0.5;
#endif

    VGLITE_SwapBuffers(&window);
    deepsleep_allowed = 1;
#ifdef GPIO_DEBUG
    GPIO_PinWrite(GPIO, GPIO_DEBUG_PORT_0, GPIO_DEBUG_PIN_0, 0);
#endif

    return;
}

uint32_t getTime(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void vglite_task(void *pvParameters)
{
    status_t status;
    vg_lite_error_t error;

    status = BOARD_PrepareVGLiteController();
    if (status != kStatus_Success)
    {
        PRINTF("Prepare VGlite contolor error\r\n");
        while (1)
            ;
    }
	
    error = init_vg_lite();
    if (error)
    {
        PRINTF("init_vg_lite failed: init_vg_lite() returned error %d\n", error);
        while (1)
            ;
    }
#if (CONTROL_FRAME_RATE == 1)
    UTICK_SetTick(UTICK0, kUTICK_Repeat, UTICK_TIME_1S - 1, UTickCallback);
#endif

    while (1)
    {
        redraw();
#if (CONTROL_FRAME_RATE == 1)
        DisableDeepSleepIRQ(GPIO_INTA_IRQn);
        xSemaphoreTake(xSemaphore_Timer, portMAX_DELAY);
#endif
    }

}


#if (CONTROL_FRAME_RATE == 1)
static void UTickCallback(void)
{
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;

   EnableDeepSleepIRQ(GPIO_INTA_IRQn);
   
#ifdef NON_DEEPSLEEP_WAKEUP
   DisableDeepSleepIRQ(GPIO_INTA_IRQn);
   DisableDeepSleepIRQ(RTC_IRQn);
   DisableDeepSleepIRQ(UTICK0_IRQn); 
#endif    
  (void)xSemaphoreGiveFromISR(xSemaphore_Timer, &xHigherPriorityTaskWoken);
   
  if( xHigherPriorityTaskWoken != pdFALSE )
  {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  
#ifdef GPIO_DEBUG
  GPIO_PortToggle(GPIO, GPIO_DEBUG_PORT_1, (1 << GPIO_DEBUG_PIN_1));
#endif
}
#endif
