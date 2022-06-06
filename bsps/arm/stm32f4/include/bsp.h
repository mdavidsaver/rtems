/**
 * @file
 * @ingroup RTEMSBSPsARMSTM32F4
 * @brief Global BSP definitions.
 */

/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Obere Lagerstr. 30
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_BSP_H
#define LIBBSP_ARM_STM32F4_BSP_H

/**
 * @defgroup RTEMSBSPsARMSTM32F4 STM32F4
 *
 * @ingroup RTEMSBSPsARM
 *
 * @brief STM32F4 Board Support Package.
 *
 * @{
 */

#include <bspopts.h>
#include <bsp/default-initial-extension.h>

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
    /** @brief High Speed External (HSE) clock frequency.
     *
     *  Set to 0 to use Internal (HSI) clock.
     */
    uint32_t hse_freq_hz;
    /** @brief PLL configuration.
     *
     *  Set pll_mult_n=0 to bypass.  (other pll_* then ignored)
     *
     *  Otherwise:
     *
     *  F_in = f_hsi | f_hse  (depending on hse_freq=)
     *  F_vco = F_in * N / M
     *  F_out = F_vco / P
     *
     *  Constraints:
     *  F_out <= 120 MHz
     *  N - [192, 432] inclusive
     *  M - [2, 63] inclusive
     *  P - 2, 4, 6, or 8
     *  192 MHz <= F_vco <= 432 MHz
     */
    unsigned pll_mult_n : 9;
    unsigned pll_div_m : 6;
    unsigned pll_div_p : 4;
    /** @brief Supply voltage level.
     *
     *  Used to compute FLASH memory access parameters.
     *  Higher voltage allows faster access.
     *
     *  Leave at 0 to use conservative default.
     *
     *  eg. For 3.3V, set to 3300
     */
    unsigned flash_voltage_mV : 13; // allow space for <= 5000
    /** @brief Upper limit on derived APB1/PCLK1 frequency
     *
     *  When non-zero, apply this limit in addition to part specific
     *  constraints.
     */
    unsigned pclk1_limit;
    /** @brief Upper limit on derived APB2/PCLK2 frequency
     *
     *  When non-zero, apply this limit in addition to part specific
     *  constraints.
     */
    unsigned pclk2_limit;

} stm32f4_clock_config_t;

/** @brief Application clock configuration.
 *
 *  An application may provide this symbol to override (HSI 16 MHz)
 *  clock configuration.
 *
 *  Use one of the STM32F4_CONFIG_CLOCK_* macros for tested configurations.
 */
extern
const stm32f4_clock_config_t stm32f4_clock_config;

/** @brief Explicitly select the power on default (16MHz).
 *
 *  A safe configuration when bringing up a new board.
 */
#define STM32F4_CONFIG_CLOCK_HSI_16MHZ \
const stm32f4_clock_config_t stm32f4_clock_config = {};

/** @brief Derive 100MHz clock from the internal osc. using the PLL.
 *
 *  Assumes worst case (low) core voltage when computing FLASH timing.
 */
#define STM32F4_CONFIG_CLOCK_HSI_PLL_100MHZ \
const stm32f4_clock_config_t stm32f4_clock_config = { \
    .pll_mult_n = 400, \
    .pll_div_m = 32, \
    .pll_div_p = 2, \
};

/** @return System clock frequency in Hz
 */
uint32_t bsp_sysclk(void);
/** @return Periphrial clock frequency in Hz
 *  @param 1 - PCLK1/APB1, 2 - PCLK2/APB2
 */
uint32_t bsp_pclk(unsigned n);

#define BSP_FEATURE_IRQ_EXTENSION

#define BSP_ARMV7M_IRQ_PRIORITY_DEFAULT (13 << 4)

#define BSP_ARMV7M_SYSTICK_PRIORITY (14 << 4)

#define BSP_ARMV7M_SYSTICK_FREQUENCY bsp_sysclk()

#ifdef __cplusplus
}
#endif /* __cplusplus */

/** @} */


#endif /* LIBBSP_ARM_STM32F4_BSP_H */
