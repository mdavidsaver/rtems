/**
 * @file
 *
 * @ingroup RTEMSBSPsARMLPC24XX_clocks
 *
 * @brief System clocks.
 */

/*
 * Copyright (c) 2008-2014 embedded brains GmbH.  All rights reserved.
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

#include <rtems/counter.h>

#include <bsp.h>
#include <bsp/lpc24xx.h>
#include <bsp/system-clocks.h>

#ifdef ARM_MULTILIB_ARCH_V7M
#  include <bsp/clock-armv7m.h>
#endif

#ifndef LPC24XX_OSCILLATOR_INTERNAL
  #error "unknown internal oscillator frequency"
#endif

#ifndef LPC24XX_OSCILLATOR_MAIN
  #error "unknown main oscillator frequency"
#endif

#ifndef LPC24XX_OSCILLATOR_RTC
  #error "unknown RTC oscillator frequency"
#endif

void lpc24xx_timer_initialize(void)
{
  /* Reset timer */
  T1TCR = TCR_RST;

  /* Set timer mode */
  T1CTCR = 0;

  /* Set prescaler to zero */
  T1PR = 0;

  /* Reset all interrupt flags */
  T1IR = 0xff;

  /* Do not stop on a match */
  T1MCR = 0;

  /* No captures */
  T1CCR = 0;

  /* Start timer */
  T1TCR = TCR_EN;
}

#ifdef ARM_MULTILIB_ARCH_V7M

uint32_t _CPU_Counter_frequency(void)
{
  return _ARMV7M_Clock_frequency();
}

CPU_Counter_ticks _CPU_Counter_read(void)
{
  return _ARMV7M_Clock_counter(&_ARMV7M_TC);
}

#else /* !ARM_MULTILIB_ARCH_V7M */

uint32_t _CPU_Counter_frequency(void)
{
  return LPC24XX_PCLK;
}

CPU_Counter_ticks _CPU_Counter_read(void)
{
  return lpc24xx_timer();
}

#endif /* !ARM_MULTILIB_ARCH_V7M */

void lpc24xx_micro_seconds_delay(unsigned us)
{
  unsigned start = lpc24xx_timer();
  unsigned delay = us * (LPC24XX_PCLK / 1000000);
  unsigned elapsed = 0;

  do {
    elapsed = lpc24xx_timer() - start;
  } while (elapsed < delay);
}

#ifdef ARM_MULTILIB_ARCH_V7M
  /* Clock tree for lpc178x/7x
   * adapted from Fig. 4 on page 22
   * UM10470 LPC178x/7x User Manual
   * Rev 4.0 19 Dec. 2016
   *
   * mosc <- LPC24XX_OSCILLATOR_MAIN
   * iosc <- LPC24XX_OSCILLATOR_INTERNAL
   *
   * PLL1 <- mosc
   *
   * if CLKSRCSEL[CLKSRC]
   *   sysclk <- mosc
   * else
   *   sysclk <- iosc
   *
   * PLL0 <- sysclk
   *
   * if CCLKSEL[CCLKSRC]
   *   sysclk_or_pll0 <- PLL0
   * else
   *   sysclk_or_pll0 <- sysclk
   *
   * cclk <- sysclk_or_pll0 / CCLKSEL[CCLKDIV]
   *
   * emclk <- cclk / (EMCCLKSEL[EMCDIV]+1)
   *
   * pclk <- sysclk_or_pll0 / PCLKSEL[PCLKDIV]
   */
  static unsigned lpc17xx_sysclk(void)
  {
    volatile lpc17xx_scb *scb = &LPC17XX_SCB;
    return (scb->clksrcsel & LPC17XX_SCB_CLKSRCSEL_CLKSRC) != 0 ?
      LPC24XX_OSCILLATOR_MAIN
        : LPC24XX_OSCILLATOR_INTERNAL;
  }

  static unsigned lpc17xx_sysclk_or_pll0(void)
  {
    volatile lpc17xx_scb *scb = &LPC17XX_SCB;
    unsigned cclk_in = 0;

    if ((scb->cclksel & LPC17XX_SCB_CCLKSEL_CCLKSEL) != 0) {
      cclk_in = lpc24xx_pllclk();
    } else {
      cclk_in = lpc17xx_sysclk();
    }
    return cclk_in;
  }
#endif

unsigned lpc24xx_pllclk(void)
{
  #ifdef ARM_MULTILIB_ARCH_V4
    unsigned clksrc = GET_CLKSRCSEL_CLKSRC(CLKSRCSEL);
    unsigned pllinclk = 0;
    unsigned pllclk = 0;

    /* Get PLL input frequency */
    switch (clksrc) {
      case 0:
        pllinclk = LPC24XX_OSCILLATOR_INTERNAL;
        break;
      case 1:
        pllinclk = LPC24XX_OSCILLATOR_MAIN;
        break;
      case 2:
        pllinclk = LPC24XX_OSCILLATOR_RTC;
        break;
      default:
        return 0;
    }

    /* Get PLL output frequency */
    if ((PLLSTAT & PLLSTAT_PLLC) != 0) {
      uint32_t pllcfg = PLLCFG;
      unsigned n = GET_PLLCFG_NSEL(pllcfg) + 1;
      unsigned m = GET_PLLCFG_MSEL(pllcfg) + 1;

      pllclk = (pllinclk / n) * 2 * m;
    } else {
      pllclk = pllinclk;
    }
  #else
    volatile lpc17xx_scb *scb = &LPC17XX_SCB;
    unsigned sysclk = lpc17xx_sysclk();
    unsigned pllstat = scb->pll_0.stat;
    unsigned pllclk = 0;
    unsigned enabled_and_locked = LPC17XX_PLL_STAT_PLLE
      | LPC17XX_PLL_STAT_PLOCK;

    if ((pllstat & enabled_and_locked) == enabled_and_locked) {
      unsigned m = LPC17XX_PLL_SEL_MSEL_GET(pllstat) + 1;

      pllclk = sysclk * m;
    }
  #endif

  return pllclk;
}

unsigned lpc24xx_cclk(void)
{
  #ifdef ARM_MULTILIB_ARCH_V4
    /* Get PLL output frequency */
    unsigned pllclk = lpc24xx_pllclk();

    /* Get CPU frequency */
    unsigned cclk = pllclk / (GET_CCLKCFG_CCLKSEL(CCLKCFG) + 1);
  #else
    volatile lpc17xx_scb *scb = &LPC17XX_SCB;
    unsigned cclksel = scb->cclksel;
    unsigned cclk_in = lpc17xx_sysclk_or_pll0();
    unsigned cclk = 0;

    /* CCLKDIV can be zero, which will freeze the CPU.
     * As we are executing code, this is unlikely to
     * be the case.
     */
    cclk = cclk_in / LPC17XX_SCB_CCLKSEL_CCLKDIV_GET(cclksel);
  #endif

  return cclk;
}

unsigned lpc24xx_pclk(void)
{
#ifdef ARM_MULTILIB_ARCH_V7M
  volatile lpc17xx_scb *scb = &LPC17XX_SCB;
  unsigned pclksel = scb->pclksel;
  unsigned clk_in = lpc17xx_sysclk_or_pll0();
  unsigned pclk = 0;
  uint32_t pclkdiv = LPC17XX_SCB_PCLKSEL_PCLKDIV_GET(pclksel);

  /* PCLKDIV may legitimately be zero, which disables
   * the peripheral clock.
   */
  if(pclkdiv)
    pclk = clk_in / LPC17XX_SCB_PCLKSEL_PCLKDIV_GET(pclksel);
  return pclk;
#else
    return 0;
#endif
}
