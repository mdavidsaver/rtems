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

#include <bsp/rcc.h>
#include <bsp/stm32f4.h>

#include <rtems.h>

static void rcc_set(
  stm32f4_rcc_index index,
  bool set,
  volatile uint32_t *regs
)
{
  int reg = index >> 5;
  uint32_t one = 1;
  uint32_t bit = one << (index & 0x1f);
  rtems_interrupt_level level;
  uint32_t val;

  rtems_interrupt_disable(level);
  val = regs [reg];
  if (set) {
    val |= bit;
  } else {
    val &= ~bit;
  }
  regs [reg] = val;
#ifdef STM32F4_FAMILY_F2XX
  /* Errata for STM32F20x/STM32F21x and STM32F215STM32F217xx
   *  ES0005 rev 11, Jan. 2021
   * 2.1.11 - "Delay after RCC peripheral clock enabling"
   * Apparently the preceding store can complete before the clock is stable.
   * Take the first workaround.
   */
  __asm__ __volatile__("dsb":::"memory");
#endif
  rtems_interrupt_enable(level);
}

void stm32f4_rcc_reset(stm32f4_rcc_index index)
{
  stm32f4_rcc_set_reset(index, true);
  stm32f4_rcc_set_reset(index, false);
}

void stm32f4_rcc_set_reset(stm32f4_rcc_index index, bool set)
{
  volatile stm32f4_rcc *rcc = STM32F4_RCC;

#ifdef STM32F4_FAMILY_F4XXXX
  rcc_set(index, set, &rcc->ahbrstr [0]);
#endif/* STM32F4_FAMILY_F4XXXX */
#ifdef STM32F4_FAMILY_F10XXX
  /* The first register is missing for the reset-block */
  rcc_set(index, set, &rcc->cir);
#endif /* STM32F4_FAMILY_F10XXX */
}

void stm32f4_rcc_set_clock(stm32f4_rcc_index index, bool set)
{
  volatile stm32f4_rcc *rcc = STM32F4_RCC;

  rcc_set(index, set, &rcc->ahbenr [0]);
}

bool stm32f4_rcc_clock_enabled(stm32f4_rcc_index index)
{
    volatile stm32f4_rcc *rcc = STM32F4_RCC;
    unsigned reg = index >> 5;
    uint32_t bit = 1u << (index & 0x1f);

    return !!(rcc->ahbenr[reg] & bit);
}

#ifdef STM32F4_FAMILY_F4XXXX
void stm32f4_rcc_set_low_power_clock(stm32f4_rcc_index index, bool set)
{
  volatile stm32f4_rcc *rcc = STM32F4_RCC;

  rcc_set(index, set, &rcc->ahblpenr [0]);
}
#endif /* STM32F4_FAMILY_F4XXXX */
