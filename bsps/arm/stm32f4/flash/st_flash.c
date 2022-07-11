/* Copyright (c) 2022 Michael Davidsaver
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <rtems/bspIo.h>
#include <rtems/sysinit.h>

#include <rtems/console.h>
#include <rtems/termiostypes.h>

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/flash.h>

#ifdef STM32F4_FAMILY_F4XXXX

#include <bsp/stm32f4.h>

static
int stm32f4_flash_unlock(volatile stm32f4_flash * const hw)
{
  int ret = 0;
  rtems_interrupt_level level;

  rtems_interrupt_disable(level);

  if(hw->cr & STM32F4_FLASH_CR_LOCK) {
    hw->keyr = 0x45670123;
    hw->keyr = 0xcdef89ab;

    if(hw->cr & STM32F4_FLASH_CR_LOCK)
      ret = -EIO;

  } else {
    // concurrent operation in progress
    ret = -EBUSY;
  }

  rtems_interrupt_enable(level);

  return ret;
}

static
void stm32f4_flash_lock(volatile stm32f4_flash * const hw)
{
  rtems_interrupt_level level;

  rtems_interrupt_disable(level);

  hw->cr &= ~(STM32F4_FLASH_CR_PG|STM32F4_FLASH_CR_SER|STM32F4_FLASH_CR_MER);

  hw->cr |= STM32F4_FLASH_CR_LOCK;

  rtems_interrupt_enable(level);
}

static
int stm32f4_flash_wait_idle(volatile stm32f4_flash * const hw)
{
  /* Flash operations are slow, but also prevent/slow down
   * normal execution.
   * TODO: Does sleeping vs. spinning have a practical benefit?
   */
  while(hw->sr & STM32F4_FLASH_SR_BSY) {}

  if(hw->sr & (STM32F4_FLASH_SR_OPERR|STM32F4_FLASH_SR_PGAERR
               |STM32F4_FLASH_SR_PGPERR|STM32F4_FLASH_SR_PGSERR
               |STM32F4_FLASH_SR_WRPERR))
  {
    return -EIO;
  }
  return 0;
}

int stm32f4_flash_program(void *addr, const void* value, size_t count)
{
  volatile stm32f4_flash * const hw = STM32F4_FLASH;
  unsigned flashV = stm32f4_clock_config.flash_voltage_mV;
  uint8_t psize = 0;

  /* use multi-byte if Vcc is high enough, and request is aligned */
  if(flashV > 2700 && (3&(size_t)addr)==0 && (3&count)==0) {
    psize = 2;

  } else if(flashV > 2100 && (1&(size_t)addr)==0 && (1&count)==0) {
    psize = 1;
  }
  uint8_t progwidth = 1u<<psize;

  int ret = stm32f4_flash_unlock(hw);
  if(ret)
    return ret;

  if(!!(ret = stm32f4_flash_wait_idle(hw)))
    goto done;

  hw->sr |=STM32F4_FLASH_SR_OPERR|STM32F4_FLASH_SR_PGAERR
      |STM32F4_FLASH_SR_PGPERR|STM32F4_FLASH_SR_PGSERR;

  for(; count; count -= progwidth, value += progwidth, addr += progwidth) {
    uint32_t cr = hw->cr;
    cr = STM32F4_FLASH_CR_PSIZE_SET(cr, psize);
    cr|= STM32F4_FLASH_CR_PG;
    cr&= ~(STM32F4_FLASH_CR_SER|STM32F4_FLASH_CR_MER);
    hw->cr = cr;

    switch(progwidth) {
    case 1: *(uint8_t*)addr  = *(const uint8_t*)value; break;
    case 2: *(uint16_t*)addr = *(const uint16_t*)value; break;
    case 4: *(uint32_t*)addr = *(const uint32_t*)value; break;
    default: ret = -EINVAL; goto done;
    }

    if(!!(ret = stm32f4_flash_wait_idle(hw)))
      goto done;
  }

done:
  stm32f4_flash_lock(hw);
  return ret;
}

int stm32f4_flash_erase_sector(unsigned sectorn)
{
  volatile stm32f4_flash * const hw = STM32F4_FLASH;
  unsigned flashV = stm32f4_clock_config.flash_voltage_mV;
  uint8_t psize = 0;
  unsigned maxn = STM32F4_FLASH_CR_SNB_GET(0xffffffff);

  if(sectorn>=maxn)
    return -EINVAL;

  /* use multi-byte if Vcc is high enough */
  if(flashV > 2700) {
    psize = 2;

  } else if(flashV > 2100) {
    psize = 1;
  }

  int ret = stm32f4_flash_unlock(hw);
  if(ret)
    return ret;

  if(!!(ret = stm32f4_flash_wait_idle(hw)))
    goto done;

  hw->sr |=STM32F4_FLASH_SR_OPERR|STM32F4_FLASH_SR_PGAERR
      |STM32F4_FLASH_SR_PGPERR|STM32F4_FLASH_SR_PGSERR;

  uint32_t cr = hw->cr;
  cr = STM32F4_FLASH_CR_PSIZE_SET(cr, psize);
  cr = STM32F4_FLASH_CR_SNB_SET(cr, sectorn);
  cr|= STM32F4_FLASH_CR_SER;
  cr&= ~(STM32F4_FLASH_CR_PG|STM32F4_FLASH_CR_MER);
  hw->cr = cr;
  hw->cr |= STM32F4_FLASH_CR_STRT;

  ret = stm32f4_flash_wait_idle(hw);

done:
  stm32f4_flash_lock(hw);
  return ret;
}

void stm32f4_flash_cache_flush_all(void)
{
  volatile stm32f4_flash * const hw = STM32F4_FLASH;
  const uint32_t cr = hw->cr;

  /* ensure disabled before resetting */
  hw->cr &= ~(STM32F4_FLASH_ACR_DCEN | STM32F4_FLASH_ACR_ICEN);

  uint32_t mask = 0u;
  if(cr & STM32F4_FLASH_ACR_DCEN)
      mask |= STM32F4_FLASH_ACR_DCRST;
  if(cr & STM32F4_FLASH_ACR_ICEN)
      mask |= STM32F4_FLASH_ACR_ICRST;

  /* toggle resets */
  hw->cr |= mask;
  hw->cr &= ~mask;

  /* (maybe) re-enable */
  hw->cr = cr;
}

#endif /* STM32F4_FAMILY_F4XXXX */
