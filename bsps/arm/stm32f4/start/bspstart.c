/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 * Copyright (c) 2022 Michael Davidsaver
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

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/irq.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <assert.h>
#include <bsp/stm32f4.h>

#ifdef STM32F4_FAMILY_F4XXXX

#include <bsp/stm32f4xxxx_rcc.h>
#include <bsp/stm32f4xxxx_flash.h>

// default clock config with all zeros
__attribute__((weak))
const stm32f4_clock_config_t stm32f4_clock_config;

#define STM32F4_HSI_OSCILLATOR 16000000

#if defined(STM32F4_HSE_OSCILLATOR) || defined(STM32F4_SYSCLK) \
  || defined(STM32F4_HCLK) || defined(STM32F4_PCLK1) \
  || defined(STM32F4_PCLK2)
#  error BSP option clock configuration now at runtime.  see bsp.h
#endif

static
uint32_t stm32f4_hclk_freq;
static
uint32_t stm32f4_pclk_freq[2];

uint32_t bsp_sysclk(void)
{
  return stm32f4_hclk_freq;
}

uint32_t bsp_pclk(unsigned n)
{
  return stm32f4_pclk_freq[n-1u];
}

// paranoia configuration for FLASH
static
void init_flash(const stm32f4_clock_config_t * const conf, uint32_t f_ref)
{
  volatile stm32f4_flash *flash = STM32F4_FLASH;
  uint32_t acr = flash->acr;
  uint32_t hclk_mhz = f_ref/1000000;

  // disable data and instruction caches, and prefetch.
  // clear resets, which should not be set.
  acr &= ~(STM32F4_FLASH_ACR_DCEN | STM32F4_FLASH_ACR_ICEN
           | STM32F4_FLASH_ACR_DCRST | STM32F4_FLASH_ACR_ICRST
           | STM32F4_FLASH_ACR_PRFTEN);

  // max. wait states
  acr = STM32F4_FLASH_ACR_LATENCY_SET(acr, 7);

  flash->acr = acr;

  // reset caches
  flash->acr = acr | STM32F4_FLASH_ACR_DCRST | STM32F4_FLASH_ACR_ICRST;
  flash->acr = acr;

  uint32_t nwaits;

  // cf. table 3 in PM0059 document from ST
  if(conf->flash_voltage_mV < 2100) {
    if(hclk_mhz<=16) {
      nwaits = 0;
    } else if(hclk_mhz<=32) {
      nwaits = 1;
    } else if(hclk_mhz<=48) {
      nwaits = 2;
    } else if(hclk_mhz<=64) {
      nwaits = 3;
    } else if(hclk_mhz<=80) {
      nwaits = 4;
    } else if(hclk_mhz<=96) {
      nwaits = 5;
    } else if(hclk_mhz<=112) {
      nwaits = 6;
    } else {
      nwaits = 7;
    }
  } else if(conf->flash_voltage_mV < 2400) {
    if(hclk_mhz<=18) {
      nwaits = 0;
    } else if(hclk_mhz<=36) {
      nwaits = 1;
    } else if(hclk_mhz<=54) {
      nwaits = 2;
    } else if(hclk_mhz<=72) {
      nwaits = 3;
    } else if(hclk_mhz<=90) {
      nwaits = 4;
    } else if(hclk_mhz<=108) {
      nwaits = 5;
    } else {
      nwaits = 6;
    }
  } else if(conf->flash_voltage_mV < 2700) {
    if(hclk_mhz<=24) {
      nwaits = 0;
    } else if(hclk_mhz<=48) {
      nwaits = 1;
    } else if(hclk_mhz<=72) {
      nwaits = 2;
    } else if(hclk_mhz<=96) {
      nwaits = 3;
    } else {
      nwaits = 4;
    }
  } else { // 2.7V -> 3.3V
    if(hclk_mhz<=30) {
      nwaits = 0;
    } else if(hclk_mhz<=60) {
      nwaits = 1;
    } else if(hclk_mhz<=90) {
      nwaits = 2;
    } else {
      nwaits = 3;
    }
  }

  acr = STM32F4_FLASH_ACR_LATENCY_SET(acr, nwaits);

  // enable caching
  acr |=  STM32F4_FLASH_ACR_DCEN | STM32F4_FLASH_ACR_ICEN;
  // PM0059 says prefetch "is useful if at least one wait
  // state is needed".
  if(nwaits)
    acr |= STM32F4_FLASH_ACR_PRFTEN;

  // enable
  flash->acr = acr;

  // ST doc recommends reading back config. to "check".
  // paranoia in case there is some timing issue.
  (void)flash->acr;
}

// find smallest PPRE such that f_out <= f_limit
static
bool calc_pclk_div(uint32_t f_in,
                   uint32_t f_limit,
                   uint32_t *pf_out,
                   uint32_t *ppre)
{
  uint32_t div = f_in / f_limit;

  if(f_in % f_limit)
    div++; // round to higher divider, lower frequency

  // round up to power of 2
  div--;
  div |= div >> 1u;
  div |= div >> 2u;
  div |= div >> 4u;
  div |= div >> 8u;
  div |= div >> 16u;
  div++;

  switch(div) {
  case 0:
  case 1: *ppre = 0x0; break;
  case 2: *ppre = 0x4; break;
  case 4: *ppre = 0x5; break;
  case 8: *ppre = 0x6; break;
  case 16:*ppre = 0x7; break;
  default:
    // impossible divider
    return false;
  }

  *pf_out = f_in / div;
  return true;
}

static
int init_main_osc(const stm32f4_clock_config_t * const conf)
{
  volatile stm32f4_rcc * const rcc = STM32F4_RCC;
  uint32_t f_ref;

  if(conf->hse_freq_hz) {
    // using external OSC (perhaps though PLL)

    rcc->cr |= STM32F4_RCC_CR_HSEON;

    // missing external clock will hang here.
    while ( !( rcc->cr & STM32F4_RCC_CR_HSERDY ) ) ;

    f_ref = conf->hse_freq_hz;

  } else {
    // using internal OSC (perhaps though PLL)

    rcc->cr |= STM32F4_RCC_CR_HSION; // likely already enabled

    while ( !( rcc->cr & STM32F4_RCC_CR_HSIRDY ) ) ;

    f_ref = STM32F4_HSI_OSCILLATOR;
  }

  if(conf->pll_mult_n) { // use PLL
    uint32_t pllcfgr = rcc->pllcfgr;

    if(conf->pll_mult_n < 192 || conf->pll_mult_n > 432)
      return 1;
    if(conf->pll_div_m < 2 || conf->pll_div_m > 63)
      return 2;
    if(conf->pll_div_p!=2 && conf->pll_div_p!=4
       && conf->pll_div_p!=6 && conf->pll_div_p!=8)
      return 3;

    uint64_t f_vco = (uint64_t)f_ref * conf->pll_mult_n / conf->pll_div_m;
    if(f_vco < 192000000 || f_vco > 432000000)
      return 4;

    pllcfgr = STM32F4_RCC_PLLCFGR_PLLN_SET(pllcfgr, conf->pll_mult_n);
    pllcfgr = STM32F4_RCC_PLLCFGR_PLLM_SET(pllcfgr, conf->pll_div_m);
    pllcfgr = STM32F4_RCC_PLLCFGR_PLLP_SET(pllcfgr, (conf->pll_div_p-1u)/2u);
    pllcfgr = STM32F4_RCC_PLLCFGR_PLLQ_SET(pllcfgr, 15);

    if(conf->hse_freq_hz)
      pllcfgr |= STM32F4_RCC_PLLCFGR_SRC;
    else
      pllcfgr &= ~STM32F4_RCC_PLLCFGR_SRC;

    rcc->cr &= ~(STM32F4_RCC_CR_PLLON | STM32F4_RCC_CR_PLLRDY);

    rcc->pllcfgr = pllcfgr;

    rcc->cr |= STM32F4_RCC_CR_PLLON;

    while ( !( rcc->cr & STM32F4_RCC_CR_PLLRDY ) ) ;

    // widen to avoid overflow
    uint64_t tmp = f_ref;
    tmp *= conf->pll_mult_n;
    tmp /= conf->pll_div_m;
    tmp /= conf->pll_div_p;
    f_ref = tmp;
  }

  if(f_ref > 120000000)
    return 10;

  uint32_t cfgr = rcc->cfgr;

#if defined(STM32F4_FAMILY_F2XX)
  uint32_t limit_pclk1 = 30000000;
  uint32_t limit_pclk2 = 60000000;
#else
  uint32_t limit_pclk1 = 42000000;
  uint32_t limit_pclk2 = 84000000;
#endif

  if(conf->pclk1_limit && limit_pclk1 > conf->pclk1_limit)
    limit_pclk1 = conf->pclk1_limit;
  if(conf->pclk2_limit && limit_pclk2 > conf->pclk2_limit)
    limit_pclk2 = conf->pclk2_limit;

  uint32_t pre_pclk1, pre_pclk2;

  if(!calc_pclk_div(f_ref, limit_pclk1, &stm32f4_pclk_freq[0], &pre_pclk1))
    return 20;
  if(!calc_pclk_div(f_ref, limit_pclk2, &stm32f4_pclk_freq[1], &pre_pclk2))
    return 30;

  // from this point, we are committed

  stm32f4_hclk_freq = f_ref;

  init_flash(conf, f_ref);

  // not using f_ref -> AHB divider
  cfgr = STM32F4_RCC_CFGR_HPRE_SET(cfgr, 0);
  // set AHB -> APBx dividers
  cfgr = STM32F4_RCC_CFGR_PPRE1_SET(cfgr, pre_pclk1);
  cfgr = STM32F4_RCC_CFGR_PPRE2_SET(cfgr, pre_pclk2);

  uint32_t sw;
  if(conf->pll_mult_n) {
    sw = STM32F4_RCC_CFGR_SW_PLL;

  } else if(conf->hse_freq_hz) {
    sw = STM32F4_RCC_CFGR_SW_HSE;

  } else {
    sw = STM32F4_RCC_CFGR_SW_HSI;
  }

  cfgr = STM32F4_RCC_CFGR_SW_SET(cfgr, sw);
  rcc->cfgr = cfgr;

  while(STM32F4_RCC_CFGR_SW_GET(rcc->cfgr) != sw) {}

  return 0;
}

#endif /* STM32F4_FAMILY_F4XXXX */

#ifdef STM32F4_FAMILY_F10XXX

static bool init_main_osc( const stm32f4_clock_config_t * const conf )
{
  (void)conf;
  return true;
}

#endif /* STM32F4_FAMILY_F10XXX */

void bsp_start( void )
{
  int err;
  if(0!=(err = init_main_osc(&stm32f4_clock_config))) {
    // fall back to internal osc.
    stm32f4_clock_config_t defconf = {};
    if(init_main_osc(&defconf))
      while(1) {}; // oops...
  }

  stm32f4_gpio_set_config_array( &stm32f4_start_config_gpio[ 0 ] );

  if(err){
    // with clocks and (some) I/O configured, hopefully printk()
    // can work now.
    printk("Warning: stm32f4_clock_config not valid.  err %d."
           "         Using default.\n",
           err);
  }

  bsp_interrupt_initialize();
}
