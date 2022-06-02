/* Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 * Copyright (c) 2022 Michael Davidsaver
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

/* Driver for ST USART core.
 *
 * This device can only buffer a single character on TX and RX
 * unless DMA is used (which we don't).  So this means an interrupt
 * per character sent or received...
 */

#include <rtems/bspIo.h>
#include <rtems/sysinit.h>

#include <rtems/console.h>
#include <rtems/termiostypes.h>

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/rcc.h>
#include <bsp/irq.h>
#include <bsp/usart.h>
#include <bsp/stm32f4.h>

/* configuration */

#define BSP_CONSOLE_MINOR 1
#define BSP_CONSOLE_CONTEXT BSP_CONSOLE_CONTEXT_(BSP_CONSOLE_MINOR)
#define BSP_CONSOLE_CONTEXT_(N) BSP_CONSOLE_CONTEXT__(N)
#define BSP_CONSOLE_CONTEXT__(N) usart ## N ## _context

typedef struct {
  const char* tty;
  const char* iname;
  volatile stm32f4_usart *hw;
  rtems_vector_number irq;
  stm32f4_rcc_index rcc;
  unsigned pclk : 2;
  unsigned ctsrts : 1;
} usart_info;

typedef struct {
  rtems_termios_device_context base;
  const usart_info* const info;
  uint32_t baud;
  uint8_t wrote;
} usart_context;

#ifdef STM32F4_ENABLE_USART_1
static const usart_info usart1_info = {
  .tty = "/dev/ttyS0",
  .iname = "UART1",
  .hw = STM32F4_USART_1,
  .irq = STM32F4_IRQ_USART1,
  .rcc = STM32F4_RCC_USART1,
  .pclk = 2,
  .ctsrts = 1,
};
static usart_context usart1_context = {
  .info = &usart1_info,
  .baud = STM32F4_USART_BAUD,
};
#endif

#ifdef STM32F4_ENABLE_USART_2
static const usart_info usart2_info = {
  .tty = "/dev/ttyS1",
  .iname = "UART2",
  .hw = STM32F4_USART_2,
  .irq = STM32F4_IRQ_USART2,
  .rcc = STM32F4_RCC_USART2,
  .pclk = 1,
  .ctsrts = 1,
};
static usart_context usart2_context = {
  .info = &usart2_info,
  .baud = STM32F4_USART_BAUD,
};
#endif

#ifdef STM32F4_ENABLE_USART_3
static const usart_info usart3_info = {
  .tty = "/dev/ttyS2",
  .iname = "UART3",
  .hw = STM32F4_USART_3,
  .irq = STM32F4_IRQ_USART3,
  .rcc = STM32F4_RCC_USART3,
  .pclk = 1,
  .ctsrts = 1,
};
static usart_context usart3_context = {
  .info = &usart3_info,
  .baud = STM32F4_USART_BAUD,
};
#endif

#ifdef STM32F4_ENABLE_USART_4
static const usart_info usart4_info = {
  .tty = "/dev/ttyS3",
  .iname = "UART4",
  .hw = STM32F4_USART_4,
  .irq = STM32F4_IRQ_UART4,
  .rcc = STM32F4_RCC_UART4,
  .pclk = 1,
  .ctsrts = 0,
};
static usart_context usart4_context = {
  .info = &usart4_info,
  .baud = STM32F4_USART_BAUD,
};
#endif

#ifdef STM32F4_ENABLE_USART_5
static const usart_info usart5_info = {
  .tty = "/dev/ttyS4",
  .iname = "UART5",
  .hw = STM32F4_USART_5,
  .irq = STM32F4_IRQ_UART5,
  .rcc = STM32F4_RCC_UART5,
  .pclk = 1,
  .ctsrts = 0,
};
static usart_context usart5_context = {
  .info = &usart5_info,
  .baud = STM32F4_USART_BAUD,
};
#endif

#ifdef STM32F4_ENABLE_USART_6
static const usart_info usart6_info = {
  .tty = "/dev/ttyS5",
  .iname = "UART6",
  .hw = STM32F4_USART_6,
  .irq = STM32F4_IRQ_USART6,
  .rcc = STM32F4_RCC_USART6,
  .pclk = 2,
  .ctsrts = 1,
};
static usart_context usart6_context = {
  .info = &usart6_info,
  .baud = STM32F4_USART_BAUD,
};
#endif

/* hardware interactions */


/*
 * a = 8 * (2 - CR1[OVER8])
 *
 * usartdiv = div_mantissa + div_fraction / a
 *
 * baud = pclk / (a * usartdiv)
 *
 * usartdiv = pclk / (a * baud)
 *
 * Calculation in integer arithmetic:
 *
 * 1. div_mantissa = pclk / (a * baud)
 *
 * 2. div_fraction = pclk / (baud - a * div_mantissa)
 */
static void usart_set_bbr(volatile stm32f4_usart *hw,
                          uint32_t pclk,
                          uint32_t baud
)
{
  uint32_t a = 8 * (2 - ((hw->cr1 & STM32F4_USART_CR1_OVER8) != 0));
  uint32_t div_mantissa_low = pclk / (a * baud);
  uint32_t div_fraction_low = pclk / (baud - a * div_mantissa_low);
  uint32_t div_mantissa_high;
  uint32_t div_fraction_high;
  uint32_t high_err;
  uint32_t low_err;
  uint32_t div_mantissa;
  uint32_t div_fraction;
  uint32_t bbr = hw->bbr;

  if (div_fraction_low < a - 1) {
    div_mantissa_high = div_fraction_low;
    div_fraction_high = div_fraction_low + 1;
  } else {
    div_mantissa_high = div_fraction_low + 1;
    div_fraction_high = 0;
  }

  high_err = pclk - baud * (a * div_mantissa_high + div_fraction_high);
  low_err = baud * (a * div_mantissa_low + div_fraction_low) - pclk;

  if (low_err < high_err) {
    div_mantissa = div_mantissa_low;
    div_fraction = div_fraction_low;
  } else {
    div_mantissa = div_mantissa_high;
    div_fraction = div_fraction_high;
  }

  bbr = STM32F4_USART_BBR_DIV_MANTISSA_SET(bbr, div_mantissa);
  bbr = STM32F4_USART_BBR_DIV_FRACTION_SET(bbr, div_fraction);
  hw->bbr = bbr;
}

static void usart_irq(void *raw)
{
  rtems_termios_tty *tty = raw;
  usart_context *ctxt = rtems_termios_get_device_context(tty);
  volatile stm32f4_usart *hw = ctxt->info->hw;
  uint32_t sr = hw->sr;

  if(sr & STM32F4_USART_SR_TXE) {
    // indirectly calls usart_write_support()
    // which either enqueues another character, or clears TXEIE
    rtems_termios_dequeue_characters(tty, ctxt->wrote);
  }

  if(sr & STM32F4_USART_SR_RXNE) {
    char c = STM32F4_USART_DR_GET(hw->dr);

    rtems_termios_enqueue_raw_characters(tty, &c, 1);
  }
}

static void usart_intialize(usart_context *ctxt,
                            struct termios *term)
{
  volatile stm32f4_usart *hw = ctxt->info->hw;
  uint32_t ref_hz = bsp_pclk(ctxt->info->pclk);

  stm32f4_rcc_set_clock(ctxt->info->rcc, true);

  /* This function likely called twice.  First for early printk(),
     * where we don't enable interrupts (and term==NULL).
     * Then again later via usart_first_open()
     */
  if(hw->cr1 & STM32F4_USART_CR1_RXNEIE)
    return; // already initialized

  if(ref_hz < 2000000 || ref_hz > 50000000)
    return; // out of range

  usart_set_bbr(hw, ref_hz, ctxt->baud);

  hw->cr1 |= STM32F4_USART_CR1_TE | STM32F4_USART_CR1_RE | STM32F4_USART_CR1_UE;

  if(term) {
    // initialize to match CR* registers
    term->c_ospeed = term->c_ispeed = rtems_termios_baud_to_number(ctxt->baud);
    term->c_cflag &= ~(CSTOPB|CSIZE|PARENB|PARODD);
    term->c_cflag |= CS8;
  }
}

/* handling for full console */

static bool usart_first_open(
    rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    struct termios *term,
    rtems_libio_open_close_args_t *args
    )
{
  usart_context *ctxt = (usart_context*)base;
  volatile stm32f4_usart *hw = ctxt->info->hw;
  rtems_status_code rc;
  (void)args;

  rc = rtems_interrupt_handler_install(ctxt->info->irq,
                                       ctxt->info->iname,
                                       RTEMS_INTERRUPT_UNIQUE,
                                       usart_irq,
                                       tty);
  if(rc!=RTEMS_SUCCESSFUL)
    return false;

  usart_intialize(ctxt, term);

  hw->cr1 |= STM32F4_USART_CR1_RXNEIE;

  return true;
}

static void usart_last_close(
    rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    rtems_libio_open_close_args_t *args
    )
{
  usart_context *ctxt = (usart_context*)base;
  volatile stm32f4_usart *hw = ctxt->info->hw;
  rtems_interrupt_level level;

  rtems_interrupt_disable(level);
  hw->cr1 &= ~(STM32F4_USART_CR1_TXEIE | STM32F4_USART_CR1_RXNEIE);
  rtems_interrupt_enable(level);

  (void)rtems_interrupt_handler_remove(ctxt->info->irq,
                                       usart_irq,
                                       tty);
}

static void usart_write_support(
    rtems_termios_device_context *base,
    const char *buf,
    size_t len
    )
{
  usart_context *ctxt = (usart_context*)base;
  volatile stm32f4_usart *hw = ctxt->info->hw;
  rtems_interrupt_level lvl;

  rtems_interrupt_disable(lvl);

  if(len == 0u) {
    hw->cr1 &= ~STM32F4_USART_CR1_TXEIE;
    ctxt->wrote = 0;

  } else {
    if(hw->sr & STM32F4_USART_SR_TXE) {
      ctxt->wrote = 1;
      hw->dr = STM32F4_USART_DR_SET(hw->dr, buf[0]);
    } else {
      ctxt->wrote = 0;
    }
    hw->cr1 |= STM32F4_USART_CR1_TXEIE;
  }

  rtems_interrupt_enable(lvl);
}

static bool usart_set_attributes(
    rtems_termios_device_context *base,
    const struct termios *term
    )
{
  usart_context *ctxt = (usart_context*)base;
  volatile stm32f4_usart *hw = ctxt->info->hw;
  uint32_t ref_hz = bsp_pclk(ctxt->info->pclk);
  rtems_interrupt_level lvl;

  uint32_t baud = rtems_termios_baud_to_number(term->c_ospeed);
  uint32_t idiv = ref_hz / 8 / baud;

  if(!(hw->cr1 & STM32F4_USART_CR1_OVER8))
    idiv /= 2u;

  if(idiv==0 || idiv>=(1u<<12u))
    return false; // divisor would be out of range

  if((term->c_cflag & CSIZE) != CS8)
    return false;

  if(term->c_cflag & (PARENB|PARODD))
    return false; // TODO: understand coupling between CR1[M] and CR1[PCE]

  if((term->c_cflag & CRTSCTS) && !ctxt->info->ctsrts)
    return false;

  rtems_interrupt_disable(lvl);

  usart_set_bbr(hw, ref_hz, baud);

  // selects 1 or 2 stops.
  // termios can't configure 0.5 or 1.5
  hw->cr2 = STM32F4_USART_CR2_STOP_SET(hw->cr2,
                                       (term->c_cflag & CSTOPB) ? 2 : 0);

  if(term->c_cflag & CRTSCTS) {
    hw->cr3 |= STM32F4_USART_CR3_CTSE | STM32F4_USART_CR3_RTSE;
  } else {
    hw->cr3 &= ~(STM32F4_USART_CR3_CTSE | STM32F4_USART_CR3_RTSE);
  }

  rtems_interrupt_enable(lvl);

  return true;
}

static const rtems_termios_device_handler usart_handler = {
  .first_open = usart_first_open,
  .set_attributes = usart_set_attributes,
  .write = usart_write_support,
  .last_close = usart_last_close,
  .mode = TERMIOS_IRQ_DRIVEN
};

static rtems_status_code usart_install(usart_context *ctxt)
{
  bool iscon = ctxt == &BSP_CONSOLE_CONTEXT;
  rtems_status_code rc;

  rc = rtems_termios_device_install(ctxt->info->tty,
                                    &usart_handler,
                                    NULL,
                                    &ctxt->base);

  if(rc==RTEMS_SUCCESSFUL && iscon) {
    (void)link(ctxt->info->tty, CONSOLE_DEVICE_NAME);
  }
  return rc;
}

rtems_status_code console_initialize(
    rtems_device_major_number major,
    rtems_device_minor_number minor,
    void *arg)
{
  rtems_status_code rc = RTEMS_SUCCESSFUL;
#ifdef STM32F4_ENABLE_USART_1
  rc = usart_install(&usart1_context);
#endif
#ifdef STM32F4_ENABLE_USART_2
  rc = usart_install(&usart2_context);
#endif
#ifdef STM32F4_ENABLE_USART_3
  rc = usart_install(&usart3_context);
#endif
#ifdef STM32F4_ENABLE_USART_4
  rc = usart_install(&usart4_context);
#endif
#ifdef STM32F4_ENABLE_USART_5
  rc = usart_install(&usart5_context);
#endif
#ifdef STM32F4_ENABLE_USART_6
  rc = usart_install(&usart6_context);
#endif
  return rc;
}

/* Handling for printk() and simple console */

static void usart_out(char c)
{
  volatile stm32f4_usart *hw = BSP_CONSOLE_CONTEXT.info->hw;
  rtems_interrupt_level level;
  uint32_t cr1;

  // (maybe) temporarily disable the TX empty interrupt while we spin
  // to preempt normal console output.
  rtems_interrupt_disable(level);
  cr1 = hw->cr1;
  if(cr1 & STM32F4_USART_CR1_TXEIE) {
    hw->cr1 = cr1 & ~(STM32F4_USART_CR1_TXEIE);
  }
  rtems_interrupt_enable(level);

  while( !(hw->sr & STM32F4_USART_SR_TXE)) {}
  hw->dr = STM32F4_USART_DR_SET(hw->dr, c);

  if(cr1 & STM32F4_USART_CR1_TXEIE) {
    rtems_interrupt_disable(level);
    hw->cr1 |= STM32F4_USART_CR1_TXEIE;
    rtems_interrupt_enable(level);
  }
}

static void usart_init(void)
{
  usart_intialize(&BSP_CONSOLE_CONTEXT, NULL);
  BSP_output_char = usart_out;
}

static void usart_early_init(char c)
{
  usart_intialize(&BSP_CONSOLE_CONTEXT, NULL);
  usart_out(c);
}

BSP_output_char_function_type BSP_output_char = usart_early_init;

BSP_polling_getchar_function_type BSP_poll_char;

RTEMS_SYSINIT_ITEM(
    usart_init,
    RTEMS_SYSINIT_BSP_START,
    RTEMS_SYSINIT_ORDER_LAST_BUT_5
    );
