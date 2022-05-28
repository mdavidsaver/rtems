/*
 * Copyright (c) 2022 Michael Davidsaver
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

/* Driver for I2C core of stm32f2 and stm32f4 (and perhaps others).
 *
 * Informed by:
 *   - Original i2c.c by Christian Mauderer
 *   - STM32F207xx ref. manual (RM0033 rev. 9)
 *   - Errata for STM32F20x/STM32F21x and STM32F215STM32F217xx
 *   - ST HAL library.  (considered "as built" documentation)
 *   - Various internet musings about recovery from bus lockup...
 *
 * Implements Linux I2C batch interface for I2C operations.
 * Only w/ 7 bit addressing.
 */

// some debug prints, not in ISRs
//#define DEBUG
//#define RTEMS_STATUS_CHECKS_USE_PRINTK

// Minimally invasive global I2C bus event log.  print 'i2c_events' from GDB.
//#define DEBUG_EVENT_LOG

#include <string.h>
#include <errno.h>

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/i2c.h>
#include <bsp/rcc.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>

#include <rtems/bspIo.h>
#include <rtems/error.h>
#include <rtems/status-checks.h>

#include <dev/i2c/i2c.h>

#define NELEM(ARR) (sizeof(ARR)/sizeof((ARR)[0]))

// manipulate bits in register
#define MANIP(VAR, SET, CLEAR) do { \
    uint32_t temp = VAR; \
    temp |= SET; \
    temp &= ~(CLEAR); \
  VAR = temp; \
  } while(0)

typedef enum {
  stm32f4_i2c_state_idle,
  stm32f4_i2c_state_start,
  stm32f4_i2c_state_addr,
  stm32f4_i2c_state_write,
  stm32f4_i2c_state_read,

  stm32f4_i2c_state_nodev,
  stm32f4_i2c_state_arlo,
  stm32f4_i2c_state_berr,
  stm32f4_i2c_state_error,
} stm32f4_i2c_state;

typedef struct {
  stm32f4_rcc_index rcc;
  volatile stm32f4_i2c * const hw;
  const rtems_vector_number irq_evt;
  const rtems_vector_number irq_err;
  char iname[5];
  char fname[10];
} stm32f4_i2c_bus_info;

struct stm32f4_i2c_bus {
  i2c_bus base;

  const stm32f4_i2c_bus_info* const info;
  const stm32f4_i2c_bus_conf* conf;

  stm32f4_i2c_state state;

  rtems_id client;

  const i2c_msg *msgs;
  uint32_t msg_count;
  uint16_t pos;
};

#ifdef DEBUG_EVENT_LOG
__attribute__((used))
static struct i2c_event {
  stm32f4_i2c_state state;
  uint32_t cr1, cr2, sr1, sr2;
  uint16_t dr, line;
  uint8_t n, N, nmsg;
} i2c_events[128];
static uint8_t i2c_events_next;

static
void record_event(unsigned line,
                  stm32f4_i2c_bus *bus,
                  uint32_t sr1,
                  uint32_t sr2,
                  uint16_t dr)
{
  uint8_t cur;
  do {
    cur = __atomic_load_n(&i2c_events_next, __ATOMIC_ACQUIRE);
    if(cur==RTEMS_ARRAY_SIZE(i2c_events))
      return;
  } while(!__atomic_compare_exchange_n(&i2c_events_next, &cur, cur+1,
                                       0,
                                       __ATOMIC_RELEASE, __ATOMIC_RELAXED));

  struct i2c_event* evt = &i2c_events[cur];

  evt->line = line;
  evt->state = bus->state;
  evt->n = bus->pos;
  evt->nmsg = bus->msg_count;
  if(bus->msg_count)
    evt->N = bus->msgs->len;

  evt->cr1 = bus->info->hw->cr1;
  evt->cr2 = bus->info->hw->cr2;
  evt->sr1 = sr1;
  evt->sr2 = sr2;
  evt->dr = dr;
}
#else
static
void record_event(unsigned line,
                  stm32f4_i2c_bus *bus,
                  uint32_t sr1,
                  uint32_t sr2,
                  uint16_t dr)
{}
#endif /* DEBUG_EVENT_LOG */

static const stm32f4_i2c_bus_info i2c_info1 = {
  .iname = "I2C1",
  .fname = STM32F4_I2C1_BUS_PATH,
  .rcc = STM32F4_RCC_I2C1,
  .hw = STM32F4_I2C1,
  .irq_evt = STM32F4_IRQ_I2C1_EV,
  .irq_err = STM32F4_IRQ_I2C1_ER,
};
stm32f4_i2c_bus stm32f4_i2c_bus1 = {
  .info = &i2c_info1,
};

static const stm32f4_i2c_bus_info i2c_info2 = {
  .iname = "I2C2",
  .fname = STM32F4_I2C2_BUS_PATH,
  .rcc = STM32F4_RCC_I2C2,
  .hw = STM32F4_I2C2,
  .irq_evt = STM32F4_IRQ_I2C2_EV,
  .irq_err = STM32F4_IRQ_I2C2_ER,
};
stm32f4_i2c_bus stm32f4_i2c_bus2 = {
  .info = &i2c_info2,
};

static const stm32f4_i2c_bus_info i2c_info3 = {
  .iname = "I2C3",
  .fname = STM32F4_I2C3_BUS_PATH,
  .rcc = STM32F4_RCC_I2C3,
  .hw = STM32F4_I2C3,
  .irq_evt = STM32F4_IRQ_I2C3_EV,
  .irq_err = STM32F4_IRQ_I2C3_ER,
};
stm32f4_i2c_bus stm32f4_i2c_bus3 = {
  .info = &i2c_info3,
};

static
int stm32f4_i2c_wait_idle(stm32f4_i2c_bus *bus, rtems_interval timeout)
{
  rtems_interval start = rtems_clock_get_ticks_since_boot();
  rtems_interval now = start;
  int ret = 0;

  while(bus->info->hw->sr2 & STM32F4_I2C_SR2_BUSY) {
    RTEMS_DEBUG_PRINT("%s busy, waiting...\n", bus->info->iname);

    (void)rtems_task_wake_after(1);

    now = rtems_clock_get_ticks_since_boot();

    if(timeout && now-start >= timeout) {
      ret = -ETIMEDOUT;
      break;
    }
  }

  return ret;
}

// return true if STOP issued (sequence complete)
static
bool stm32f4_i2c_maybe_start_stop(stm32f4_i2c_bus *bus)
{
  volatile stm32f4_i2c * const hw = bus->info->hw;

  if(bus->msg_count<=1) { // last message
    hw->cr1 |= STM32F4_I2C_CR1_STOP;
    bus->state = stm32f4_i2c_state_idle;
    return 1;

  } else { // repeat START for next message
    hw->cr1 |= STM32F4_I2C_CR1_START;
    bus->state = stm32f4_i2c_state_start;
    return 0;
  }
}

static
void stm32f4_i2c_maybe_next_msg(stm32f4_i2c_bus *bus)
{
  if(bus->msgs->len == bus->pos) {
    bus->msgs++;
    bus->msg_count--;
    bus->pos = 0;
  }
}

static
void stm32f4_i2c_set_error(stm32f4_i2c_bus *bus)
{
  volatile stm32f4_i2c * const hw = bus->info->hw;

  // disable interrupts to prevent live-lock
  hw->cr2 &= ~(STM32F4_I2C_CR2_ITBUFEN|STM32F4_I2C_CR2_ITEVTEN|STM32F4_I2C_CR2_ITERREN);

  record_event(__LINE__,
               bus,
               hw->sr1,
               hw->sr2,
               hw->dr);

  bus->state = stm32f4_i2c_state_error;
}

// invoked for: BERR, ARLO, AF, OVR, PECERR, TIMEOUT, SMBALERT
static
void stm32f4_i2c_error_irq(void* raw)
{
  stm32f4_i2c_bus *bus = (stm32f4_i2c_bus*)raw;
  volatile stm32f4_i2c * const hw = bus->info->hw;
  uint32_t sr2 = hw->sr2;
  uint32_t sr1 = hw->sr1;

  record_event(__LINE__,
               bus,
               sr1,
               sr2,
               0);

  if(sr1 & STM32F4_I2C_SR1_ARLO) {
    // arbitration lost
    //   ARLO implicitly switching off master mode
    hw->sr1 &= ~STM32F4_I2C_SR1_ARLO;
    bus->state = stm32f4_i2c_state_arlo;

  } else if(sr1 & STM32F4_I2C_SR1_BERR) {
    // bus error
    hw->sr1 &= ~STM32F4_I2C_SR1_BERR;
    bus->state = stm32f4_i2c_state_berr;

  } else if(sr1 & STM32F4_I2C_SR1_AF) { // no ACK for requested address
    hw->sr1 &= ~STM32F4_I2C_SR1_AF;

    hw->cr1 |= STM32F4_I2C_CR1_STOP;

    bus->state = stm32f4_i2c_state_nodev;

  } else {
    bus->state = stm32f4_i2c_state_error;
  }

  hw->cr2 &= ~(STM32F4_I2C_CR2_ITBUFEN|STM32F4_I2C_CR2_ITEVTEN|STM32F4_I2C_CR2_ITERREN);

  if(bus->client)
    rtems_event_transient_send(bus->client);
}

// invoked for: SB, ADDR, ADDR10, STOPF, BTF, TxE, ITBUFEN, RxNE
static
void stm32f4_i2c_event_irq(void* raw)
{
  stm32f4_i2c_bus *bus = (stm32f4_i2c_bus*)raw;
  volatile stm32f4_i2c * const hw = bus->info->hw;
  const i2c_msg *msg = bus->msgs;
  int done = 0;

  if(hw->cr1 & (STM32F4_I2C_CR1_START|STM32F4_I2C_CR1_STOP)) {
    /* We sometimes see these bits set when a command(s) has been issued,
     * but not yet executed.  Annoyingly this results in a burst of spurious
     * interrupts which can't be masked without also hiding the next legit
     * state... sigh
     *
     * Observed transient invalid states:
     * - on reSTART after write: CR1[START|PE] and SR1[TxE|BTF]
     * - on STOP after read: CR1[STOP|PE] and SR1[RxNE]
     */
    return;
  }

  /* Here read SR2 and then SR1 in order to see SR2 without
   * acknowledging/clearing ADDR.
   */
  uint32_t sr2 = hw->sr2;
  uint32_t sr1 = hw->sr1;

  if(!(sr1 & (STM32F4_I2C_SR1_ADDR|STM32F4_I2C_SR1_ADD10|STM32F4_I2C_SR1_SB
              |STM32F4_I2C_SR1_TxE|STM32F4_I2C_SR1_RxNE|STM32F4_I2C_SR1_BTF
              |STM32F4_I2C_SR1_STOPF)))
  {
    /* Interrupted with nothing to do.  Some noise on the line?
     * Seems to happen sometimes.  No big deal...
     */
    return;
  }

  record_event(__LINE__,
               bus,
               sr1,
               sr2,
               0);

  if(!msg || !bus->msg_count || !bus->client) {
    stm32f4_i2c_set_error(bus);
    return;
  }

  /* On each iteration, it is possible to match two of the following
   * three conditions.  Practically, first reading from, and then writing
   * to the DR register (perhaps twice each).
   *
   * - When beginning a write, both ADDR and TxE|BTF can be set.
   *   So after handling ADDR, two bytes may be queued.
   * - When reSTARTing after a single byte read, both RxNE and SB
   *   may be set.  So after dequeue of the final byte, the next
   *   address is written.
   */

  // maybe read data from DR (and shift)
  if(!(sr2 & STM32F4_I2C_SR2_TRA)) { // reading

    uint16_t nremain = msg->len - bus->pos;

    /* Because of DR and shift register games, we have to handle
     * the last 2 bytes specially.  This involves the last 2 or 3
     * interrupts, depending on whether RxNE or RxNE|BTF are set.
     */
    uint8_t pop = 0;
    if(nremain > 4) {
      // nothing special, pop from DR
      if(sr1 & (STM32F4_I2C_SR1_RxNE | STM32F4_I2C_SR1_BTF)) {
        pop = 1;
      }

    } else if(nremain == 4) {
      if(sr1 & (STM32F4_I2C_SR1_RxNE | STM32F4_I2C_SR1_BTF)) {
        // DR full, and shift is full or filling.
        // two outstanding

        // switch to BTF only for remaining bytes to avoid possible
        // race due to ambiguity about shift register state.
        hw->cr2 &= ~STM32F4_I2C_CR2_ITBUFEN;

        pop = 1;
      }

    } else if(nremain == 3) {
      if(sr1 & STM32F4_I2C_SR1_BTF) {
        // both DR and shift are full.
        // one outstanding

        // NACK the next (last) byte
        hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);

        pop = 1;

      } else if(sr1 & STM32F4_I2C_SR1_RxNE) {
        // DR is full, shift is filling.
        // one outstanding

        // wait for BTF to avoid NACK of the wrong byte
        hw->cr2 &= ~STM32F4_I2C_CR2_ITBUFEN;
      }

    } else if(nremain == 2) {
      if(sr1 & STM32F4_I2C_SR1_BTF) {
        // both DR and shift are full.
        // none outstanding.

        // disable NACK (redundant?)
        hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);

        if(stm32f4_i2c_maybe_start_stop(bus))
          done = 1;

        // pop twice to get values in DR and shift
        pop = 2;

      } else if(sr1 & STM32F4_I2C_SR1_RxNE) {
        // DR is full, shift is filling

        // Wait for BTF.
        hw->cr2 &= ~STM32F4_I2C_CR2_ITBUFEN;
      }

    } else if(nremain == 1) {
      // BTF should never be set in this situation,
      // and never without RxNE also being set.
      // Paranoia check of both.

      if(sr1 & (STM32F4_I2C_SR1_RxNE | STM32F4_I2C_SR1_BTF)) {
        // DR is full, shift should not be filling (due to NACK).
        hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);

        // STOP already issued while handling ADDR

        pop = 1;
      }

    } else { // nremain == 0
      if(sr1 & (STM32F4_I2C_SR1_RxNE | STM32F4_I2C_SR1_BTF)) {
        // extraneous data?

        // attempt to read
        uint8_t v = hw->dr;
        record_event(__LINE__,
                     bus,
                     sr1,
                     sr2,
                     v);

        hw->cr2 &= ~(STM32F4_I2C_CR2_ITBUFEN|STM32F4_I2C_CR2_ITEVTEN|STM32F4_I2C_CR2_ITERREN);

        bus->state = stm32f4_i2c_state_error;
        done = 1;
      }
    }

    if(pop) {
      uint8_t v = hw->dr;

      record_event(__LINE__,
                   bus,
                   sr1,
                   sr2,
                   v);
      msg->buf[bus->pos++] = v;

      if(pop==2) {
        v = hw->dr;

        record_event(__LINE__,
                     bus,
                     sr1,
                     sr2,
                     v);
        msg->buf[bus->pos++] = v;
      }

      stm32f4_i2c_maybe_next_msg(bus);
    }
  }

  // handle (re)START or ADDR.  (believed to be mutually exclusive)
  if(done) {
    // no-op

  } else if(sr1 & STM32F4_I2C_SR1_SB) { // (re)STARTed
    uint8_t addr = msg->addr<<1u;
    if(msg->flags & I2C_M_RD)
      addr |= 1u;

    record_event(__LINE__,
                 bus,
                 sr1,
                 sr2,
                 addr);

    hw->dr = addr; // implicitly clears SR1[SB]

    bus->state = stm32f4_i2c_state_addr;

  } else if(sr1 & STM32F4_I2C_SR1_ADDR) {
    if(msg->len==0) {
      // zero length read/write, aka. i2cdetect, aka. SMBus Quick Command

      hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);

      // will issue STOP or reSTART below

    } else if(msg->flags & I2C_M_RD) { // Read
      if(msg->len==1) {
        // prepare to NACK the only byte read
        hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);

        // now wait for RxNE | BTF

        hw->cr2 |= STM32F4_I2C_CR2_ITBUFEN;

      } else if(msg->len==2) {
        /* The Ref. Manual is very specific about special handling
         * when reading exactly two bytes to NACK two bytes in
         * advance.
         *
         * As I understand it, the ACK bit acts when the shift
         * register (behind the Data Register) becomes full.
         * When starting a new transfer, with both registers empty,
         * so this could potentially happen twice before software has a
         * chance to act.
         *
         * This necessitates special handling when exactly two bytes
         * remain to be received, both on ADDR and below when RxNE | BTF.
         */
        MANIP(hw->cr1,
              STM32F4_I2C_CR1_POS,
              STM32F4_I2C_CR1_ACK);

        // now wait for BTF only
        hw->cr2 &= ~STM32F4_I2C_CR2_ITBUFEN;

      } else { // len > 2
        MANIP(hw->cr1,
              STM32F4_I2C_CR1_ACK,
              STM32F4_I2C_CR1_POS);

        // now wait for RxNE | BTF

        hw->cr2 |= STM32F4_I2C_CR2_ITBUFEN;
      }

      bus->state = stm32f4_i2c_state_read;

    } else { // Write

      if(msg->flags & I2C_M_RD) {
        // oops, trying to transmit a read message?
        stm32f4_i2c_set_error(bus);

      } else {
        // paranoia.  Not used during a write, but clear anyway.
        hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);

        hw->cr2 |= STM32F4_I2C_CR2_ITBUFEN;

        bus->state = stm32f4_i2c_state_write;
      }

    }

    (void)hw->sr2; // implicitly clears SR1[ADDR]

    if(msg->len==0) {
      // issue STOP or reSTART
      if(stm32f4_i2c_maybe_start_stop(bus))
        done = 1;
      stm32f4_i2c_maybe_next_msg(bus);
      // 'msg' no longer valid

    } else if((msg->flags & I2C_M_RD) && msg->len==1) {
      // special case for 1 byte read.
      // After clearing ACK and ADDR, trigger a STOP/reSTART.
      // Still need to wait for RxNE.

      (void)stm32f4_i2c_maybe_start_stop(bus);
    }

    /* TODO: As an optimization, when writing, we could fall through immediate
     * if TxE is set.  Which it seems to happen.
     */
  }

  // Write data
  // When beginning a write, both ADDR and TxE|BTF may be set.
  if(done) {
    // no-op

  } else if(sr2 & STM32F4_I2C_SR2_TRA) { // writing

    if(bus->pos < msg->len && (sr1 & STM32F4_I2C_SR1_TxE)) {
      // more bytes to send...

      uint8_t v = msg->buf[bus->pos++];

      record_event(__LINE__,
                   bus,
                   sr1,
                   sr2,
                   v);
      hw->dr = v;

      if(bus->pos < msg->len && (sr1 & STM32F4_I2C_SR1_BTF)) {
        // immediate space for another
        uint8_t v = msg->buf[bus->pos++];

        record_event(__LINE__,
                     bus,
                     sr1,
                     sr2,
                     v);
        hw->dr = v;
      }

      if(bus->pos == msg->len)
        hw->cr2 &= ~STM32F4_I2C_CR2_ITBUFEN;

    } else if(bus->pos == msg->len && (sr1 & (STM32F4_I2C_SR1_BTF))) {

      if(stm32f4_i2c_maybe_start_stop(bus))
        done = 1;
      stm32f4_i2c_maybe_next_msg(bus);
    }
  }

  if(done && bus->client) {
    rtems_event_transient_send(bus->client);
  }
}
__attribute__((weak))
int stm32f4_i2c_reset_hook(stm32f4_i2c_bus *bus,
                           bool reset)
{
  (void)bus;
  (void)reset;
  return 0;
}

static
void stm32f4_i2c_reset_gpio(stm32f4_i2c_bus *bus)
{
#ifdef STM32F4_FAMILY_F4XXXX
  stm32f4_i2c_bus_conf tconf = *bus->conf;

  // switch SDA and SCL to GPIO Input
  tconf.sda.fields.mode = STM32F4_GPIO_MODE_INPUT;
  tconf.scl.fields.mode = STM32F4_GPIO_MODE_INPUT;
  stm32f4_gpio_set_config(&tconf.sda);
  stm32f4_gpio_set_config(&tconf.scl);

  if(!stm32f4_gpio_get_input(tconf.scl.fields.pin_first)) {
    // SCL is being pulled low.  There is nothing we can do.
    // Another master is mis-behaving.
    RTEMS_SYSLOG_ERROR("%s unrecoverable lockup\n",
                       bus->info->iname);

  } else if(!stm32f4_gpio_get_input(tconf.sda.fields.pin_first)) {

    // switch SCL to GPIO high Output
    tconf.scl.fields.mode = STM32F4_GPIO_MODE_OUTPUT;
    stm32f4_gpio_set_output(tconf.scl.fields.pin_first, true);
    stm32f4_gpio_set_config(&tconf.scl);

    /* toggle out 9x SCL pulses to complete any partially complete frame.
     * If reading, the slave should detect a NACK on one of these, and
     * release SDA.  However, without a STOP at the proper time, things
     * may still not fully recover.
     */
    for(unsigned n=0; n<9; n++) {
      stm32f4_gpio_set_output(tconf.scl.fields.pin_first, false);
      (void)rtems_task_wake_after(1);
      stm32f4_gpio_set_output(tconf.scl.fields.pin_first, true);
      (void)rtems_task_wake_after(1);
    }

    // If SDA
    if(stm32f4_gpio_get_input(tconf.sda.fields.pin_first))
      RTEMS_SYSLOG_PRINT("%s GPIO reset apparently successful.\n",
                         bus->info->iname);
    else
      RTEMS_SYSLOG_ERROR("%s GPIO reset not successful.\n",
                         bus->info->iname);
  }

#else
  RTEMS_DEBUG_PRINT("%s stm32f4_i2c_reset_gpio() not implemented\n",
                    bus->info->iname);
#endif
}

static
void stm32f4_i2c_hw_init(stm32f4_i2c_bus *bus)
{
  uint32_t cr1 = bus->info->hw->cr1;
  uint32_t cr2 = bus->info->hw->cr2;

  // disable all interrupts
  cr2 &= ~(STM32F4_I2C_CR2_ITEVTEN | STM32F4_I2C_CR2_ITERREN);
  cr2 &= ~(STM32F4_I2C_CR2_ITBUFEN | STM32F4_I2C_CR2_DMAEN | STM32F4_I2C_CR2_LAST);

  // CR2[FREQ] is ref. clock frequency in MHz
  cr2 = STM32F4_I2C_CR2_FREQ_SET(cr2, bsp_pclk(1)/1000000);

  bus->info->hw->cr2 = cr2;

  // enable
  cr1 |= STM32F4_I2C_CR1_PE;
  // paranoia, other control/command bits should be cleared by reset
  cr1 &= ~(STM32F4_I2C_CR1_ALERT | STM32F4_I2C_CR1_PEC | STM32F4_I2C_CR1_POS);
  cr1 &= ~(STM32F4_I2C_CR1_ACK | STM32F4_I2C_CR1_STOP | STM32F4_I2C_CR1_START);
  cr1 &= ~(STM32F4_I2C_CR1_NOSTRETCH | STM32F4_I2C_CR1_ENGC | STM32F4_I2C_CR1_ENPEC);
  cr1 &= ~(STM32F4_I2C_CR1_ENARP | STM32F4_I2C_CR1_SMBTYPE | STM32F4_I2C_CR1_SMBUS);

  bus->info->hw->cr1 = cr1;

  if(bus->info->hw->sr2 & STM32F4_I2C_SR2_BUSY) {
    RTEMS_SYSLOG_WARNING("%s bus initially busy, lockup?\n", bus->info->iname);
  }

  bus->state = stm32f4_i2c_state_idle;
}

static
void stm32f4_i2c_reset(stm32f4_i2c_bus *bus)
{
  volatile stm32f4_i2c * const hw = bus->info->hw;

  RTEMS_SYSLOG_PRINT("%s bus reset\n", bus->info->iname);

  // save clock config
  uint32_t ccr = hw->ccr;
  uint32_t trise = hw->trise;

  // disable all interrupts
  hw->cr2 &= ~(STM32F4_I2C_CR2_ITEVTEN | STM32F4_I2C_CR2_ITERREN | STM32F4_I2C_CR2_ITBUFEN);

  // place i2c core in reset
  hw->cr1 |= STM32F4_I2C_CR1_SWRST;

  // try to clean up...
  stm32f4_i2c_reset_hook(bus, true);
  if(!bus->conf->inhibit_recover)
    stm32f4_i2c_reset_gpio(bus);
  stm32f4_i2c_reset_hook(bus, false);

  // pins set back to AF
  stm32f4_gpio_set_config(&bus->conf->scl);
  stm32f4_gpio_set_config(&bus->conf->sda);

  // resume from reset
  hw->cr1 &= ~STM32F4_I2C_CR1_SWRST;

  // restore clock config
  hw->ccr = ccr;
  hw->trise = trise;

  // re-enable
  stm32f4_i2c_hw_init(bus);
}

static
void stm32f4_i2c_abort(stm32f4_i2c_bus *bus)
{
  volatile stm32f4_i2c * const hw = bus->info->hw;

  record_event(__LINE__,
               bus,
               hw->sr1,
               hw->sr2,
               0);

  // last op hasn't completed?
  // wait a bit to see if this is transient.
  if(hw->sr2 & STM32F4_I2C_SR2_MSL) {
    RTEMS_DEBUG_PRINT("Abort while transfer in progress?\n");

    rtems_interval start = rtems_clock_get_ticks_since_boot();
    do {
      (void)rtems_task_wake_after(1);
    }while((hw->sr2 & STM32F4_I2C_SR2_MSL) && rtems_clock_get_ticks_since_boot()-start<bus->base.timeout);
  }

  // Something has gone wrong.
  // make an attempt to recover within i2c core functionality
  if(hw->sr2 & STM32F4_I2C_SR2_MSL) {
    RTEMS_DEBUG_PRINT("Abort while transfer in progress!\n");

    hw->cr1 &= ~(STM32F4_I2C_CR1_ACK|STM32F4_I2C_CR1_POS);
    hw->cr1 |= STM32F4_I2C_CR1_STOP;

    if(!(hw->sr2 & STM32F4_I2C_SR2_TRA)) {
      if(hw->sr1 & STM32F4_I2C_SR1_BTF)
        (void)hw->dr;
      if(hw->sr1 & STM32F4_I2C_SR1_RxNE)
        (void)hw->dr;
    }
  }

  // Something has gone really wrong.
  // do a full reset, and attempt to recover the bus.
  if(hw->sr2 & STM32F4_I2C_SR2_MSL) {
    RTEMS_DEBUG_PRINT("Abort while transfer in progress!!\n");

    stm32f4_i2c_reset(bus);
  }

  // PE=0 clears interrupt status (and by extension data and shift registers)
  hw->cr1 &= ~STM32F4_I2C_CR1_PE;
  hw->cr1 |= STM32F4_I2C_CR1_PE;

  bus->state = stm32f4_i2c_state_idle;
}

static
int stm32f4_i2c_set_clock(i2c_bus *base, unsigned long clock_hz)
{
  stm32f4_i2c_bus *bus = (stm32f4_i2c_bus*)base;
  const uint32_t ref_clk_hz = bsp_pclk(1);

  if(clock_hz==0) {
    return -EINVAL;

  } else if(clock_hz >=88000 && clock_hz<=100000) {
    /* The STM32F205 Errata mentions a possible timing violation during a
     * repeated START with standard mode speeds in the range 88KHz -> 100KHz.
     * So default to a slower speed.
     */
    RTEMS_DEBUG_PRINT("%s Fudging %u -> 88000 on Errata\n",
                      bus->info->iname);
    clock_hz = 88000;

  } else if(clock_hz > 100000 && ref_clk_hz < 4000000) {
    // fast mode requires at least 4MHz ref.
    clock_hz = 100000;

  } else if(clock_hz>400000) {
    clock_hz = 400000;
  }

  uint32_t ccr = bus->info->hw->ccr;
  uint32_t trise = bus->info->hw->trise;

  ccr &= ~(STM32F4_I2C_CCR_FS | STM32F4_I2C_CCR_DUTY);

  uint32_t div;
  if(clock_hz <= 100000) {
    // Select standard mode (aka. Sm).
    // DUTY not used in Sm, but clear it anyway

    // in Sm, CCR is a simply divider.
    // T_i2c = 2*CCR*T_ref
    div = ref_clk_hz / (2 * clock_hz);

    if(div<4)
      return -EINVAL;

  } else if(clock_hz <= 400000) { // fast mode
    ccr |= STM32F4_I2C_CCR_FS;

    div = ref_clk_hz / (3 * clock_hz);

    if(div<4)
      return -EINVAL;

  } else { // super fast mode?
    // TODO: is this the condition which requires that F_pclk1
    //       be a multiple of 10MHz?
    ccr |= STM32F4_I2C_CCR_FS|STM32F4_I2C_CCR_DUTY;

    div = ref_clk_hz / (25 * clock_hz);

    if(div<1)
      return -EINVAL;
  }

  if(div > STM32F4_I2C_CCR_CCR_MAX) {
    return -EINVAL;
  }
  ccr = STM32F4_I2C_CCR_CCR_SET(ccr, div);

  // max. permitted clock rise times.  (SMBus spec. v3.1)
  // <=100KHz, 1000 ns (1MHz)
  // <=400KHz,  300 ns (3.33MHz)
  // <=1MHz  ,  120 ns (8.33MHz)
  // TRise is the number ref. clock ticks in >= this interval
  uint32_t risef;
  if(clock_hz <= 100000) {
    risef = 1000000;

  } else if(clock_hz <= 400000) {
    risef = 3333333;

  } else { // <= 1MHz
    risef = 8333333;
  }
  trise = STM32F4_I2C_TRISE_SET(trise, 1 + ref_clk_hz / risef);

  RTEMS_DEBUG_PRINT("%s set clock %lu ccr=0x%x trise=0x%x\n",
                    bus->info->iname, clock_hz, ccr, trise);

  // doc for CCR and TRISE says that we must disable before changing.
  bus->info->hw->cr1 &= ~STM32F4_I2C_CR1_PE;

  bus->info->hw->ccr = ccr;
  bus->info->hw->trise = trise;

  bus->info->hw->cr1 |= STM32F4_I2C_CR1_PE;

  return 0;
}

static
int stm32f4_i2c_transfer(i2c_bus *base, i2c_msg *msgs, uint32_t msg_count)
{
  stm32f4_i2c_bus *bus = (stm32f4_i2c_bus*)base;
  rtems_interval timeout = bus->base.timeout;
  rtems_interrupt_level lvl;
  uint32_t m;
  int ret;

  if(!msg_count)
    return 0;

  // argument validation
  for(m=0; m<msg_count; m++) {
    i2c_msg *msg = &msgs[m];

    if(msg->flags&~I2C_M_RD) { // no I2C_M_TEN, etc.
      RTEMS_SYSLOG_ERROR("%s unsupported flags 0x%04x\n", bus->info->iname, msg->flags);
      return -EINVAL;

    } else if(msg->addr > 0x7f) {
      RTEMS_SYSLOG_ERROR("%s address in lower 7 bits\n", bus->info->iname);
      return -EINVAL;
    }
  }

  i2c_bus_obtain(&bus->base);

  if(bus->info->hw->sr2 & STM32F4_I2C_SR2_MSL) {
    RTEMS_SYSLOG_ERROR("%s previous op left bus in lockup!\n", bus->info->iname);
    return -ENODEV;
  }

  RTEMS_DEBUG_PRINT("%s transfer cnt=%u first 0x%08x %u\n",
                    bus->info->iname, msg_count,
                    msgs[0].flags, msgs[0].len);

  ret = stm32f4_i2c_wait_idle(bus, timeout);
  if(ret) {
    RTEMS_DEBUG_PRINT("%s timeout waiting for idle\n", bus->info->iname);

  } else {

    bus->client = rtems_task_self();
    bus->msgs = msgs;
    bus->msg_count = msg_count;
    bus->pos = 0u;
    bus->state = stm32f4_i2c_state_start;

    record_event(__LINE__,
                 bus,
                 0xffffffff,
                 0xffffffff,
                 0);

    rtems_interrupt_disable(lvl);

    // enable event and error interrupts
    // buffer interrupt may be enabled later
    bus->info->hw->cr2 |= (STM32F4_I2C_CR2_ITEVTEN | STM32F4_I2C_CR2_ITERREN);

    /* ST docs don't require setting ACK here, but examples found all did so.
     * ACK is sticky, and will remain set until we clear it to end a read.
     * Seems to do no harm during a write.
     */
    bus->info->hw->cr1 |= STM32F4_I2C_CR1_START | STM32F4_I2C_CR1_ACK;

    rtems_interrupt_enable(lvl);

    rtems_status_code rc = rtems_event_transient_receive(RTEMS_WAIT, timeout);

    rtems_interrupt_disable(lvl);

#ifdef DEBUG
    {
      // read order avoids accidental clearing ADDR
      uint32_t sr2 = bus->info->hw->sr2;
      uint32_t sr1 = bus->info->hw->sr1;
      RTEMS_DEBUG_PRINT("%s FIN state=%d CR1=%08x CR2=%08x SR1=0x%08x SR2=0x%08x\n",
                        bus->info->iname, bus->state,
                        bus->info->hw->cr1, bus->info->hw->cr2,
                        sr1, sr2);
    }
#endif

    // disable interrupts and check what has happened
    bus->info->hw->cr2 &= ~(STM32F4_I2C_CR2_ITEVTEN | STM32F4_I2C_CR2_ITERREN | STM32F4_I2C_CR2_ITBUFEN);

    rtems_interrupt_enable(lvl);

    stm32f4_i2c_state final_state = bus->state;
    stm32f4_i2c_abort(bus);

    // spoil
    bus->client = 0;

    if(rc!=RTEMS_SUCCESSFUL) {
      rtems_event_transient_clear();
    }

    if(!bus->msg_count && final_state==stm32f4_i2c_state_idle) {
      RTEMS_DEBUG_PRINT("%s complete\n", bus->info->iname);
      ret = 0;

    } else if(bus->msg_count==msg_count && final_state==stm32f4_i2c_state_nodev) {
      RTEMS_DEBUG_PRINT("%s no ack to first ADDR\n", bus->info->iname);
      ret = -EIO;

    } else if(final_state==stm32f4_i2c_state_arlo) {
      RTEMS_SYSLOG_ERROR("%s arbitration lost\n", bus->info->iname);
      ret = -EIO;

    } else if(final_state==stm32f4_i2c_state_berr) {
      RTEMS_SYSLOG_ERROR("%s bus error\n", bus->info->iname);
      ret = -EIO;

    } else if(rc==RTEMS_TIMEOUT) {
      RTEMS_DEBUG_PRINT("%s timeout\n", bus->info->iname);
      ret = -ETIMEDOUT;

    } else if(rc!=RTEMS_SUCCESSFUL) {
      RTEMS_SYSLOG_ERROR_WITH_SC(rc, "I2C error\n");
      ret = -ETIMEDOUT;

    } else {
      RTEMS_SYSLOG_ERROR("I2C unexpected error.  @ %u, %u in %u\n",
                         bus->pos, bus->msg_count, final_state);
      ret = -EIO;
    }
  }

  i2c_bus_release(&bus->base);

  return ret;
}

static
void stm32f4_i2c_destroy(i2c_bus *base)
{
  stm32f4_i2c_bus *bus = (stm32f4_i2c_bus*)base;
  rtems_status_code rc;

  // disable interrupts
  bus->info->hw->cr2 &= ~(STM32F4_I2C_CR2_ITEVTEN | STM32F4_I2C_CR2_ITERREN);

  rc = rtems_interrupt_handler_remove(bus->info->irq_evt, &stm32f4_i2c_event_irq, bus);
  RTEMS_SYSLOG_ERROR_SC(rc, "I2C IRQ Event Disconnect");

  rc = rtems_interrupt_handler_remove(bus->info->irq_err, &stm32f4_i2c_error_irq, bus);
  RTEMS_SYSLOG_ERROR_SC(rc, "I2C IRQ Error Disconnect");

  i2c_bus_destroy(base);
}

int stm32f4_i2c_register(stm32f4_i2c_bus *bus, const stm32f4_i2c_bus_conf *conf)
{
  int err;
  uint32_t ref_Mhz = bsp_pclk(1)/1000000;
  rtems_status_code rc;

  if(ref_Mhz<2 || ref_Mhz>50) {
    return -ENOTSUP;
  }

  if(bus->conf && bus->conf==conf) {
    return 0;

  } else if(bus->conf) {
    return -EBUSY;

  } else if(!conf
            || conf->scl.fields.mode != STM32F4_GPIO_MODE_AF
            || conf->sda.fields.mode != STM32F4_GPIO_MODE_AF) {
    return -EINVAL;
  }

  bus->conf = conf;

  RTEMS_SYSLOG_PRINT("%s Initialize\n", bus->info->iname);

  // must enable clock before any access to I2C registers.
  stm32f4_rcc_set_clock(bus->info->rcc, true);

  if(bus->info->hw->cr1 & STM32F4_I2C_CR1_PE) {
    RTEMS_SYSLOG_ERROR("%s already initialized\n", bus->info->iname);
    return -EBUSY;
  }

  i2c_bus_init(&bus->base);
  // from this point, must de-init bus
  // rely on default ->destroy() until interrupt installed

  rc = rtems_interrupt_handler_install(bus->info->irq_evt, bus->info->iname,
                                       RTEMS_INTERRUPT_UNIQUE,
                                       &stm32f4_i2c_event_irq, bus);
  if(rc!=RTEMS_SUCCESSFUL) {
    RTEMS_SYSLOG_ERROR_WITH_SC(rc, "Failed to attach I2C Event IRQ");
    // only need to explicitly remove() here.
    // after error ISR attached, our destroy() will remove both ISRs
    (void)rtems_interrupt_handler_remove(bus->info->irq_evt, &stm32f4_i2c_event_irq, bus);
    err = -EIO;
    goto error;
  }

  rc = rtems_interrupt_handler_install(bus->info->irq_err, bus->info->iname,
                                       RTEMS_INTERRUPT_UNIQUE,
                                       &stm32f4_i2c_error_irq, bus);
  if(rc!=RTEMS_SUCCESSFUL) {
    RTEMS_SYSLOG_ERROR_WITH_SC(rc, "Failed to attach I2C Error IRQ");
    err = -EIO;
    goto error;
  }

  bus->base.transfer = stm32f4_i2c_transfer;
  bus->base.set_clock =  stm32f4_i2c_set_clock;
  bus->base.destroy = stm32f4_i2c_destroy;
  bus->base.functionality = I2C_FUNC_I2C;
  // arbitrary default timeout
  bus->base.timeout = RTEMS_MILLISECONDS_TO_TICKS(200u);

  stm32f4_i2c_reset(bus);

  /* The STM32F205 Errata mentions a possible timing violation during a
   * repeated START with standard mode speeds in the range 88KHz -> 100KHz.
   * So default to a slower speed.
   */
  err = stm32f4_i2c_set_clock(&bus->base, 80000);
  RTEMS_CLEANUP_RV(err, error, "I2C Clock Init");

  return i2c_bus_register(&bus->base, bus->info->fname);
error:
  (*bus->base.destroy)(&bus->base);
  return err;
}
