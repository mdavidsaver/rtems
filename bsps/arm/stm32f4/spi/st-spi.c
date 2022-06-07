/*
 * Copyright (c) 2022 Michael Davidsaver.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

//#define DEBUG

#include <string.h>
#include <errno.h>

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/spi.h>
#include <bsp/rcc.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>

#include <rtems/bspIo.h>
#include <rtems/error.h>
#include <rtems/status-checks.h>

#include <dev/spi/spi.h>

RTEMS_STATIC_ASSERT(offsetof(stm32f4_spi, i2spr)==0x20, "");

typedef struct {
    stm32f4_rcc_index rcc;
    volatile stm32f4_spi * const hw;
    const rtems_vector_number irq;
    unsigned pclk : 2;
    char iname[5];
    char *fname;
} spi_info_t;

struct stm32f4_spi_t {
    spi_bus base;

    const spi_info_t* const info;
    const stm32f4_spi_config_t* conf;

    uint16_t ss_pin;
    bool ss_active;

    rtems_id client;

    const uint8_t *tx_buf;
    uint8_t *rx_buf;
    size_t tx_remain;
    size_t rx_remain;
};

static
const spi_info_t spi1_info = {
    .rcc = STM32F4_RCC_SPI1,
    .hw = STM32F4_SPI1,
    .irq = STM32F4_IRQ_SPI1,
    .pclk = 2,
    .iname = "SPI1",
    .fname = STM32F4_SPI1_BUS_PATH,
};
stm32f4_spi_t stm32f4_spi1 = {
    .info = &spi1_info,
};

static
const spi_info_t spi2_info = {
    .rcc = STM32F4_RCC_SPI2,
    .hw = STM32F4_SPI2,
    .irq = STM32F4_IRQ_SPI2,
    .pclk = 1,
    .iname = "SPI2",
    .fname = STM32F4_SPI2_BUS_PATH,
};
stm32f4_spi_t stm32f4_spi2 = {
    .info = &spi2_info,
};

static
const spi_info_t spi3_info = {
    .rcc = STM32F4_RCC_SPI3,
    .hw = STM32F4_SPI3,
    .irq = STM32F4_IRQ_SPI3,
    .pclk = 1,
    .iname = "SPI3",
    .fname = STM32F4_SPI3_BUS_PATH,
};
stm32f4_spi_t stm32f4_spi3 = {
    .info = &spi3_info,
};

static
void stm32f4_spi_isr(void *raw)
{
    stm32f4_spi_t *bus = raw;
    volatile stm32f4_spi * const hw = bus->info->hw;
    uint32_t sr = hw->sr;

    // paranoia.  Errors should not be possible in plain SPI mode.
    if(sr & (STM32F4_SPI_SR_TIFRF|STM32F4_SPI_SR_MODF|STM32F4_SPI_SR_CRCERR)) {
        // Disable
        hw->cr2 &= ~(STM32F4_SPI_CR2_TXEIE|STM32F4_SPI_CR2_RXNEIE|STM32F4_SPI_CR2_ERRIE);
        hw->cr1 &= ~STM32F4_SPI_CR1_SPE;

        // wake task
        (void)rtems_event_transient_send(bus->client);
    }

    if(bus->tx_remain && (sr & STM32F4_SPI_SR_TXE)) {
        // ready to send next byte

        uint8_t v = 0u;
        if(bus->tx_buf)
            v = *bus->tx_buf++;
        bus->tx_remain--;

        hw->dr = v;
    }

    // If the CPU is slow enough, or this ISR is preempted,
    // it is possible to match all three conditions when
    // handling the last byte.
    sr = hw->sr;

    if(bus->rx_remain && (sr & STM32F4_SPI_SR_RXNE)) {
        // previous byte has been completely shifted in.

        uint8_t v = hw->dr;
        if(bus->rx_buf)
            *bus->rx_buf++ = v;
        bus->rx_remain--;

    }

    if(!bus->tx_remain && !bus->rx_remain) {
        // all bytes have now been completely shifted out.

        // disable
        hw->cr1 &= ~STM32F4_SPI_CR1_SPE;

        // wake task
        (void)rtems_event_transient_send(bus->client);
    }
}

static
int stm32f4_spi_transfer(spi_bus *base,
                         const spi_ioc_transfer *msgs,
                         uint32_t msg_count)
{
    int ret = -EIO;
    stm32f4_spi_t *bus = (stm32f4_spi_t*)base;
    const uint32_t ref_hz = bsp_pclk(bus->info->pclk);
    const rtems_interval us_per_tick = rtems_configuration_get_microseconds_per_tick();
    volatile stm32f4_spi * const hw = bus->info->hw;
    const stm32f4_spi_ss_config_t* cs_active = NULL;
    bool cs_force = false;

    if(!msg_count)
        return 0;

    if(hw->cr1 & STM32F4_SPI_CR1_SPE) {
        RTEMS_SYSLOG_ERROR("%s logic error during previous call?\n",
                           bus->info->iname);
        return -ENODEV;
    }

    bus->client = rtems_task_self();
    if(!bus->client) {
        RTEMS_SYSLOG_ERROR("%s whoami?\n",
                           bus->info->iname);
        return -ENOENT;
    }

    // we don't depend on the core to manage select(s),
    // so SSOE doesn't have any effect, but is required
    // or SR[TIFRF] results from any transfer.
    hw->cr2 |= STM32F4_SPI_CR2_SSOE;

    uint32_t m;
    for(m=0u; m<msg_count; m++) {
        const spi_ioc_transfer *msg = &msgs[m];

        // chip select handling

        if(bus->conf->num_selects) {
            if(!(msg->mode & SPI_NO_CS) && (msg->cs >= bus->conf->num_selects)) {
                RTEMS_SYSLOG_ERROR("%s msg[%u] invalid CS %u/%u\n",
                                   bus->info->iname,
                                   m,
                                   msg->cs,
                                   bus->conf->num_selects);
                break;
            }

            // de-select previous
            if(cs_active && (cs_force || cs_active-bus->conf->selects!=msg->cs)) {
                stm32f4_gpio_set_output(cs_active->pin.fields.pin_first,
                                        !cs_active->active);
                cs_active = NULL;
            }

            // select next
            if(!(msg->mode & SPI_NO_CS)) {
                cs_active = &bus->conf->selects[msg->cs];
                cs_force = msg->cs_change;

                stm32f4_gpio_set_output(cs_active->pin.fields.pin_first,
                                        cs_active->active);
            }
        }

        // bit clock setup
        // we will silently use a slower clock than requested,
        // but fail if requested clock is slower than can be achieved.

        uint32_t speed_hz = msg->speed_hz;
        if(!speed_hz)
            speed_hz = bus->base.speed_hz;

        if(!speed_hz) {
            speed_hz = ref_hz/256u; // slow default...

        } else if(speed_hz > ref_hz) {
            speed_hz = ref_hz;
        }

        uint32_t div = ref_hz / speed_hz;
        if(ref_hz % speed_hz)
            div++; // round to higher divider, lower frequency

        uint8_t br;
        if(div<=2) {
            br = 0;
        } else if(div<=4) {
            br = 1;
        } else if(div<=8) {
            br = 2;
        } else if(div<=16) {
            br = 3;
        } else if(div<=32) {
            br = 4;
        } else if(div<=64) {
            br = 5;
        } else if(div<=128) {
            br = 6;
        } else if(div<=256) {
            br = 7;
        } else {
            // requested speed is slower than we can achieve
            RTEMS_SYSLOG_ERROR("%s msg[%u] required clk div %u too low (> 256)\n",
                               bus->info->iname,
                               m,
                               div);
            ret = -EINVAL;
            break;
        }

        if(msg->len) {

            bus->tx_buf = msg->tx_buf;
            bus->rx_buf = msg->rx_buf;
            bus->tx_remain = bus->rx_remain = msg->len;

            uint32_t cr1 = hw->cr1;
            cr1 |= STM32F4_SPI_CR1_SPE|STM32F4_SPI_CR1_MSTR;

            cr1 &= ~(STM32F4_SPI_CR1_CPHA|STM32F4_SPI_CR1_CPOL);
            cr1 |= msg->mode & (SPI_CPHA|SPI_CPOL);

            RTEMS_STATIC_ASSERT((STM32F4_SPI_CR1_CPHA|STM32F4_SPI_CR1_CPOL)
                                ==(SPI_CPHA|SPI_CPOL),
                                "Happy bitmask coincidence");

            if(msg->mode&SPI_LSB_FIRST)
                cr1 |= STM32F4_SPI_CR1_LSBFIRST;
            else
                cr1 &= ~STM32F4_SPI_CR1_LSBFIRST;

            cr1 = STM32F4_SPI_CR1_BR_SET(cr1, br);

            hw->cr1 = cr1;
            // now enabled.

            hw->cr2 |= STM32F4_SPI_CR2_TXEIE|STM32F4_SPI_CR2_RXNEIE|STM32F4_SPI_CR2_ERRIE;

            rtems_status_code rc = rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);

            hw->cr2 &= ~(STM32F4_SPI_CR2_TXEIE|STM32F4_SPI_CR2_RXNEIE|STM32F4_SPI_CR2_ERRIE|STM32F4_SPI_CR2_SSOE);

            hw->cr1 &= ~STM32F4_SPI_CR1_SPE;

            if(rc!=RTEMS_SUCCESSFUL || bus->tx_remain || bus->rx_remain) {
                RTEMS_SYSLOG_ERROR("%s msg[%u] error %d with %zu/%zu\n",
                                   bus->info->iname,
                                   m,
                                   rc,
                                   bus->tx_remain,
                                   bus->rx_remain);
                break;
            }
        }

        if(msg->delay_usecs
                && rtems_task_wake_after((msg->delay_usecs+1u)/us_per_tick)!=RTEMS_SUCCESSFUL)
        {
            RTEMS_SYSLOG_ERROR("%s msg[%u] delay %u us error\n",
                               bus->info->iname,
                               m,
                               msg->delay_usecs);
            break;
        }
    }

    hw->cr2 &= ~STM32F4_SPI_CR2_SSOE;

    bus->client = 0;

    if(cs_active) {
        stm32f4_gpio_set_output(cs_active->pin.fields.pin_first,
                                !cs_active->active);
        cs_active = NULL;
    }

    if(m==msg_count) {
        // success!
        ret = 0;

    } else {
        RTEMS_DEBUG_PRINT("%s xfer error %d on msg %u/%u\n",
                          bus->info->iname,
                          ret,
                          m,
                          msg_count);
    }

    return ret;
}

static
int stm32f4_spi_setup(spi_bus *base)
{
    stm32f4_spi_t *bus = (stm32f4_spi_t*)base;
    uint32_t ref_hz = bsp_pclk(bus->info->pclk);

    if(bus->base.speed_hz < ref_hz/256
            || bus->base.bits_per_word!=8
            || bus->base.cs!=0
            || bus->base.mode & (SPI_3WIRE|SPI_LOOP|SPI_TX_DUAL|SPI_TX_QUAD|SPI_RX_DUAL|SPI_RX_QUAD)) {
        return -EINVAL;
    }

    return 0;
}

static
void stm32f4_spi_destroy(spi_bus *base)
{
    stm32f4_spi_t *bus = (stm32f4_spi_t*)base;
    rtems_status_code rc;

    // disable interrupts
    bus->info->hw->cr2 &= ~(STM32F4_SPI_CR2_TXEIE|STM32F4_SPI_CR2_RXNEIE|STM32F4_SPI_CR2_ERRIE);

    // should be redundant
    bus->info->hw->cr1 &= ~(STM32F4_SPI_CR1_SPE|STM32F4_SPI_CR1_MSTR);

    rc = rtems_interrupt_handler_remove(bus->info->irq, &stm32f4_spi_isr, bus);
    RTEMS_SYSLOG_ERROR_SC(rc, "I2C IRQ Event Disconnect");

    // TODO: de-init pins?

    spi_bus_destroy(&bus->base);
}

int stm32f4_spi_register(stm32f4_spi_t *bus,
                         const stm32f4_spi_config_t* conf)
{
    int err;
    rtems_status_code rc;

    if(bus->conf && bus->conf==conf) {
        return 0;

    } else if(bus->conf) {
        return -EINVAL;
    }

    stm32f4_rcc_set_clock(bus->info->rcc, true);

    stm32f4_gpio_set_config(&conf->sck);
    stm32f4_gpio_set_config(&conf->miso);
    stm32f4_gpio_set_config(&conf->mosi);

    for(uint8_t n=0u; n<conf->num_selects; n++) {
        const stm32f4_spi_ss_config_t *sel = &conf->selects[n];

        if(sel->pin.fields.mode != STM32F4_GPIO_MODE_OUTPUT)
            return -EINVAL;
        if(sel->pin.fields.pin_first != sel->pin.fields.pin_last)
            return -EINVAL;

        stm32f4_gpio_set_clock(sel->pin.fields.pin_first, true);
        stm32f4_gpio_set_output(sel->pin.fields.pin_first, !sel->active);
        stm32f4_gpio_set_config(&conf->miso);
    }

    spi_bus_init(&bus->base);
    // from this point, must de-init bus
    // rely on default ->destroy() until interrupt installed

    rc = rtems_interrupt_handler_install(bus->info->irq, bus->info->iname,
                                         RTEMS_INTERRUPT_UNIQUE,
                                         &stm32f4_spi_isr, bus);
    if(rc!=RTEMS_SUCCESSFUL) {
        RTEMS_SYSLOG_ERROR_WITH_SC(rc, "Failed to attach SPI Event IRQ");
        err = -EIO;
        goto error;
    }

    bus->base.setup = &stm32f4_spi_setup;
    bus->base.transfer = &stm32f4_spi_transfer;
    bus->base.destroy = &stm32f4_spi_destroy;
    bus->base.speed_hz = bsp_pclk(bus->info->pclk)/256;

    err = spi_bus_register(&bus->base, bus->info->fname);

    if(!err) {
        bus->conf = conf;
        return 0;
    }
error:
    (*bus->base.destroy)(&bus->base);
    return err;
}
