/*
 * Copyright (c) 2021 Michael Davidsaver  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <stdlib.h>

#include <bsp.h>
#include <bsp/io.h>
#include <bsp/irq.h>
#include <bsp/system-clocks.h>
#include <bsp/eeprom.h>

#include <rtems/error.h>
#include <rtems/imfs.h>

#define EE_CMD_READ8   (0)
#define EE_CMD_READ16  (1)
#define EE_CMD_READ32  (2)
#define EE_CMD_WRITE8  (3)
#define EE_CMD_WRITE16 (4)
#define EE_CMD_WRITE32 (5)
#define EE_CMD_ERASE   (6)
#define EE_CMD_RDPREFETCH BSP_BIT32(3)

#define EE_INT_RW_DONE BSP_BIT32(26)
#define EE_INT_PROG_DONE BSP_BIT32(28)

#define EE_PWRDWN_PWRDWN BSP_BIT32(0)

typedef struct {
    uint8_t _R0[0x080];
    uint32_t cmd;
    uint32_t addr;
    uint32_t wdata;
    uint32_t rdata;
    uint32_t wstate;
    uint32_t clkdiv;
    uint32_t pwrdwn;
    uint8_t _R1[0xfd8-0x098-4];
    uint32_t enclr;
    uint32_t enset;
    uint32_t stat;
    uint32_t inten;
    uint32_t statclr;
    uint32_t statset;
} lpc177x_eeprom;
RTEMS_STATIC_ASSERT(sizeof(lpc177x_eeprom)==0xff0, sizeof_lpc177x_eeprom);

#define EE (*(volatile lpc177x_eeprom*)0x00200000)

#define EEPROM_SIZE ((size_t)lpc24xx_eeprom_size)

extern const char lpc24xx_eeprom_size[];

static rtems_mutex eeprom_lock = RTEMS_MUTEX_INITIALIZER("EEPROM");
static rtems_id eeprom_client;

static void eeprom_isr(void* unused)
{
    // disable pending
    EE.enclr = EE.stat;
    rtems_event_transient_send(eeprom_client);
}

static void eeprom_wait_for_interrupt(uint32_t mask)
{
    eeprom_client = rtems_task_self();

    EE.enset = mask;

    while((EE.stat&mask)!=mask) {
        rtems_event_transient_receive(RTEMS_WAIT, 0);
    }

    EE.enclr = mask;
    EE.statclr = mask;

    eeprom_client = 0;
}

static size_t bound_count(off_t off, size_t count)
{
    const size_t total_size = EEPROM_SIZE;

    if((size_t)off > total_size)
        count = 0;
    else if(total_size - (size_t)off > count)
        count = total_size - (size_t)off;

    return count;
}

static ssize_t eeprom_read(
        rtems_libio_t *iop,
        void          *raw,
        size_t         count
      )
{
    uint8_t *buf = raw;
    off_t off = iop->offset;
    size_t n;

    if(!(count = bound_count(off, count)))
        return 0;

    rtems_mutex_lock(&eeprom_lock);

    EE.addr = off;
    /* byte-wise read with auto-increment */
    EE.cmd = EE_CMD_READ8 | EE_CMD_RDPREFETCH;

    for(n=0; n<count; n++) {
        eeprom_wait_for_interrupt(EE_INT_RW_DONE);
        buf[n] = EE.rdata;
    }

    rtems_mutex_unlock(&eeprom_lock);

    return count;
}

static ssize_t eeprom_write(
        rtems_libio_t *iop,
        const void    *raw,
        size_t         count
      )
{
    const uint8_t *buf = raw;
    off_t off = iop->offset;
    size_t n;

    if(!(count = bound_count(off, count)))
        return 0;

    rtems_mutex_lock(&eeprom_lock);

    EE.addr = off;
    /* byte-wise write with auto-increment */
    EE.cmd = EE_CMD_WRITE8;

    for(n=0; n<count; n++) {
        EE.wdata = buf[n];
        eeprom_wait_for_interrupt(EE_INT_RW_DONE);
    }

    rtems_mutex_unlock(&eeprom_lock);

    return count;
}

static int eeprom_stat(
        const rtems_filesystem_location_info_t *loc,
        struct stat *buf
      )
{
    int ret = IMFS_stat(loc, buf);
    if(!ret) {
        buf->st_size = EEPROM_SIZE;
        buf->st_blksize = 64u; // HW has a 64-byte buffer (aka. "page register")
    }
    return ret;
}

static const rtems_filesystem_file_handlers_r eeprom_handlers = {
    .open_h = rtems_filesystem_default_open,
    .close_h = rtems_filesystem_default_close,
    .read_h = eeprom_read,
    .write_h = eeprom_write,
    .ioctl_h = rtems_filesystem_default_ioctl,
    .lseek_h = rtems_filesystem_default_lseek,
    .fstat_h = eeprom_stat,
    .ftruncate_h = rtems_filesystem_default_ftruncate,
    .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
    .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
    .fcntl_h = rtems_filesystem_default_fcntl,
    .kqfilter_h = rtems_filesystem_default_kqfilter,
    .mmap_h = rtems_filesystem_default_mmap,
    .poll_h = rtems_filesystem_default_poll,
    .readv_h = rtems_filesystem_default_readv,
    .writev_h = rtems_filesystem_default_writev
};

static const IMFS_node_control eeprom_imfs_node_control =
        IMFS_GENERIC_CONTROL_INITIALIZER(&eeprom_handlers,
                                         IMFS_node_initialize_generic,
                                         IMFS_node_destroy_default);

int lpc24xx_register_eeprom(void)
{
    // clkdiv and wstate registers are "minus 1 encoded", which we ignore
    // to act as an implicit round up (to lower frequency)
    unsigned ref = LPC24XX_PCLK;
    uint32_t ph1 = ref / 28571428; // CCLK * 35ns
    uint32_t ph2 = ref / 18181818; // CCLK * 55ns
    uint32_t ph3 = ref / 66666666; // CCLK * 15ns
    rtems_status_code ec;
    int ret;

    if(EEPROM_SIZE==0)
        return -ENODEV; // Add lpc24xx_eeprom_size to start/linkcmds.$(RTEMS_BSP)

    ec = rtems_interrupt_handler_install(LPC24XX_IRQ_EEPROM, "EEPROM", RTEMS_INTERRUPT_UNIQUE,
                                         &eeprom_isr, NULL);
    if(ec!=RTEMS_SUCCESSFUL) {
        rtems_error(ec, "Error attaching ISR for EEPROM\n");
        return -EINVAL;
    }

    // power up (paranoia?)
    EE.pwrdwn = 0;

    // divide CCLK down to 375KHz
    EE.clkdiv = ref / 375000;

    EE.wstate = ph1<<16 | ph2<<8 | ph3;

    // clear any leftover interrupts (paranoia?)
    EE.statclr = EE.stat;

    ret = IMFS_make_generic_node(
      LPC24XX_EEPROM_PATH,
      S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
      &eeprom_imfs_node_control,
      NULL
    );

    if(ret)
        rtems_interrupt_handler_remove(LPC24XX_IRQ_EEPROM, &eeprom_isr, NULL);
    return ret;
}
